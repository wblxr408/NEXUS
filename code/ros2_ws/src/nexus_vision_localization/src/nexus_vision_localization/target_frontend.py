"""Use one dense image inference for detected and reference-conditioned targets."""

from copy import copy

import numpy as np

from .superpoint_frontend import roi_mask
from .target_identity import IdentityConfig, TargetCandidate, TargetIdentityTracker, appearance_descriptor, geometric_match
from .lightweight_identity import LightweightIdentityTransformer


def _overlap(first, second):
    low = np.maximum(first[:2], second[:2])
    high = np.minimum(first[:2] + first[2:], second[:2] + second[2:])
    intersection = np.prod(np.maximum(0., high - low))
    return intersection / max(1e-12, np.prod(first[2:]) + np.prod(second[2:]) - intersection)


class TargetVisualFrontend:
    def __init__(self, config=None, identity_transformer_checkpoint=""):
        transformer = LightweightIdentityTransformer(identity_transformer_checkpoint or None)
        self.tracker = TargetIdentityTracker(config or IdentityConfig(), attention=transformer)

    def configure_compute_budget(self, *, maximum_rois, transformer_tokens, transformer_history):
        """Apply a bounded runtime budget without changing model coordinates."""
        if not all(isinstance(value, int) and value >= 0
                   for value in (maximum_rois, transformer_tokens, transformer_history)):
            raise ValueError("target compute budget must contain nonnegative integers")
        if not 1 <= maximum_rois <= 32 or not 4 <= transformer_tokens <= 128 or not 1 <= transformer_history <= 32:
            raise ValueError("target compute budget is outside supported limits")
        attention = self.tracker.attention
        attention.maximum_tokens = transformer_tokens
        attention.maximum_history = transformer_history
        self.maximum_rois = maximum_rois

    def register(self, identifier, dense, image, bbox_xywh_px, feature_options, *, class_id=None, existing_track_id=None,
                 motion_model="static", anchor_reference_px=None):
        size = (image.shape[1], image.shape[0])
        selected = dense.features(roi_mask(size, bbox_xywh_px), **feature_options)
        if len(selected.points_px) < self.tracker.config.minimum_geometric_matches:
            raise ValueError("reference ROI has insufficient stable features")
        candidate = TargetCandidate(bbox_xywh_px, 1., selected, appearance_descriptor(image, bbox_xywh_px), class_id)
        return self.tracker.register_reference(identifier, candidate, existing_track_id=existing_track_id,
                                               motion_model=motion_model, anchor_reference_px=anchor_reference_px)

    def prepare(self, dense, image, stamp_ns, detections, feature_options, camera_homography=None,
                maximum_rois=None, image_quality=None):
        """Return a candidate tracker; caller commits only while frame is fresh."""
        size = (image.shape[1], image.shape[0])
        candidates = []
        budget = getattr(self, "maximum_rois", 32) if maximum_rois is None else maximum_rois
        if not isinstance(budget, int) or not 1 <= budget <= 32:
            raise ValueError("maximum_rois must be an integer in [1, 32]")
        # Detection order is not a confidence contract.  Spend the ROI budget on
        # the strongest candidates deterministically, then retain registered
        # reference proposals below.
        ranked = sorted(detections, key=lambda item: (-float(item["confidence"]), int(item["class_id"])))[:budget]
        for detection in ranked:
            bbox = np.asarray(detection["bbox_xywh_px"], dtype=float)
            if not roi_mask(size, bbox).any():
                continue
            features = dense.features(roi_mask(size, bbox), **feature_options)
            candidates.append(TargetCandidate(bbox, float(detection["confidence"]), features,
                                              appearance_descriptor(image, bbox), int(detection["class_id"])))
        if any(track.registered for track in self.tracker.tracks.values()):
            full = dense.features(np.ones(size[::-1], bool), **feature_options)
            for candidate in self.tracker.reference_candidates(full, image):
                if not any(_overlap(candidate.bbox_xywh_px, existing.bbox_xywh_px) > .7 for existing in candidates):
                    candidates.append(candidate)
        pending = copy(self.tracker)
        observations = pending.update(stamp_ns, candidates, camera_homography)
        exclusions = [candidate.bbox_xywh_px for candidate in candidates]
        for observation in observations:
            if observation.last_observed_ns is None:
                continue
            gap = (stamp_ns - observation.last_observed_ns) / 1e9
            if not observation.observed and gap <= pending.config.prediction_horizon_s:
                # Retain a conservative mask for a briefly lost designated
                # object, so its features do not become platform landmarks.
                margin = 3 * np.sqrt(np.maximum(0., np.diag(observation.covariance)[:2]))
                margin = np.minimum(margin, np.asarray(size) / 2)
                box = observation.bbox_xywh_px.copy()
                box[:2] -= margin
                box[2:] += 2 * margin
                exclusions.append(box)
        return pending, observations, exclusions

    @staticmethod
    def packet(stamp_ns, frame_id, image_size_wh, observations, diagnostics=(), *, expired=False, tracker=None,
               image_quality=None, exposure_rotation_sigma_rad=None):
        tracks = []
        for item in observations:
            geometry = {}
            if tracker is not None:
                track = tracker.tracks[item.track_id]
                reference_pixel = (None if track.anchor_feature_id is None else
                                   track.reference.features.points_px[track.anchor_feature_id].tolist())
                geometry = {"reference_id": track.reference_id, "motion_model": track.motion_model,
                            "anchor_feature_id": track.anchor_feature_id, "anchor_reference_px": reference_pixel,
                            "reference_feature_count": len(track.reference.features.points_px),
                            "feature_observations": []}
                if item.observed and not item.identity_ambiguous and not expired:
                    match = geometric_match(track.reference, track.latest.features, tracker.config.minimum_geometric_matches, return_matches=True)
                    if match is not None:
                        geometry["feature_observations"] = [
                            {"feature_id": int(first), "pixel_px": track.latest.features.points_px[second].tolist(),
                             "reference_pixel_px": track.reference.features.points_px[first].tolist()}
                            for first, second in zip(match[3], match[4])]
            tracks.append({"track_id": item.track_id, "state": "lost" if expired else item.state,
                           "observed": item.observed and not expired, "identity_confidence": item.identity_confidence,
                           "identity_ambiguous": item.identity_ambiguous, "bbox_xywh_px": item.bbox_xywh_px.tolist(),
                           "velocity_px_s": item.velocity_px_s.tolist(), "covariance_px": item.covariance.reshape(-1).tolist(),
                           "last_observed_ns": item.last_observed_ns, "feature_inliers": item.feature_inliers,
                           "world_position_observed": False, "orientation_observed": False, **geometry})
        packet = {"schema_version": 1, "sample_timestamp_ns": stamp_ns, "frame_id": frame_id,
                "image_size_wh": list(image_size_wh), "unit": "px", "valid": not expired,
                "tracks": tracks, "diagnostics": list(diagnostics)}
        if image_quality is not None:
            packet["image_quality"] = float(image_quality)
        if exposure_rotation_sigma_rad is not None:
            packet["exposure_rotation_sigma_rad"] = float(exposure_rotation_sigma_rad)
        return packet
