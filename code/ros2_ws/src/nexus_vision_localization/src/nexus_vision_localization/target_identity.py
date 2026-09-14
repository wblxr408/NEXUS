"""Per-instance image tracks with geometric appearance and ambiguity handling."""

from dataclasses import dataclass, field, replace
import hashlib

import cv2
import numpy as np

from .lightweight_identity import LightweightIdentityTransformer
from scipy.optimize import linear_sum_assignment

from .superpoint_frontend import FeatureSet, match_features, roi_mask


def _bbox(value):
    box = np.asarray(value, dtype=float)
    if box.shape != (4,) or not np.all(np.isfinite(box)) or min(box[2:]) <= 0:
        raise ValueError("target box requires finite x/y and positive width/height")
    return box.copy()


def appearance_descriptor(image, bbox, mask=None):
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("target appearance requires an 8-bit BGR image")
    selection = roi_mask((image.shape[1], image.shape[0]), bbox)
    if mask is not None:
        mask = np.asarray(mask)
        if mask.shape != selection.shape or mask.dtype != bool:
            raise ValueError("appearance mask must be same-image boolean pixels")
        selection &= mask
    if not selection.any():
        raise ValueError("target appearance ROI is outside the image or fully masked")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    histogram = cv2.calcHist([hsv], [0, 1], selection.astype(np.uint8), [16, 8], [0, 180, 0, 256]).reshape(-1)
    return histogram / histogram.sum()


def appearance_distance(first, second):
    return float(np.sqrt(max(0., 1. - np.sum(np.sqrt(first * second)))))


def _iou(first, second):
    low = np.maximum(first[:2], second[:2])
    high = np.minimum(first[:2] + first[2:], second[:2] + second[2:])
    intersection = float(np.prod(np.maximum(0., high - low)))
    return intersection / max(1e-12, np.prod(first[2:]) + np.prod(second[2:]) - intersection)


@dataclass(frozen=True)
class TargetCandidate:
    bbox_xywh_px: np.ndarray
    confidence: float
    features: FeatureSet
    appearance: np.ndarray
    class_id: int | None = None

    def __post_init__(self):
        object.__setattr__(self, "bbox_xywh_px", _bbox(self.bbox_xywh_px))
        if not isinstance(self.features, FeatureSet):
            raise TypeError("target candidate requires FeatureSet")
        if not np.isfinite(self.confidence) or not 0 < self.confidence <= 1:
            raise ValueError("target confidence must be in (0,1]")
        histogram = np.asarray(self.appearance, dtype=float)
        if (histogram.shape != (128,) or not np.all(np.isfinite(histogram)) or np.any(histogram < 0)
                or not np.isclose(histogram.sum(), 1., atol=1e-5)):
            raise ValueError("target appearance must be a normalized 128-bin histogram")
        object.__setattr__(self, "appearance", histogram.copy())


@dataclass(frozen=True)
class IdentityConfig:
    confirmation_hits: int = 3
    minimum_geometric_matches: int = 8
    maximum_motion_d2: float = 13.82
    association_threshold: float = .75
    ambiguity_margin: float = .08
    pixel_acceleration_sigma: float = 80.
    pixel_observation_sigma: float = 3.
    prediction_horizon_s: float = .5
    lost_after_s: float = .3
    automatic_retention_s: float = 5.
    maximum_tracks: int = 64
    attention_weight: float = .15

    def __post_init__(self):
        if (any(not np.isfinite(value) or value <= 0 for value in self.__dict__.values())
                or self.minimum_geometric_matches < 4 or self.association_threshold > 1
                or self.ambiguity_margin >= self.association_threshold
                or not 0 <= self.attention_weight <= .5
                or any(not isinstance(value, int) for value in (self.confirmation_hits, self.minimum_geometric_matches, self.maximum_tracks))):
            raise ValueError("invalid identity tracking configuration")


@dataclass
class _Track:
    track_id: str
    reference: TargetCandidate
    latest: TargetCandidate
    x: np.ndarray
    covariance: np.ndarray
    state_stamp_ns: int
    last_observed_ns: int | None
    registered: bool = False
    state: str = "tentative"
    hits: int = 1
    identity_confidence: float = .5
    ambiguous: bool = False
    observed: bool = False
    feature_inliers: int = 0
    was_confirmed: bool = False
    anchor_feature_id: int | None = None
    motion_model: str = "static"
    reference_id: str = ""
    identity_history: list = field(default_factory=list)
    temporal_kv_cache: list = field(default_factory=list)


@dataclass(frozen=True)
class IdentityObservation:
    track_id: str
    stamp_ns: int
    state: str
    observed: bool
    identity_confidence: float
    bbox_xywh_px: np.ndarray
    velocity_px_s: np.ndarray
    covariance: np.ndarray
    last_observed_ns: int | None
    identity_ambiguous: bool
    feature_inliers: int


def geometric_match(reference, candidate_features, minimum_matches=8, *, return_matches=False):
    """Validate a local image warp; this is not a general 3D pose estimate."""
    matches = match_features(reference.features, candidate_features)
    if len(matches.distances) < minimum_matches:
        return None
    first = reference.features.points_px[matches.previous_indices]
    second = candidate_features.points_px[matches.current_indices]
    homography, inliers = cv2.findHomography(first, second, cv2.RANSAC, 3.)
    if homography is None or inliers is None or not np.all(np.isfinite(homography)):
        return None
    keep = inliers.reshape(-1).astype(bool)
    if keep.sum() < minimum_matches or keep.mean() < .6:
        return None
    x, y, width, height = reference.bbox_xywh_px
    corners = np.array([[x, y], [x + width, y], [x + width, y + height], [x, y + height]], np.float32)
    mapped = cv2.perspectiveTransform(corners[None], homography)[0]
    if not np.all(np.isfinite(mapped)) or not cv2.isContourConvex(mapped):
        return None
    lower, upper = mapped.min(axis=0), mapped.max(axis=0)
    size = upper - lower
    if np.any(size <= 0) or not .05 <= np.prod(size) / (width * height) <= 20.:
        return None
    source_area = abs(cv2.contourArea(cv2.convexHull(first[keep]))) / (width * height)
    if source_area < .02:
        return None
    feature_cost = .5 * (1 - float(keep.mean())) + .5 * float(np.median(matches.distances[keep])) / .7
    result = (np.r_[lower, size], float(feature_cost), int(keep.sum()))
    if return_matches:
        return (*result, matches.previous_indices[keep], matches.current_indices[keep])
    return result


class TargetIdentityTracker:
    def __init__(self, config=None, attention=None):
        self.config = config or IdentityConfig()
        self.attention = attention or LightweightIdentityTransformer()
        self.tracks = {}
        self.last_stamp_ns = 0
        self._next_id = 1
        self.last_diagnostics = []

    def _new_track(self, identifier, candidate, stamp_ns, registered=False, destination=None):
        destination = self.tracks if destination is None else destination
        if not identifier or identifier in destination or len(destination) >= self.config.maximum_tracks:
            raise ValueError("target identity is empty, duplicated, or track capacity is exhausted")
        center = candidate.bbox_xywh_px[:2] + candidate.bbox_xywh_px[2:] / 2
        covariance = np.diag([self.config.pixel_observation_sigma ** 2] * 2 + [400.] * 2)
        if stamp_ns is None:
            covariance[:2, :2] = np.eye(2) * 1e6  # no live image position known yet
        track = _Track(identifier, candidate, candidate, np.r_[center, 0., 0.], covariance,
                       stamp_ns or 0, stamp_ns, registered, hits=int(stamp_ns is not None), observed=stamp_ns is not None)
        if len(candidate.features.points_px):
            track.anchor_feature_id = int(np.argmin(np.linalg.norm(candidate.features.points_px - center, axis=1)))
        track.reference_id = self._reference_id(candidate)
        if track.hits >= self.config.confirmation_hits:
            track.state, track.was_confirmed = "confirmed", True
        destination[identifier] = track
        return track

    @staticmethod
    def _reference_id(candidate):
        return hashlib.sha256(candidate.features.points_px.tobytes() + candidate.features.descriptors.tobytes()).hexdigest()

    def register_reference(self, track_id, candidate, stamp_ns=None, existing_track_id=None,
                           motion_model="static", anchor_reference_px=None):
        if not isinstance(track_id, str) or not track_id.strip():
            raise ValueError("reference target ID must be a nonempty string")
        if stamp_ns is not None and (not isinstance(stamp_ns, int) or stamp_ns <= 0 or stamp_ns < self.last_stamp_ns):
            raise ValueError("live ROI reference timestamp must not precede the tracker")
        if motion_model != "static":
            raise ValueError("this project requires a static map target")
        anchor = (candidate.bbox_xywh_px[:2] + candidate.bbox_xywh_px[2:] / 2 if anchor_reference_px is None
                  else np.asarray(anchor_reference_px, dtype=float))
        if anchor.shape != (2,) or not np.all(np.isfinite(anchor)):
            raise ValueError("anchor_reference_px must contain two finite coordinates")
        anchor_id = int(np.argmin(np.linalg.norm(candidate.features.points_px - anchor, axis=1))) if len(candidate.features.points_px) else None
        if existing_track_id is not None:
            if existing_track_id not in self.tracks or (track_id in self.tracks and track_id != existing_track_id):
                raise ValueError("existing target identity is missing or requested ID is already in use")
            existing = self.tracks.pop(existing_track_id)
            self.tracks[track_id] = replace(existing, track_id=track_id, reference=candidate, registered=True,
                                            anchor_feature_id=anchor_id, motion_model=motion_model, reference_id=self._reference_id(candidate))
            return track_id
        result = self._new_track(str(track_id), candidate, stamp_ns, registered=True)
        result.anchor_feature_id, result.motion_model = anchor_id, motion_model
        return result.track_id

    def reference_candidates(self, features, image):
        """Locate registered textured references even without category detections."""
        output = []
        for track in self.tracks.values():
            if not track.registered:
                continue
            match = geometric_match(track.reference, features, self.config.minimum_geometric_matches)
            if match is None:
                continue
            box, cost, _ = match
            upper = np.minimum(box[:2] + box[2:], np.asarray(features.image_size_wh))
            box[:2] = np.maximum(box[:2], 0.)
            box[2:] = upper - box[:2]
            if np.any(box[2:] <= 0) or any(_iou(box, item.bbox_xywh_px) > .8 for item in output):
                continue
            output.append(TargetCandidate(box, max(.01, 1 - cost), features.in_roi(box),
                                          appearance_descriptor(image, box), track.reference.class_id))
        return output

    def _predict(self, track, stamp_ns, camera_homography):
        if track.state_stamp_ns == 0:
            track.state_stamp_ns = stamp_ns
            return
        elapsed = (stamp_ns - track.state_stamp_ns) / 1e9
        remaining = ((track.last_observed_ns or track.state_stamp_ns) - track.state_stamp_ns) / 1e9 + self.config.prediction_horizon_s
        dt = max(0., min(elapsed, remaining))
        transition = np.eye(4)
        transition[:2, 2:] = np.eye(2) * dt
        track.x = transition @ track.x
        injection = np.vstack((np.eye(2) * elapsed ** 2 / 2, np.eye(2) * elapsed))
        track.covariance = transition @ track.covariance @ transition.T + injection @ injection.T * self.config.pixel_acceleration_sigma ** 2
        if camera_homography is not None:
            homogeneous = camera_homography @ np.r_[track.x[:2], 1.]
            if abs(homogeneous[2]) < 1e-8:
                raise ValueError("camera motion sends a target prediction to infinity")
            center = homogeneous[:2] / homogeneous[2]
            derivative = (camera_homography[:2, :2] - center[:, None] * camera_homography[2, :2]) / homogeneous[2]
            jacobian = np.zeros((4, 4))
            jacobian[:2, :2] = jacobian[2:, 2:] = derivative
            track.x[:2], track.x[2:] = center, derivative @ track.x[2:]
            track.covariance = jacobian @ track.covariance @ jacobian.T
        track.state_stamp_ns, track.observed = stamp_ns, False

    def _cost(self, track, candidate):
        if (track.reference.class_id is not None and candidate.class_id is not None
                and track.reference.class_id != candidate.class_id):
            return np.inf, 0
        gap = np.inf if track.last_observed_ns is None else (track.state_stamp_ns - track.last_observed_ns) / 1e9
        use_reference = track.ambiguous or track.state == "lost" or gap > self.config.lost_after_s
        template = track.reference if use_reference else track.latest
        match = geometric_match(template, candidate.features, self.config.minimum_geometric_matches)
        appearance = appearance_distance(template.appearance, candidate.appearance)
        center = candidate.bbox_xywh_px[:2] + candidate.bbox_xywh_px[2:] / 2
        residual = center - track.x[:2]
        covariance = track.covariance[:2, :2] + np.eye(2) * self.config.pixel_observation_sigma ** 2
        motion_d2 = float(residual @ np.linalg.solve(covariance, residual))
        if not use_reference and motion_d2 > self.config.maximum_motion_d2:
            return np.inf, 0
        if use_reference and match is None and appearance > .15:
            return np.inf, 0
        # Re-identification of a user-supplied reference image needs texture,
        # since color alone cannot establish which object was designated.
        if track.last_observed_ns is None and match is None:
            return np.inf, 0
        scale_cost = min(1., float(np.linalg.norm(np.log(candidate.bbox_xywh_px[2:] / template.bbox_xywh_px[2:]))) / 2)
        attention = self.attention.compare(track.reference.features, candidate.features, track.identity_history,
                                           temporal_cache=track.temporal_kv_cache)
        attention_cost = 1 - attention.score
        terms = [(appearance, .25), (scale_cost, .1)]
        inliers = 0
        if match is not None:
            warped_box, feature_cost, inliers = match
            terms.extend([(feature_cost, .3), (1 - _iou(warped_box, candidate.bbox_xywh_px), .1)])
        elif len(template.features.points_px) >= self.config.minimum_geometric_matches and len(candidate.features.points_px) >= self.config.minimum_geometric_matches:
            return np.inf, 0  # textured but geometrically incompatible
        if not use_reference and track.last_observed_ns is not None:
            terms.append((min(1., motion_d2 / self.config.maximum_motion_d2), .25))
        if self.config.attention_weight:
            terms.append((attention_cost, self.config.attention_weight))
        return sum(value * weight for value, weight in terms) / sum(weight for _, weight in terms), inliers

    def update(self, stamp_ns, candidates, camera_homography=None):
        if not isinstance(stamp_ns, int) or stamp_ns <= self.last_stamp_ns:
            raise ValueError("target frame timestamps must be strictly increasing")
        candidates = list(candidates)
        if any(not isinstance(item, TargetCandidate) for item in candidates):
            raise TypeError("identity tracker requires TargetCandidate inputs")
        if any(stamp_ns < track.state_stamp_ns for track in self.tracks.values()):
            raise ValueError("target frame precedes a registered live reference")
        if camera_homography is not None:
            camera_homography = np.asarray(camera_homography, dtype=float)
            if camera_homography.shape != (3, 3) or not np.all(np.isfinite(camera_homography)) or abs(np.linalg.det(camera_homography)) < 1e-9:
                raise ValueError("camera image transform must be finite and nonsingular")
        working = {identifier: replace(track, x=track.x.copy(), covariance=track.covariance.copy(),
                                       identity_history=list(track.identity_history), temporal_kv_cache=list(track.temporal_kv_cache))
                   for identifier, track in self.tracks.items()}
        for identifier, track in list(working.items()):
            if not track.registered and track.last_observed_ns is not None and (stamp_ns - track.last_observed_ns) / 1e9 > self.config.automatic_retention_s:
                del working[identifier]
        tracks = list(working.values())
        for track in tracks:
            self._predict(track, stamp_ns, camera_homography)
        cost = np.full((len(tracks), len(candidates)), np.inf)
        inliers = np.zeros_like(cost, dtype=int)
        for i, track in enumerate(tracks):
            for j, candidate in enumerate(candidates):
                cost[i, j], inliers[i, j] = self._cost(track, candidate)
        ambiguous_tracks, ambiguous_detections = set(), set()
        threshold = self.config.association_threshold
        for i in range(len(tracks)):
            ordered = np.argsort(cost[i])
            if len(ordered) >= 2 and cost[i, ordered[1]] < threshold and cost[i, ordered[1]] - cost[i, ordered[0]] < self.config.ambiguity_margin:
                ambiguous_tracks.add(i)
                ambiguous_detections.update(int(j) for j in ordered if cost[i, j] - cost[i, ordered[0]] < self.config.ambiguity_margin)
        for j in range(len(candidates)):
            ordered = np.argsort(cost[:, j])
            if len(ordered) >= 2 and cost[ordered[1], j] < threshold and cost[ordered[1], j] - cost[ordered[0], j] < self.config.ambiguity_margin:
                ambiguous_detections.add(j)
                ambiguous_tracks.update(int(i) for i in ordered if cost[i, j] - cost[ordered[0], j] < self.config.ambiguity_margin)
        for i in ambiguous_tracks:
            tracks[i].ambiguous, tracks[i].identity_confidence = True, .25
            tracks[i].covariance *= 4.
            cost[i] = np.inf
        for j in ambiguous_detections:
            cost[:, j] = np.inf
        diagnostics = ([{"decision": "identity_ambiguous", "track_ids": [tracks[i].track_id for i in sorted(ambiguous_tracks)],
                         "candidate_indices": sorted(ambiguous_detections)}] if ambiguous_tracks else [])
        assigned_tracks, used = set(), set()
        if tracks:
            augmented = np.column_stack((cost, np.full((len(tracks), len(tracks)), threshold)))
            rows, columns = linear_sum_assignment(augmented)
            for i, j in zip(rows, columns):
                if j >= len(candidates) or cost[i, j] >= threshold:
                    continue
                track, candidate = tracks[i], candidates[j]
                gap = np.inf if track.last_observed_ns is None else (stamp_ns - track.last_observed_ns) / 1e9
                reacquired = track.ambiguous or track.state in {"lost", "degraded"} or gap > self.config.lost_after_s
                fresh = track.last_observed_ns != stamp_ns
                center = candidate.bbox_xywh_px[:2] + candidate.bbox_xywh_px[2:] / 2
                if reacquired:
                    track.x = np.r_[center, 0., 0.]
                    track.covariance = np.diag([self.config.pixel_observation_sigma ** 2] * 2 + [400.] * 2)
                elif fresh:
                    measurement_covariance = np.eye(2) * self.config.pixel_observation_sigma ** 2 / candidate.confidence
                    gain = np.linalg.solve(track.covariance[:2, :2] + measurement_covariance, track.covariance[:, :2].T).T
                    track.x += gain @ (center - track.x[:2])
                    correction = np.eye(4)
                    correction[:, :2] -= gain
                    track.covariance = correction @ track.covariance @ correction.T + gain @ measurement_covariance @ gain.T
                track.latest, track.last_observed_ns, track.observed = candidate, stamp_ns, True
                embedding, _ = self.attention.embed(candidate.features)
                track.identity_history.append(embedding)
                del track.identity_history[:-self.attention.maximum_history]
                track.temporal_kv_cache.append(self.attention.kv_cache(candidate.features))
                del track.temporal_kv_cache[:-self.attention.maximum_history]
                track.hits += int(fresh)
                track.feature_inliers = int(inliers[i, j])
                track.ambiguous = False
                track.identity_confidence = float(np.clip(1 - cost[i, j], .5, 1.))
                track.state = ("re-associated" if reacquired and track.was_confirmed
                               else "confirmed" if track.hits >= self.config.confirmation_hits else "tentative")
                track.was_confirmed |= track.hits >= self.config.confirmation_hits
                assigned_tracks.add(i)
                used.add(int(j))
        for i, track in enumerate(tracks):
            if i in assigned_tracks:
                continue
            gap = np.inf if track.last_observed_ns is None else (stamp_ns - track.last_observed_ns) / 1e9
            track.state = "lost" if gap > self.config.lost_after_s else "degraded"
            track.hits = 0
            track.identity_confidence = min(track.identity_confidence, .25 if track.ambiguous else .5)
            track.feature_inliers = 0
        for j, candidate in enumerate(candidates):
            if j in used or j in ambiguous_detections:
                continue
            if len(working) >= self.config.maximum_tracks:
                diagnostics.append({"decision": "unassigned", "reason": "track_capacity", "candidate_index": j})
                continue
            while f"track_{self._next_id:04d}" in working:
                self._next_id += 1
            self._new_track(f"track_{self._next_id:04d}", candidate, stamp_ns, destination=working)
            self._next_id += 1
        self.tracks, self.last_diagnostics = working, diagnostics
        self.last_stamp_ns = stamp_ns
        return self.observations(stamp_ns)

    def observations(self, stamp_ns=None):
        stamp_ns = self.last_stamp_ns if stamp_ns is None else stamp_ns
        return [IdentityObservation(track.track_id, stamp_ns, track.state, track.observed,
                                    track.identity_confidence, np.r_[track.x[:2] - track.latest.bbox_xywh_px[2:] / 2, track.latest.bbox_xywh_px[2:]],
                                    track.x[2:].copy(), track.covariance.copy(), track.last_observed_ns,
                                    track.ambiguous, track.feature_inliers) for track in self.tracks.values()]
