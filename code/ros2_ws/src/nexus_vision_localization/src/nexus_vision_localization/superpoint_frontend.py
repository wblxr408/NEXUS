"""Local dense SuperPoint GPU inference and geometry-neutral correspondences.

The network asset is external and mandatory. Postprocessing follows the
explicit dense-head and descriptor sampling contract in the architecture doc.
No alternate feature detector is silently substituted when a model is absent.
"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from nexus_vision_localization.edge_model import create_model


@dataclass(frozen=True)
class FeatureSet:
    points_px: np.ndarray
    descriptors: np.ndarray
    scores: np.ndarray
    image_size_wh: tuple

    def __post_init__(self):
        points = np.asarray(self.points_px, dtype=np.float32)
        descriptors = np.asarray(self.descriptors, dtype=np.float32)
        scores = np.asarray(self.scores, dtype=np.float32)
        if (points.ndim != 2 or points.shape[1] != 2 or descriptors.shape != (len(points), 256)
                or scores.shape != (len(points),) or not all(np.all(np.isfinite(v)) for v in (points, descriptors, scores))):
            raise ValueError("features require finite Nx2 pixels, Nx256 descriptors and N scores")
        if len(self.image_size_wh) != 2 or any(not isinstance(v, (int, np.integer)) or v <= 0 for v in self.image_size_wh):
            raise ValueError("features require positive integer image dimensions")
        width, height = self.image_size_wh
        if (np.any(points < 0) or np.any(points[:, 0] >= width) or np.any(points[:, 1] >= height)
                or np.any(scores < 0) or np.any(scores > 1)):
            raise ValueError("feature coordinates/scores outside their image/probability bounds")
        if len(points) and not np.allclose(np.linalg.norm(descriptors, axis=1), 1., atol=1e-4):
            raise ValueError("feature descriptors must be L2 normalized")
        object.__setattr__(self, "points_px", points.copy())
        object.__setattr__(self, "descriptors", descriptors.copy())
        object.__setattr__(self, "scores", scores.copy())

    def select(self, selection):
        return FeatureSet(self.points_px[selection], self.descriptors[selection],
                          self.scores[selection], self.image_size_wh)

    def in_roi(self, bbox_xywh):
        mask = roi_mask(self.image_size_wh, bbox_xywh)
        pixels = np.floor(self.points_px + .5).astype(int)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, self.image_size_wh[0] - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, self.image_size_wh[1] - 1)
        return self.select(mask[pixels[:, 1], pixels[:, 0]])


def roi_mask(image_size_wh, bbox_xywh):
    width, height = image_size_wh
    bbox = np.asarray(bbox_xywh, dtype=float)
    if bbox.shape != (4,) or not np.all(np.isfinite(bbox)) or np.any(bbox[2:] <= 0):
        raise ValueError("ROI requires finite x/y and positive width/height")
    x0, y0 = np.floor(bbox[:2]).astype(int)
    x1, y1 = np.ceil(bbox[:2] + bbox[2:]).astype(int)
    mask = np.zeros((height, width), dtype=bool)
    mask[np.clip(y0, 0, height):np.clip(y1, 0, height),
         np.clip(x0, 0, width):np.clip(x1, 0, width)] = True
    return mask


def static_background_mask(image_size_wh, dynamic_boxes=(), dynamic_mask=None, margin_px=4):
    """Remove dynamic/target pixels before selecting background keypoints."""
    width, height = image_size_wh
    if int(margin_px) != margin_px or margin_px < 0:
        raise ValueError("mask margin must be a nonnegative integer")
    excluded = np.zeros((height, width), dtype=bool)
    if dynamic_mask is not None:
        dynamic_mask = np.asarray(dynamic_mask)
        if dynamic_mask.shape != excluded.shape or dynamic_mask.dtype != bool:
            raise ValueError("dynamic mask must be same-image boolean pixels")
        excluded |= dynamic_mask
    for bbox in dynamic_boxes:
        excluded |= roi_mask(image_size_wh, bbox)
    if margin_px:
        size = 2 * int(margin_px) + 1
        excluded = cv2.dilate(excluded.astype(np.uint8), np.ones((size, size), np.uint8)).astype(bool)
    return ~excluded


@dataclass(frozen=True)
class DenseSuperPoint:
    logits: np.ndarray
    descriptor_map: np.ndarray
    image_size_wh: tuple
    descriptor_sampling: str = "lightglue_v1"

    def __post_init__(self):
        if self.descriptor_sampling not in {"lightglue_v1", "superpoint_mit_v1"}:
            raise ValueError("unsupported descriptor sampling")
        logits = np.asarray(self.logits, dtype=np.float32)
        descriptors = np.asarray(self.descriptor_map, dtype=np.float32)
        if len(self.image_size_wh) != 2 or any(not isinstance(v, (int, np.integer)) or v <= 0 for v in self.image_size_wh):
            raise ValueError("dense features require positive integer image dimensions")
        if (logits.ndim != 4 or logits.shape[:2] != (1, 65) or min(logits.shape[2:]) < 1
                or descriptors.shape != (1, 256, *logits.shape[2:])
                or not np.all(np.isfinite(logits)) or not np.all(np.isfinite(descriptors))):
            raise ValueError("SuperPoint dense model must output finite 65-channel logits and 256-channel descriptor maps")
        object.__setattr__(self, "logits", logits.copy())
        norms = np.linalg.norm(descriptors, axis=1, keepdims=True)
        object.__setattr__(self, "descriptor_map", descriptors / np.maximum(norms, 1e-12))

    def _sample_descriptors(self, points):
        # lightglue_v1 grid convention: pixel -> normalized grid -> coarse
        # coordinate with align_corners=True. Interpolation uses zero padding.
        _, _, height, width = self.descriptor_map.shape
        scale = np.array([width - 1, height - 1], dtype=float)
        denominator = np.array([width * 8 - 4.5, height * 8 - 4.5])
        coarse = ((points + .5) / 8 - .5 if self.descriptor_sampling == "superpoint_mit_v1"
                  else (points - 3.5) / denominator * scale)
        low = np.floor(coarse).astype(int)
        fraction = coarse - low
        output = np.zeros((len(points), 256), dtype=np.float32)
        for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
            x, y = low[:, 0] + dx, low[:, 1] + dy
            valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
            weights = (fraction[:, 0] if dx else 1 - fraction[:, 0]) * (fraction[:, 1] if dy else 1 - fraction[:, 1])
            output[valid] += weights[valid, None] * self.descriptor_map[0, :, y[valid], x[valid]]
        norm = np.linalg.norm(output, axis=1)
        valid = norm > 1e-8
        output[valid] /= norm[valid, None]
        return output, valid

    def features(self, valid_mask=None, *, threshold=.015, nms_radius=4, maximum_points=1024, border_px=4):
        if (not np.isfinite(threshold) or not 0 < threshold < 1 or int(nms_radius) != nms_radius
                or int(maximum_points) != maximum_points or int(border_px) != border_px
                or min(nms_radius, border_px) < 0 or maximum_points < 1):
            raise ValueError("invalid SuperPoint feature selection settings")
        logits = self.logits[0]
        probabilities = np.exp(logits - np.max(logits, axis=0, keepdims=True))
        probabilities /= probabilities.sum(axis=0, keepdims=True)
        coarse_height, coarse_width = logits.shape[1:]
        height, width = coarse_height * 8, coarse_width * 8
        heatmap = probabilities[:64].transpose(1, 2, 0).reshape(coarse_height, coarse_width, 8, 8)
        heatmap = heatmap.transpose(0, 2, 1, 3).reshape(height, width).copy()
        allowed = np.ones((height, width), dtype=bool)
        if valid_mask is not None:
            valid_mask = np.asarray(valid_mask)
            if valid_mask.dtype != bool or valid_mask.shape != self.image_size_wh[::-1]:
                raise ValueError("feature mask must be a boolean mask in original image coordinates")
            # Map candidate pixel centers, rather than resizing the mask with
            # a potentially different nearest-neighbour coordinate convention.
            xs = np.minimum(((np.arange(width) + .5) * self.image_size_wh[0] / width).astype(int), self.image_size_wh[0] - 1)
            ys = np.minimum(((np.arange(height) + .5) * self.image_size_wh[1] / height).astype(int), self.image_size_wh[1] - 1)
            allowed = valid_mask[np.ix_(ys, xs)]
        if border_px:
            allowed[:border_px] = allowed[-border_px:] = False
            allowed[:, :border_px] = allowed[:, -border_px:] = False
        heatmap[~allowed] = -1.
        radius = int(nms_radius)
        maximum = cv2.dilate(heatmap, np.ones((radius * 2 + 1, radius * 2 + 1), np.uint8))
        ys, xs = np.where((heatmap >= threshold) & (heatmap == maximum))
        order = np.argsort(-heatmap[ys, xs], kind="stable")
        suppressed, selected = np.zeros_like(allowed), []
        for index in order:
            x, y = xs[index], ys[index]
            if suppressed[y, x]:
                continue
            selected.append(index)
            suppressed[max(0, y - radius):y + radius + 1, max(0, x - radius):x + radius + 1] = True
            if len(selected) == maximum_points:
                break
        selected = np.asarray(selected, dtype=int)
        points = np.column_stack((xs[selected], ys[selected])).astype(np.float32)
        scores = heatmap[ys[selected], xs[selected]]
        descriptors, valid = self._sample_descriptors(points)
        points, descriptors, scores = points[valid], descriptors[valid], scores[valid]
        scale = np.asarray(self.image_size_wh, dtype=float) / [width, height]
        points = (points + .5) * scale - .5
        # A user-selected border of zero can map the first upsampled center
        # slightly outside the source image. Such points have no source pixel.
        valid = (points[:, 0] >= 0) & (points[:, 1] >= 0)
        return FeatureSet(points[valid], descriptors[valid], scores[valid], self.image_size_wh)


class SuperPointOnnx:
    """Fixed input size, explicit output names, and checksum-verified weights."""

    def __init__(self, manifest_path):
        path = Path(manifest_path)
        with path.open(encoding="utf-8") as stream:
            manifest = json.load(stream)
        required = {"schema_version", "architecture", "model_file", "sha256", "input_name", "detector_output",
                    "descriptor_output", "input_size_wh", "descriptor_sampling", "source", "license"}
        if not isinstance(manifest, dict) or not required.issubset(manifest):
            raise ValueError("SuperPoint model manifest is incomplete")
        if (manifest["schema_version"] != 1 or manifest["architecture"] != "superpoint_dense_v1"
                or manifest["descriptor_sampling"] not in {"lightglue_v1", "superpoint_mit_v1"}):
            raise ValueError("unsupported SuperPoint model/head/sampling format")
        size = manifest["input_size_wh"]
        if (not isinstance(size, list) or len(size) != 2
                or any(isinstance(v, bool) or not isinstance(v, int) or v < 8 or v % 8 for v in size)):
            raise ValueError("SuperPoint input dimensions must be positive multiples of eight")
        if "dynamic_input" in manifest and not isinstance(manifest["dynamic_input"], bool):
            raise ValueError("SuperPoint dynamic_input must be boolean when declared")
        if any(not isinstance(manifest[name], str) or not manifest[name].strip()
               for name in required - {"schema_version", "input_size_wh"}):
            raise ValueError("SuperPoint manifest names/provenance must be nonempty strings")
        model = path.parent / manifest["model_file"]
        digest = hashlib.sha256(model.read_bytes()).hexdigest()
        if digest != manifest["sha256"].lower():
            raise ValueError("SuperPoint model SHA256 mismatch")
        self.manifest = manifest
        self.network = create_model(model, manifest)

    def infer(self, image, *, image_scale=1.):
        image = np.asarray(image)
        if image.dtype != np.uint8 or image.ndim not in (2, 3) or min(image.shape[:2]) < 8:
            raise ValueError("SuperPoint requires an 8-bit grayscale or BGR image")
        if not np.isfinite(image_scale) or not .25 <= image_scale <= 1.:
            raise ValueError("SuperPoint image scale must be in [0.25, 1]")
        if image.ndim == 3:
            if image.shape[2] != 3:
                raise ValueError("SuperPoint color input must be BGR with three channels")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        original_size = (image.shape[1], image.shape[0])
        if self.manifest.get("dynamic_input", False):
            # A dynamic Tiny-SuperPoint graph receives the scheduled size
            # directly.  Unlike a fixed graph, this lowers its tensor shape
            # and MAC count rather than merely discarding image information.
            size = tuple(max(8, 8 * max(1, int(round(value * image_scale / 8.))))
                         for value in original_size)
            resized = image if size == original_size else cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        else:
            if image_scale < 1.:
                scaled_size = tuple(max(8, int(round(value * image_scale))) for value in original_size)
                image = cv2.resize(image, scaled_size, interpolation=cv2.INTER_AREA)
            size = tuple(self.manifest["input_size_wh"])
            resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        blob = (resized.astype(np.float32) / 255.)[None, None]
        logits, descriptors = self.network.run(blob)
        if np.shape(logits)[2:] != (size[1] // 8, size[0] // 8):
            raise ValueError("SuperPoint output resolution does not match declared input size")
        # DenseSuperPoint maps the fixed model grid directly back to the
        # original camera pixels.  Downsampling therefore reduces real input
        # work/information while preserving all downstream image coordinates.
        return DenseSuperPoint(logits, descriptors, original_size,
                               self.manifest["descriptor_sampling"])


@dataclass(frozen=True)
class FeatureMatches:
    previous_indices: np.ndarray
    current_indices: np.ndarray
    distances: np.ndarray


def match_features(previous, current, *, ratio=.8, maximum_distance=.7):
    """Mutual nearest neighbours with ratio checks in both directions."""
    if not 0 < ratio < 1 or not np.isfinite(maximum_distance) or not 0 < maximum_distance <= 2:
        raise ValueError("invalid descriptor ratio/distance threshold")
    empty = FeatureMatches(np.zeros(0, dtype=int), np.zeros(0, dtype=int), np.zeros(0))
    if min(len(previous.points_px), len(current.points_px)) < 2:
        return empty
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward = matcher.knnMatch(previous.descriptors, current.descriptors, k=2)
    backward = matcher.knnMatch(current.descriptors, previous.descriptors, k=2)
    reverse = {pair[0].queryIdx: pair[0].trainIdx for pair in backward
               if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance}
    accepted = [pair[0] for pair in forward if len(pair) == 2
                and pair[0].distance <= maximum_distance and pair[0].distance < ratio * pair[1].distance
                and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
    return FeatureMatches(np.array([item.queryIdx for item in accepted], dtype=int),
                          np.array([item.trainIdx for item in accepted], dtype=int),
                          np.array([item.distance for item in accepted], dtype=float))


def feature_quality(features, matches=None):
    width, height = features.image_size_wh
    points = features.points_px if matches is None else features.points_px[matches.current_indices]
    area = abs(cv2.contourArea(cv2.convexHull(points))) if len(points) >= 3 else 0.
    result = {"tracked_points": len(points), "feature_coverage": float(area / (width * height))}
    if matches is not None:
        result["tracked_points"] = len(matches.distances)
        if len(matches.distances):
            result["match_distance"] = float(np.median(matches.distances))
    return result
