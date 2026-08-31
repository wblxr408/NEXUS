"""CPU/downsample adapter for XFeat scene-control-point anchoring."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Any, Callable

import cv2
import numpy as np

from .factors import ControlPointFactor


@dataclass(frozen=True)
class FeatureSet:
    keypoints_px: np.ndarray
    descriptors: np.ndarray
    scores: np.ndarray
    scale_to_full_resolution: float
    runtime_ms: float


def load_xfeat(top_k: int = 2048, repo_or_dir: str = "verlab/accelerated_features"):
    """Load the official XFeat implementation from torch hub or a local clone."""
    import torch
    source = "local" if not repo_or_dir.startswith("verlab/") and "/" in repo_or_dir else "github"
    model = torch.hub.load(repo_or_dir, "XFeat", pretrained=True, top_k=top_k, source=source)
    return model.eval().cpu()


def extract_xfeat(image: np.ndarray, *, backend: Any, maximum_width: int = 800) -> FeatureSet:
    """Extract sparse features after an explicit high-resolution downsample."""
    height, width = image.shape[:2]
    scale = min(1.0, maximum_width / float(width))
    resized = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1.0 else image
    started = perf_counter()
    result = backend.detectAndCompute(resized)
    elapsed = (perf_counter() - started) * 1000.0
    # Official XFeat returns a dict with keypoints/descriptors/scores.
    if isinstance(result, (list, tuple)):
        result = result[0]
    keypoints = np.asarray(result["keypoints"], dtype=float)
    return FeatureSet(keypoints / scale, np.asarray(result["descriptors"]),
                      np.asarray(result.get("scores", np.ones(len(keypoints)))), 1.0 / scale, elapsed)


def control_point_factors(matches: np.ndarray, features: FeatureSet, control_points_map_m: np.ndarray,
                          camera_id: int, camera_matrix: np.ndarray, sigma_px: float = 1.0) -> list[ControlPointFactor]:
    """Convert ``[feature_index, control_point_index]`` matches to F4."""
    return [ControlPointFactor(camera_id, control_points_map_m[int(control_index)],
                               features.keypoints_px[int(feature_index)], camera_matrix,
                               sigma_px / max(np.sqrt(features.scores[int(feature_index)]), 0.1))
            for feature_index, control_index in np.asarray(matches, dtype=int)]
