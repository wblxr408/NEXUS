"""Small, bounded optical-flow tracker for the 10 Hz online path."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class TrackedTarget:
    target_id: int
    center_px: np.ndarray
    bbox_xywh_px: np.ndarray
    confidence: float
    sigma_px: float
    valid: bool = True
    reason: str = ""


def _gray(image: np.ndarray) -> np.ndarray:
    array = np.asarray(image)
    if array.ndim == 2:
        return array.astype(np.uint8, copy=False)
    if array.ndim == 3 and array.shape[2] >= 3:
        return cv2.cvtColor(array[..., :3], cv2.COLOR_BGR2GRAY)
    raise ValueError("image must be grayscale or BGR")


class PatchTracker:
    """Track one point per target using pyramidal LK and subpixel refinement.

    The tracker intentionally tracks the detector-provided center rather than
    inventing object depth or pose.  A detector can replace a track at any
    time, and all outputs carry a measured pixel uncertainty for F1.
    """

    def __init__(self, *, win_size: int = 21, max_level: int = 3,
                 max_error_px: float = 12.0, sigma_floor_px: float = 0.8):
        if win_size < 5 or win_size % 2 == 0:
            raise ValueError("win_size must be an odd value >= 5")
        self.win_size = int(win_size)
        self.max_level = int(max_level)
        self.max_error_px = float(max_error_px)
        self.sigma_floor_px = float(sigma_floor_px)
        self._previous_gray: np.ndarray | None = None
        self._tracks: dict[int, TrackedTarget] = {}

    @staticmethod
    def _refine(gray: np.ndarray, point: np.ndarray) -> np.ndarray:
        points = np.asarray(point, dtype=np.float32).reshape(1, 1, 2)
        refined = cv2.cornerSubPix(
            gray, points, (5, 5), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
        )
        return refined.reshape(2).astype(float)

    def initialize(self, image: np.ndarray, detections: Iterable[TrackedTarget]) -> list[TrackedTarget]:
        gray = _gray(image)
        self._previous_gray = gray
        self._tracks = {}
        for detection in detections:
            center = self._refine(gray, detection.center_px)
            bbox = np.asarray(detection.bbox_xywh_px, dtype=float).reshape(4)
            self._tracks[int(detection.target_id)] = TrackedTarget(
                int(detection.target_id), center, bbox, float(detection.confidence),
                max(float(detection.sigma_px), self.sigma_floor_px), True, "",
            )
        return list(self._tracks.values())

    def update(self, image: np.ndarray) -> list[TrackedTarget]:
        gray = _gray(image)
        if self._previous_gray is None or not self._tracks:
            self._previous_gray = gray
            return []
        ids = list(self._tracks)
        old_points = np.asarray([self._tracks[key].center_px for key in ids], dtype=np.float32)
        new_points, status, errors = cv2.calcOpticalFlowPyrLK(
            self._previous_gray, gray, old_points.reshape(-1, 1, 2), None,
            winSize=(self.win_size, self.win_size), maxLevel=self.max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
        )
        output: list[TrackedTarget] = []
        for index, target_id in enumerate(ids):
            track = self._tracks[target_id]
            ok = bool(status[index, 0]) and np.all(np.isfinite(new_points[index, 0]))
            error = float(errors[index, 0]) if errors is not None else self.max_error_px
            ok = ok and error <= self.max_error_px
            if ok:
                center = self._refine(gray, new_points[index, 0])
                delta = center - track.center_px
                bbox = track.bbox_xywh_px.copy()
                bbox[:2] += delta
                confidence = float(np.clip(track.confidence * 0.98 + 0.02 * np.exp(-error / 8.0), 0.0, 1.0))
                output.append(TrackedTarget(target_id, center, bbox, confidence,
                                            max(self.sigma_floor_px, error), True, ""))
            else:
                output.append(TrackedTarget(target_id, track.center_px.copy(), track.bbox_xywh_px.copy(),
                                            0.0, float("nan"), False, "optical_flow_lost"))
        self._tracks = {item.target_id: item for item in output if item.valid}
        self._previous_gray = gray
        return output
