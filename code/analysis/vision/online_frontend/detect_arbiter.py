"""Detector/tracker arbitration for the bounded online visual path."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Mapping, Any

import numpy as np

from .patch_tracker import PatchTracker, TrackedTarget


@dataclass(frozen=True)
class Detection:
    target_id: int
    bbox_xywh_px: np.ndarray
    confidence: float
    sigma_px: float = 3.0

    @property
    def center_px(self) -> np.ndarray:
        x, y, width, height = np.asarray(self.bbox_xywh_px, dtype=float)
        return np.array([x + width / 2.0, y + height / 2.0])


def _coerce_detection(value: Detection | Mapping[str, Any]) -> Detection:
    if isinstance(value, Detection):
        return value
    return Detection(
        target_id=int(value["target_id"]),
        bbox_xywh_px=np.asarray(value["bbox_xywh_px"], dtype=float),
        confidence=float(value.get("confidence", 0.0)),
        sigma_px=float(value.get("sigma_px", 3.0)),
    )


class DetectionArbiter:
    """Run a low-rate detector and use LK tracks between detector frames.

    ``detector`` may return mappings or :class:`Detection` values.  Detector
    observations always reset the corresponding track, preventing drift from
    accumulating across the whole flight.  A detector failure leaves the
    current tracks intact but marks the cycle as degraded.
    """

    def __init__(self, detector: Callable[[np.ndarray], Iterable[Detection | Mapping[str, Any]]],
                 *, detector_period_s: float = 0.5, min_confidence: float = 0.35,
                 tracker: PatchTracker | None = None):
        if detector_period_s <= 0:
            raise ValueError("detector_period_s must be positive")
        self.detector = detector
        self.detector_period_s = float(detector_period_s)
        self.min_confidence = float(min_confidence)
        self.tracker = tracker or PatchTracker()
        self._last_detection_time_s: float | None = None
        self.last_mode = "uninitialized"
        self.last_reason = ""

    def process(self, image: np.ndarray, timestamp_s: float) -> list[TrackedTarget]:
        now = float(timestamp_s)
        run_detector = self._last_detection_time_s is None or now - self._last_detection_time_s >= self.detector_period_s
        if run_detector:
            try:
                detections = [_coerce_detection(item) for item in self.detector(image)]
                detections = [item for item in detections if item.confidence >= self.min_confidence]
            except Exception as exc:  # detector errors degrade to tracking, never a fake pose
                self.last_mode, self.last_reason = "tracker", f"detector_error:{type(exc).__name__}"
                return self.tracker.update(image)
            tracks = [TrackedTarget(item.target_id, item.center_px, item.bbox_xywh_px,
                                    item.confidence, item.sigma_px) for item in detections]
            self._last_detection_time_s = now
            self.last_mode, self.last_reason = "detector", "" if tracks else "no_detection"
            return self.tracker.initialize(image, tracks)
        self.last_mode, self.last_reason = "tracker", ""
        return self.tracker.update(image)
