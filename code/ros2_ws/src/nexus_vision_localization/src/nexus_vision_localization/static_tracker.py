"""Bounded reference-frame management for calibrated static visual motion."""

from dataclasses import dataclass

import cv2
import numpy as np

from .visual_motion import MotionConfig, VisualMotion, estimate_static_motion


@dataclass(frozen=True)
class StaticTrackingUpdate:
    stamp_ns: int
    previous_stamp_ns: int | None
    state: str
    reason: str
    motion: VisualMotion | None = None


class StaticMotionTracker:
    def __init__(self, config=None, maximum_reference_age_s=.5):
        self.config = config or MotionConfig()
        if not np.isfinite(maximum_reference_age_s) or maximum_reference_age_s <= 0:
            raise ValueError("maximum reference age must be positive")
        self.maximum_reference_age_ns = int(maximum_reference_age_s * 1e9)
        self.reference = None
        self.reference_stamp_ns = None
        self.last_stamp_ns = 0
        self.geometry = None

    def reset(self):
        self.reference, self.reference_stamp_ns, self.geometry = None, None, None

    def process(self, features, stamp_ns, camera_matrix, distortion, predicted_rotation_i_j=None):
        if not isinstance(stamp_ns, int) or stamp_ns <= self.last_stamp_ns:
            raise ValueError("static frame timestamps must be strictly increasing integers")
        self.last_stamp_ns = stamp_ns
        signature = (features.image_size_wh, np.asarray(camera_matrix).tobytes(), np.asarray(distortion).tobytes())
        changed = self.geometry is not None and signature != self.geometry
        expired = self.reference_stamp_ns is not None and stamp_ns - self.reference_stamp_ns > self.maximum_reference_age_ns
        if changed or expired:
            self.reset()
        if len(features.points_px) < self.config.minimum_matches:
            return StaticTrackingUpdate(stamp_ns, self.reference_stamp_ns,
                                        "degraded" if self.reference is not None else "lost", "insufficient_static_features")
        if self.reference is None:
            self.reference, self.reference_stamp_ns, self.geometry = features, stamp_ns, signature
            reason = "camera_geometry_changed" if changed else "reference_expired" if expired else "reference_initialized"
            return StaticTrackingUpdate(stamp_ns, None, "initializing", reason)
        previous = self.reference_stamp_ns
        try:
            motion = estimate_static_motion(self.reference, features, camera_matrix, distortion,
                                            config=self.config, predicted_rotation_i_j=predicted_rotation_i_j)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            # Preserve a still-young reference, allowing weak inter-frame
            # parallax to accumulate instead of restarting at every image.
            return StaticTrackingUpdate(stamp_ns, previous, "degraded", str(error))
        self.reference, self.reference_stamp_ns, self.geometry = features, stamp_ns, signature
        return StaticTrackingUpdate(stamp_ns, previous, "tracking", "", motion)
