#!/usr/bin/env python3
"""IMX219 RGB frame source for real-hardware vision pipelines.

The returned pixels are RGB888 and are already rotated into the configured
airframe orientation.  This module neither opens the flight-controller UART
nor emits any MAVLink data.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    """One camera frame suitable for a GDR-Net image preprocessor."""

    rgb: np.ndarray
    captured_monotonic_ns: int
    sensor_timestamp_ns: int | None
    exposure_time_us: int | None
    analogue_gain: float | None


class Imx219FrameSource:
    """Own an IMX219 capture session and yield RGB888 frames."""

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fps: float = 15.0,
        rotation_deg: int = 180,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("width and height must be positive")
        if fps <= 0:
            raise ValueError("fps must be positive")
        if rotation_deg not in (0, 180):
            raise ValueError("rotation_deg must be 0 or 180")
        self.width = width
        self.height = height
        self.fps = fps
        self.rotation_deg = rotation_deg
        self._camera: Any | None = None

    def start(self) -> None:
        """Configure and start the CSI camera exactly once."""
        if self._camera is not None:
            raise RuntimeError("camera is already started")

        from libcamera import Transform
        from picamera2 import Picamera2

        period_us = round(1_000_000 / self.fps)
        transform = Transform(
            hflip=self.rotation_deg == 180,
            vflip=self.rotation_deg == 180,
        )
        camera = Picamera2()
        configuration = camera.create_video_configuration(
            # libcamera BGR888 produces byte-ordered RGB arrays on the Pi.
            main={"size": (self.width, self.height), "format": "BGR888"},
            transform=transform,
            controls={"FrameDurationLimits": (period_us, period_us)},
            buffer_count=4,
        )
        camera.configure(configuration)
        camera.start()
        self._camera = camera

    def capture(self) -> CameraFrame:
        """Capture one independent RGB frame plus sensor exposure metadata."""
        if self._camera is None:
            raise RuntimeError("call start() before capture()")

        request = self._camera.capture_request()
        try:
            captured_monotonic_ns = __import__("time").monotonic_ns()
            metadata = request.get_metadata()
            # make_array returns a frame backed by the completed request; copy it
            # before releasing that request back to libcamera.
            rgb = np.ascontiguousarray(request.make_array("main")).copy()
        finally:
            request.release()

        return CameraFrame(
            rgb=rgb,
            captured_monotonic_ns=captured_monotonic_ns,
            sensor_timestamp_ns=_optional_int(metadata.get("SensorTimestamp")),
            exposure_time_us=_optional_int(metadata.get("ExposureTime")),
            analogue_gain=_optional_float(metadata.get("AnalogueGain")),
        )

    def close(self) -> None:
        """Release the camera device; safe to call after a failed start."""
        if self._camera is None:
            return
        try:
            self._camera.stop()
        finally:
            self._camera.close()
            self._camera = None

    def __enter__(self) -> Imx219FrameSource:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _optional_int(value: Any) -> int | None:
    return None if value is None else int(value)


def _optional_float(value: Any) -> float | None:
    return None if value is None else float(value)
