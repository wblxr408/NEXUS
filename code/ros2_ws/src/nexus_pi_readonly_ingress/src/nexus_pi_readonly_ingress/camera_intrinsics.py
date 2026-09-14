"""Load measured intrinsics for exactly the calibrated camera mode."""

import json
from pathlib import Path

import cv2
import numpy as np


class CameraIntrinsics:
    def __init__(self, path):
        self.document = json.loads(Path(path).read_text())
        data = self.document
        if data.get("schema_version") != 1 or data.get("calibration_status") != "measured":
            raise ValueError("measured camera intrinsics are required")
        if data.get("distortion_model") != "plumb_bob":
            raise ValueError("unsupported camera distortion model")
        self.matrix = np.asarray(data["camera_matrix"], dtype=float)
        self.distortion = np.asarray(data["distortion_coefficients"], dtype=float)
        if (self.matrix.shape != (3, 3) or self.distortion.shape != (5,)
                or not np.isfinite(self.matrix).all() or not np.isfinite(self.distortion).all()
                or min(self.matrix[0, 0], self.matrix[1, 1]) <= 0
                or not np.allclose(self.matrix[2], [0, 0, 1])):
            raise ValueError("invalid camera matrix or distortion coefficients")
        self.size = (int(data["width"]), int(data["height"]))
        if min(self.size) <= 0:
            raise ValueError("invalid calibrated image size")
        roi = np.asarray(data.get("valid_roi_normalized", [0.0, 0.0, 1.0, 1.0]), dtype=float)
        if (roi.shape != (4,) or not np.isfinite(roi).all() or np.any(roi < 0.0)
                or np.any(roi > 1.0) or roi[0] >= roi[2] or roi[1] >= roi[3]):
            raise ValueError("invalid normalized valid ROI")
        x0, y0 = np.floor(roi[:2] * self.size).astype(int)
        x1, y1 = np.ceil(roi[2:] * self.size).astype(int)
        if x1 <= x0 or y1 <= y0:
            raise ValueError("valid ROI has no pixels")
        self.valid_roi_normalized = roi
        self.roi_xyxy = (int(x0), int(y0), int(x1), int(y1))
        self.rectified_matrix = self.matrix.copy()
        self.rectified_matrix[0, 2] -= x0
        self.rectified_matrix[1, 2] -= y0
        self.maps = cv2.initUndistortRectifyMap(
            self.matrix, self.distortion, np.eye(3), self.matrix, self.size, cv2.CV_32FC1)

    def check_mode(self, width, height, rotation):
        if (width, height) != self.size or rotation != self.document["rotation_deg"]:
            raise ValueError("camera mode does not match measured intrinsics")

    def rectify(self, rgb):
        return cv2.remap(rgb, *self.maps, interpolation=cv2.INTER_LINEAR)

    def crop_rectified(self, rgb):
        x0, y0, x1, y1 = self.roi_xyxy
        return rgb[y0:y1, x0:x1]
