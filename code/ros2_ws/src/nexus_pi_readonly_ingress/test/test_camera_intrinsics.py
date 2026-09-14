import json

import numpy as np
import pytest

from nexus_pi_readonly_ingress.camera_intrinsics import CameraIntrinsics


def test_rectification_preserves_zero_distortion_and_checks_camera_mode(tmp_path):
    data = dict(schema_version=1, calibration_status="measured", distortion_model="plumb_bob",
                width=64, height=48, rotation_deg=180, camera_matrix=[[50, 0, 32], [0, 50, 24], [0, 0, 1]],
                distortion_coefficients=[0] * 5)
    path = tmp_path / "synthetic_intrinsics.json"
    path.write_text(json.dumps(data))
    calibration = CameraIntrinsics(path)
    image = np.random.default_rng(1).integers(0, 256, (48, 64, 3), dtype=np.uint8)
    np.testing.assert_array_equal(calibration.rectify(image), image)
    calibration.check_mode(64, 48, 180)
    with pytest.raises(ValueError, match="mode"):
        calibration.check_mode(64, 48, 0)
    data["calibration_status"] = "assumed"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="measured"):
        CameraIntrinsics(path)


def test_rectified_valid_roi_has_cropped_size_and_shifted_principal_point(tmp_path):
    data = dict(schema_version=1, calibration_status="measured", distortion_model="plumb_bob",
                width=64, height=48, rotation_deg=180,
                camera_matrix=[[50, 0, 32], [0, 50, 24], [0, 0, 1]],
                distortion_coefficients=[0] * 5,
                valid_roi_normalized=[0.25, 0.25, 0.75, 0.75])
    path = tmp_path / "roi_intrinsics.json"
    path.write_text(json.dumps(data))
    calibration = CameraIntrinsics(path)
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    assert calibration.roi_xyxy == (16, 12, 48, 36)
    assert calibration.crop_rectified(calibration.rectify(image)).shape == (24, 32, 3)
    np.testing.assert_allclose(calibration.rectified_matrix,
                               [[50, 0, 16], [0, 50, 12], [0, 0, 1]])
