import cv2
import numpy as np
import pytest

from calibration.calibrate_intrinsics import solve_views


def test_known_camera_recovery_and_heldout_reprojection():
    rng = np.random.default_rng(20260905)
    matrix = np.array([[900., 0, 820], [0, 910, 616], [0, 0, 1]])
    distortion = np.array([-.12, .03, .001, -.002, .01])
    pattern = np.zeros((54, 3), np.float32)
    pattern[:, :2] = np.mgrid[:9, :6].T.reshape(-1, 2) * .025
    objects, images = [], []
    for _ in range(30):
        rotation = rng.uniform(-.6, .6, 3)
        translation = np.array([rng.uniform(-.25, .1), rng.uniform(-.2, .1), rng.uniform(.5, .9)])
        projected, _ = cv2.projectPoints(pattern, rotation, translation, matrix, distortion)
        objects.append(pattern.copy())
        images.append(projected)
    fitted, fitted_d, validation = solve_views(objects, images, (1640, 1232))
    np.testing.assert_allclose(fitted, matrix, atol=.01)
    np.testing.assert_allclose(fitted_d.reshape(-1), distortion, atol=.001)
    assert max(validation["holdout_rms_px"]) < .001
    assert not set(validation["training_indices"]) & set(validation["holdout_indices"])


def test_too_few_views_are_not_marked_calibrated():
    with pytest.raises(ValueError, match="20"):
        solve_views([], [], (1640, 1232))
