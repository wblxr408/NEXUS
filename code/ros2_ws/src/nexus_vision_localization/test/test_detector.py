import numpy as np
import pytest

from nexus_vision_localization.detector import (
    marker_object_points,
    reprojection_error,
    validate_camera_parameters,
)
from nexus_vision_localization.vision_node import rotation_matrix_to_quaternion


def test_marker_points_are_metric_and_centered():
    points = marker_object_points(0.2)
    assert points.shape == (4, 3)
    assert np.allclose(points.mean(axis=0), 0.0)
    assert np.isclose(points[0, 0], -0.1)


def test_camera_parameter_validation_rejects_bad_matrix():
    with pytest.raises(ValueError):
        validate_camera_parameters(np.eye(2), np.zeros(5))


def test_reprojection_error_is_zero_for_projected_points():
    camera = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    distortion = np.zeros(5)
    rotation = np.zeros(3)
    translation = np.array([0.0, 0.0, 1.0])
    import cv2
    points, _ = cv2.projectPoints(marker_object_points(0.1), rotation, translation, camera, distortion)
    assert reprojection_error(points, camera, distortion, rotation, translation, 0.1) < 1e-9


def test_rotation_matrix_conversion_produces_identity_quaternion():
    quaternion = rotation_matrix_to_quaternion(np.eye(3))
    assert np.allclose(quaternion, [0.0, 0.0, 0.0, 1.0])
