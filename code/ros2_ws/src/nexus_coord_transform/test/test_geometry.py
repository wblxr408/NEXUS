import numpy as np
import pytest

from nexus_coord_transform.geometry import (
    BODY_TO_OPTICAL,
    CARLA_ACTOR_TO_OPTICAL,
    carla_camera_pose,
    carla_point_to_map,
    carla_rotation_matrix,
    carla_rotation_to_map,
    body_to_camera_rotation,
    direct_target_measurement,
    centimeters_to_meters,
    matrix_to_quaternion,
    meters_to_centimeters,
    ned_to_enu,
    quaternion_to_matrix,
    umeyama_alignment,
    platform_relative_measurement,
    rotate_position_covariance,
)


def test_carla_axis_swap_and_optical_pose_preserve_proper_rotation():
    assert np.allclose(carla_point_to_map([1.0, 2.0, 3.0]), [1.0, -2.0, 3.0])
    actor = carla_rotation_matrix(0.0, 0.0, 0.0)
    assert np.isclose(np.linalg.det(carla_rotation_to_map(actor)), 1.0)
    camera_rotation, camera_translation = carla_camera_pose([1.0, 2.0, 3.0], 0.0, 0.0, 0.0)
    assert np.isclose(np.linalg.det(camera_rotation), 1.0)
    assert np.allclose(camera_translation, [1.0, -2.0, 3.0])
    assert np.allclose(CARLA_ACTOR_TO_OPTICAL[:, 0], [0.0, 1.0, 0.0])


def test_body_optical_axis_swap_has_expected_handedness():
    assert np.isclose(np.linalg.det(BODY_TO_OPTICAL), 1.0)
    assert np.isclose(np.linalg.det(body_to_camera_rotation(-90.0)), 1.0)


def test_unit_and_axis_conversion_are_explicit():
    assert np.allclose(centimeters_to_meters([100, -50, 25]), [1, -0.5, 0.25])
    assert np.allclose(meters_to_centimeters([1, -0.5, 0.25]), [100, -50, 25])
    assert np.allclose(ned_to_enu([1, 2, 3]), [2, 1, -3])


def test_quaternion_round_trip():
    quaternion = np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])
    result = matrix_to_quaternion(quaternion_to_matrix(quaternion))
    assert np.isclose(abs(np.dot(result, quaternion)), 1.0, atol=1e-6)


def test_umeyama_recovers_rigid_transform():
    source = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    translation = np.array([2., -1., 0.5])
    target = (rotation @ source.T).T + translation
    recovered_rotation, recovered_translation = umeyama_alignment(source, target)
    assert np.allclose(recovered_rotation, rotation)
    assert np.allclose(recovered_translation, translation)


def test_umeyama_rejects_too_few_points():
    with pytest.raises(ValueError):
        umeyama_alignment(np.zeros((2, 3)), np.zeros((2, 3)))


def test_direct_and_platform_relative_paths_are_distinct_entries():
    identity = np.eye(3)
    assert np.allclose(direct_target_measurement([1, 2, 3], identity, [4, 5, 6]), [5, 7, 9])
    assert np.allclose(platform_relative_measurement([4, 5, 6], identity, [1, 2, 3]), [5, 7, 9])


def test_position_covariance_rotates_with_frame():
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    covariance = np.diag([1., 4., 9.])
    assert np.allclose(
        rotate_position_covariance(covariance, rotation), np.diag([4., 1., 9.]))
