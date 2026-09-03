"""Two-view geometric tests use synthetic rays, not pretrained image inference."""

import cv2
import numpy as np
import pytest

from nexus_vision_localization.superpoint_frontend import FeatureSet
from nexus_vision_localization.visual_motion import MotionConfig, estimate_static_motion


CAMERA = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])


def scene(*, translation=(.3, .01, .02), noise=.15, outliers=0):
    rng = np.random.default_rng(481)
    rotation = cv2.Rodrigues(np.array([.02, -.04, .05]))[0]
    translation = np.asarray(translation)
    points = rng.uniform([-1.5, -1., 3.], [1.5, 1., 7.], (140, 3))
    transformed = (points - translation) @ rotation

    def project(values):
        pixels = values @ CAMERA.T
        return pixels[:, :2] / pixels[:, 2:] + rng.normal(0., noise, (len(values), 2))

    first, second = project(points), project(transformed)
    if outliers:
        second[:outliers] = rng.uniform([50., 50.], [590., 430.], (outliers, 2))
    descriptors = rng.normal(size=(len(first), 256)).astype(np.float32)
    descriptors /= np.linalg.norm(descriptors, axis=1, keepdims=True)
    previous = FeatureSet(first, descriptors, np.ones(len(first)), (640, 480))
    current = FeatureSet(second, descriptors, np.ones(len(first)), (640, 480))
    return previous, current, rotation, translation


def test_static_motion_rejects_wrong_correspondences_and_recovers_direction_not_scale():
    previous, current, rotation, translation = scene(outliers=20)
    cv2.setRNGSeed(7)
    result = estimate_static_motion(previous, current, CAMERA, np.zeros(5))
    error = cv2.Rodrigues(rotation.T @ result.rotation_camera_i_j)[0]
    assert np.linalg.norm(error) < .02
    assert np.dot(translation / np.linalg.norm(translation), result.direction_camera_i_j) > .99
    assert np.linalg.norm(result.direction_camera_i_j) == pytest.approx(1.)
    assert np.count_nonzero(result.previous_indices < 20) <= 2
    assert len(result.previous_indices) >= 100
    assert np.linalg.eigvalsh(result.covariance).min() > 0.
    assert result.quality["inlier_ratio"] > .7
    assert result.median_parallax_deg > .2


def test_covariance_respects_declared_pixel_noise_and_has_position_rotation_cross_terms():
    previous, current, _, _ = scene(noise=0.)
    cv2.setRNGSeed(1)
    precise = estimate_static_motion(previous, current, CAMERA, np.zeros(5), config=MotionConfig(pixel_sigma_px=.5))
    cv2.setRNGSeed(1)
    noisy = estimate_static_motion(previous, current, CAMERA, np.zeros(5), config=MotionConfig(pixel_sigma_px=1.))
    radial = np.zeros((6, 6))
    radial[:3, :3] = np.outer(precise.direction_camera_i_j, precise.direction_camera_i_j)
    np.testing.assert_allclose(noisy.covariance - radial, 4 * (precise.covariance - radial), atol=1e-9, rtol=1e-5)
    assert np.max(np.abs(precise.covariance[:3, 3:])) > 1e-8


def test_pure_rotation_and_insufficient_features_never_produce_metric_motion():
    previous, current, _, _ = scene(translation=(0., 0., 0.), noise=0.)
    with pytest.raises(ValueError, match="parallax|geometric"):
        estimate_static_motion(previous, current, CAMERA, np.zeros(5))
    with pytest.raises(ValueError, match="insufficient mutual"):
        estimate_static_motion(previous.select(slice(0, 5)), current, CAMERA, np.zeros(5))


def test_rotation_conflict_with_independent_imu_prediction_is_rejected():
    previous, current, _, _ = scene(noise=0.)
    with pytest.raises(ValueError, match="IMU prediction"):
        estimate_static_motion(previous, current, CAMERA, np.zeros(5),
                               predicted_rotation_i_j=cv2.Rodrigues(np.array([0., 0., 1.]))[0])


def test_camera_body_conversion_retains_nonzero_lever_and_sample_times():
    previous, current, rotation, translation = scene(noise=0.)
    result = estimate_static_motion(previous, current, CAMERA, np.zeros(5))
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = cv2.Rodrigues(np.array([.4, -.2, .1]))[0]
    extrinsic[:3, 3] = [.2, -.1, .05]
    observation = result.body_observation(1_000_000_000, 1_100_000_000, extrinsic)
    assert observation["metric_translation"] is False
    assert observation["sensor_body_m"] == [.2, -.1, .05]
    assert observation["previous_stamp_ns"] == 1_000_000_000
    assert observation["sample_timestamp_ns"] == 1_100_000_000
    camera_j = np.eye(4)
    camera_j[:3, :3], camera_j[:3, 3] = rotation, translation
    body_i, body_j = np.linalg.inv(extrinsic), camera_j @ np.linalg.inv(extrinsic)
    relative_body = np.linalg.inv(body_i) @ body_j
    expected_direction = relative_body[:3, 3] + relative_body[:3, :3] @ extrinsic[:3, 3] - extrinsic[:3, 3]
    expected_direction /= np.linalg.norm(expected_direction)
    np.testing.assert_allclose(observation["translation_i_j"], expected_direction, atol=1e-5)
    np.testing.assert_allclose(observation["rotation_i_j"], relative_body[:3, :3], atol=1e-5)
    assert np.linalg.eigvalsh(np.reshape(observation["covariance"], (6, 6))).min() > 0
