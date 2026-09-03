import cv2
import numpy as np
import pytest
from scipy.optimize import least_squares

from nexus_vision_localization.pose_timeline import PoseTimeline, TimedPose
from nexus_vision_localization.target_multiview import BearingObservation, solve_multiview


CAMERA = np.array([[700., 0., 320.], [0., 710., 240.], [0., 0., 1.]])
DISTORTION = np.array([.01, -.005, .0002, -.0001, 0.])


def synthetic_views(*, moving=False, linear_camera=False, count=10, platform_sigma=0., noise=0.):
    rng = np.random.default_rng(324)
    end_position, velocity = np.array([.1, -.2, 4.]), np.array([.2, -.15, .05]) if moving else np.zeros(3)
    output = []
    for index, t in enumerate(np.linspace(-1., 0., count)):
        camera = np.eye(4)
        camera[:3, 3] = [t * .7, 0., 0.] if linear_camera else [.7 * t, .5 * t ** 2, .2 * np.sin(4 * t)]
        point = end_position + velocity * t
        pixel = cv2.projectPoints(point.reshape(1, 3), np.zeros(3), -camera[:3, 3], CAMERA, DISTORTION)[0].reshape(2)
        pixel += rng.normal(0., noise, 2)
        body = TimedPose(2_000_000_000 + int(t * 1e9), camera, np.eye(6) * platform_sigma ** 2)
        output.append(BearingObservation(body, pixel, CAMERA, DISTORTION, pixel_sigma_px=max(noise, .1)))
    return output, end_position, velocity


def test_static_reference_point_is_triangulated_with_outlier_rejection():
    observations, expected, _ = synthetic_views(noise=.1)
    item = observations[3]
    observations[3] = BearingObservation(item.platform, item.pixel + [90., -60.], CAMERA, DISTORTION)
    result = solve_multiview(observations, np.eye(4), np.zeros((6, 6)))
    # Compare noisy recovery against an independent numerical-Jacobian fit
    # with the known clean subset. A single random draw need not fall inside
    # a chosen confidence interval or a fixed millimetre threshold.
    good = [view for index, view in enumerate(observations) if index != 3]

    def residual(point):
        return np.concatenate([(cv2.projectPoints(point.reshape(1, 3), np.zeros(3), -view.platform.transform[:3, 3],
                                                  CAMERA, DISTORTION)[0].reshape(2) - view.pixel) / .1 for view in good])

    independent = least_squares(residual, np.array([0., 0., 3.]), loss="huber", f_scale=1.5)
    np.testing.assert_allclose(result.position_m, independent.x, atol=1e-5)
    exact, _, _ = synthetic_views()
    np.testing.assert_allclose(solve_multiview(exact, np.eye(4), np.zeros((6, 6))).position_m, expected, atol=1e-7)
    assert item.platform.stamp_ns not in result.inlier_stamps and len(result.inlier_stamps) == 9
    assert result.velocity_mps is None and result.covariance.shape == (3, 3)
    assert np.linalg.eigvalsh(result.covariance).min() > 0
    assert result.reprojection_rmse_px < .3


def test_current_outlier_cannot_be_reported_as_fresh_target_position():
    observations, _, _ = synthetic_views()
    item = observations[-1]
    observations[-1] = BearingObservation(item.platform, item.pixel + [80., 90.], CAMERA, DISTORTION)
    with pytest.raises(ValueError, match="current_target_observation_is_outlier"):
        solve_multiview(observations, np.eye(4), np.zeros((6, 6)))


def test_constant_velocity_target_requires_non_affine_camera_motion():
    observations, position, velocity = synthetic_views(moving=True, count=12, noise=.03)
    result = solve_multiview(observations, np.eye(4), np.zeros((6, 6)), motion_model="constant_velocity")
    np.testing.assert_allclose(result.position_m, position, atol=.008)
    np.testing.assert_allclose(result.velocity_mps, velocity, atol=.01)
    assert result.covariance.shape == (6, 6) and np.linalg.eigvalsh(result.covariance).min() > 0
    ambiguous, _, _ = synthetic_views(moving=True, linear_camera=True)
    with pytest.raises(ValueError, match="unobservable"):
        solve_multiview(ambiguous, np.eye(4), np.zeros((6, 6)), motion_model="constant_velocity")


def test_no_baseline_and_unordered_frames_cannot_supply_depth():
    observations, _, _ = synthetic_views()
    repeated = [BearingObservation(
        TimedPose(item.platform.stamp_ns, observations[0].platform.transform, np.zeros((6, 6))),
        observations[0].pixel, CAMERA, DISTORTION) for item in observations]
    with pytest.raises(ValueError, match="unobservable"):
        solve_multiview(repeated, np.eye(4), np.zeros((6, 6)))
    with pytest.raises(ValueError, match="increasing"):
        solve_multiview(observations[::-1], np.eye(4), np.zeros((6, 6)))


def test_shared_extrinsic_and_platform_uncertainty_are_not_averaged_away():
    observations, _, _ = synthetic_views()
    baseline = solve_multiview(observations, np.eye(4), np.zeros((6, 6)))
    extrinsic_covariance = np.zeros((6, 6))
    extrinsic_covariance[:3, :3] = np.eye(3) * .02 ** 2
    uncertain = solve_multiview(observations, np.eye(4), extrinsic_covariance)
    np.testing.assert_allclose(uncertain.covariance - baseline.covariance, np.eye(3) * .02 ** 2, atol=1e-8)
    noisy_platform, _, _ = synthetic_views(platform_sigma=.01)
    result = solve_multiview(noisy_platform, np.eye(4), extrinsic_covariance)
    assert np.trace(result.covariance) > np.trace(uncertain.covariance)


def test_rotating_camera_and_nonzero_mount_extrinsic_recover_same_map_point():
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = cv2.Rodrigues(np.array([.2, -.3, .1]))[0]
    extrinsic[:3, 3] = [.12, -.06, .08]
    target = np.array([.1, -.2, 4.])
    observations = []
    for i in range(8):
        camera = np.eye(4)
        camera[:3, :3] = cv2.Rodrigues(np.array([.015 * i, -.01 * i, .008 * i]))[0]
        camera[:3, 3] = [.1 * i, .02 * i ** 2, .04 * i]
        rotation = camera[:3, :3].T
        pixel = cv2.projectPoints(target.reshape(1, 3), cv2.Rodrigues(rotation)[0], -rotation @ camera[:3, 3],
                                  CAMERA, DISTORTION)[0].reshape(2)
        body = TimedPose(1_000_000_000 + i * 100_000_000, camera @ np.linalg.inv(extrinsic), np.eye(6) * 1e-5)
        observations.append(BearingObservation(body, pixel, CAMERA, DISTORTION))
    result = solve_multiview(observations, extrinsic, np.eye(6) * 1e-6)
    np.testing.assert_allclose(result.position_m, target, atol=1e-7)
    assert np.linalg.eigvalsh(result.covariance).min() > 0


def test_pose_timeline_interpolates_original_time_and_preserves_common_error():
    start = 1_700_000_000_000_000_000
    timeline = PoseTimeline()
    first, second = np.eye(4), np.eye(4)
    second[:3, 3] = [.1, .02, -.01]
    second[:3, :3] = cv2.Rodrigues(np.array([0., 0., .2]))[0]
    covariance = np.eye(6) * .01
    timeline.add(TimedPose(start + 100_000_000, second, covariance))
    timeline.add(TimedPose(start, first, covariance))
    result = timeline.at(start + 50_000_000)
    assert result.stamp_ns == start + 50_000_000 and result.interpolated
    np.testing.assert_allclose(result.transform[:3, 3], second[:3, 3] / 2)
    np.testing.assert_allclose(cv2.Rodrigues(result.transform[:3, :3])[0].reshape(3), [0., 0., .1], atol=1e-8)
    assert np.min(np.diag(result.covariance)) >= .01
    assert timeline.at(start).interpolated is False
    with pytest.raises(ValueError, match="not_bracketed"):
        timeline.at(start + 110_000_000)
    timeline.add(TimedPose(start + 1_000_000_000, second, covariance))
    with pytest.raises(ValueError, match="gap"):
        timeline.at(start + 500_000_000)
