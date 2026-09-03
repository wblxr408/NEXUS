"""Known 3D keypoints, gross outliers, and independent SE(3) derivative checks."""

import cv2
import numpy as np
import pytest

from nexus_vision_localization.reference_geometry import perturb_transform, solve_keypoint_pose
from nexus_vision_localization.target_geometry import compose_target_pose


CAMERA = np.array([[650., 0., 320.], [0., 680., 240.], [0., 0., 1.]])
DISTORTION = np.array([.01, -.005, .0001, -.0002, 0.])


def pose(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = cv2.Rodrigues(np.asarray(rotation, dtype=float))[0]
    result[:3, 3] = translation
    return result


def test_known_metric_keypoints_recover_pose_and_reject_gross_outliers():
    rng = np.random.default_rng(2309)
    objects = rng.uniform(-.2, .2, (32, 3))
    expected = pose([.3, -.15, .2], [.15, -.08, 2.])
    pixels = cv2.projectPoints(objects, cv2.Rodrigues(expected[:3, :3])[0], expected[:3, 3], CAMERA, DISTORTION)[0].reshape(-1, 2)
    pixels += rng.normal(0, .1, pixels.shape)
    pixels[-6:] += [90., -70.]
    result = solve_keypoint_pose(objects, pixels, CAMERA, DISTORTION)
    np.testing.assert_allclose(result.transform_camera_reference[:3, 3], expected[:3, 3], atol=.005)
    angle = cv2.Rodrigues(expected[:3, :3].T @ result.transform_camera_reference[:3, :3])[0]
    assert np.linalg.norm(angle) < .005
    assert result.inlier_count == 26 and all(index < 26 for index in result.inlier_indices)
    assert result.reprojection_error_px < .3 and not result.tag_ids
    assert np.linalg.eigvalsh(result.covariance).min() > 0


def test_nonplanar_four_point_geometry_and_metric_scale_are_explicit():
    objects = np.array([[0., 0., 0.], [.2, 0., .1], [0., .3, .05], [-.1, -.2, .3]])
    expected = pose([.2, -.3, .1], [.1, .05, 1.5])
    pixels = cv2.projectPoints(objects, cv2.Rodrigues(expected[:3, :3])[0], expected[:3, 3], CAMERA, DISTORTION)[0].reshape(-1, 2)
    result = solve_keypoint_pose(objects, pixels, CAMERA, DISTORTION)
    np.testing.assert_allclose(result.transform_camera_reference, expected, atol=1e-5)
    scaled = solve_keypoint_pose(objects * 2, pixels, CAMERA, DISTORTION)
    np.testing.assert_allclose(scaled.transform_camera_reference[:3, 3], expected[:3, 3] * 2, atol=1e-5)


def test_degenerate_keypoints_and_insufficient_inlier_ratio_reject():
    with pytest.raises(ValueError, match="degenerate"):
        solve_keypoint_pose(np.column_stack((np.arange(8.), np.zeros((8, 2)))), np.ones((8, 2)), CAMERA, DISTORTION)
    rng = np.random.default_rng(85)
    objects = rng.uniform(-.2, .2, (32, 3))
    pixels = cv2.projectPoints(objects, np.zeros(3), np.array([0., 0., 2.]), CAMERA, DISTORTION)[0].reshape(-1, 2)
    pixels[20:] = rng.uniform([0., 0.], [640., 480.], (12, 2))
    with pytest.raises(ValueError, match="inlier_ratio"):
        solve_keypoint_pose(objects, pixels, CAMERA, DISTORTION, minimum_inlier_ratio=.9)


def test_target_chain_and_correlated_covariance_match_independent_finite_differences():
    transforms = [pose([.3, -.1, .2], [1., -2., .8]), pose([-.2, .4, .1], [.09, -.03, .05]),
                  pose([.1, -.3, .2], [.2, -.1, 3.])]
    rng = np.random.default_rng(18)
    root = rng.normal(0, .001, (18, 18))
    joint = root @ root.T
    marginals = [joint[i:i + 6, i:i + 6] for i in (0, 6, 12)]
    actual, covariance = compose_target_pose(*transforms, *marginals, joint_covariance=joint)
    expected = transforms[0] @ transforms[1] @ transforms[2]
    np.testing.assert_allclose(actual, expected, atol=1e-12)
    jacobian = np.zeros((6, 18))
    epsilon = 1e-4
    for index in range(18):
        component, axis = divmod(index, 6)
        plus, minus = list(transforms), list(transforms)
        if component == 0 and axis >= 3:
            delta = np.eye(3)[axis - 3] * epsilon
            plus[0], minus[0] = transforms[0].copy(), transforms[0].copy()
            plus[0][:3, :3] = cv2.Rodrigues(delta)[0] @ transforms[0][:3, :3]
            minus[0][:3, :3] = cv2.Rodrigues(-delta)[0] @ transforms[0][:3, :3]
        else:
            plus[component] = perturb_transform(transforms[component], axis, epsilon)
            minus[component] = perturb_transform(transforms[component], axis, -epsilon)
        a, b = plus[0] @ plus[1] @ plus[2], minus[0] @ minus[1] @ minus[2]
        jacobian[:3, index] = (a[:3, 3] - b[:3, 3]) / (2 * epsilon)
        jacobian[3:, index] = cv2.Rodrigues(a[:3, :3] @ b[:3, :3].T)[0].reshape(3) / (2 * epsilon)
    np.testing.assert_allclose(covariance, jacobian @ joint @ jacobian.T, atol=1e-10, rtol=1e-5)
    _, independent = compose_target_pose(*transforms, *marginals)
    assert not np.allclose(covariance, independent, atol=1e-8)


def test_platform_and_extrinsic_errors_increase_target_position_uncertainty():
    transforms = [np.eye(4), np.eye(4), pose([0., 0., 0.], [0., 0., 5.])]
    zero = np.zeros((6, 6))
    platform = zero.copy()
    platform[3, 3] = .01 ** 2  # 10 mrad roll creates 5 cm lateral uncertainty at 5 m
    _, result = compose_target_pose(*transforms, platform, zero, zero)
    assert result[1, 1] == pytest.approx(.05 ** 2)
    assert abs(result[1, 3]) > 0
    with pytest.raises(ValueError, match="marginals"):
        compose_target_pose(*transforms, platform, zero, zero, joint_covariance=np.eye(18))
