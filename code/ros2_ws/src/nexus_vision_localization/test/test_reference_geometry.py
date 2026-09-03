import cv2
import numpy as np
import pytest

from nexus_vision_localization.reference_geometry import (
    SurveyedTag, board_to_map_body, solve_tag_board,
)


CAMERA_MATRIX = np.array([[700., 0., 320.], [0., 700., 240.], [0., 0., 1.]])


def scene(count=3):
    tags = []
    for index in range(count):
        pose = np.eye(4)
        pose[:3, 3] = [.4 * (index - 1), .15 * (index % 2), 0.]
        tags.append(SurveyedTag(index + 2, .2, pose))
    camera_map = np.eye(4)
    camera_map[:3, :3] = cv2.Rodrigues(np.array([np.pi - .15, .1, .05]))[0]
    camera_map[:3, 3] = [.1, .1, 2.]
    detections = []
    for tag in tags:
        pixels = cv2.projectPoints(tag.corners(), cv2.Rodrigues(camera_map[:3, :3])[0],
                                   camera_map[:3, 3], CAMERA_MATRIX, np.zeros(5))[0].reshape(4, 2)
        detections.append((tag.marker_id, pixels))
    return tags, camera_map, detections


def test_joint_reference_pnp_recovers_camera_pose_without_target_tags():
    tags, expected, detections = scene()
    result = solve_tag_board(detections, tags, CAMERA_MATRIX, np.zeros(5))
    np.testing.assert_allclose(result.transform_camera_reference, expected, atol=1e-5)
    assert result.inlier_count == 12 and result.inlier_ratio == 1.
    assert result.reprojection_error_px < 1e-5
    assert np.all(np.linalg.eigvalsh(result.covariance) > 0)


def test_bad_reference_corners_are_rejected_by_joint_ransac():
    tags, expected, detections = scene(count=4)
    detections[-1] = (detections[-1][0], detections[-1][1] + [80., -50.])
    result = solve_tag_board(detections, tags, CAMERA_MATRIX, np.zeros(5))
    np.testing.assert_allclose(result.transform_camera_reference, expected, atol=1e-5)
    assert result.inlier_ratio < 1. and result.inlier_count == 12
    assert tags[-1].marker_id not in result.tag_ids


def test_unknown_tags_cannot_become_map_references():
    tags, _, detections = scene()
    with pytest.raises(ValueError, match="no_surveyed"):
        solve_tag_board([(999, detections[0][1])], tags, CAMERA_MATRIX, np.zeros(5))
    with pytest.raises(ValueError, match="duplicate"):
        solve_tag_board(detections + [detections[0]], tags, CAMERA_MATRIX, np.zeros(5))


def test_camera_extrinsic_and_survey_covariance_propagate_to_body():
    tags, camera_map, detections = scene()
    board = solve_tag_board(detections, tags, CAMERA_MATRIX, np.zeros(5))
    body_camera = np.eye(4)
    body_camera[:3, :3] = cv2.Rodrigues(np.array([.2, -.1, .3]))[0]
    body_camera[:3, 3] = [.08, -.03, .04]
    pose, covariance = board_to_map_body(board, body_camera, np.eye(6) * 1e-5, np.eye(6) * 2e-5)
    np.testing.assert_allclose(pose, np.linalg.inv(camera_map) @ np.linalg.inv(body_camera), atol=1e-5)
    assert np.all(np.linalg.eigvalsh(covariance) > 0)
    _, worse = board_to_map_body(board, body_camera, np.eye(6) * 1e-3, np.eye(6) * 2e-3)
    assert np.trace(worse) > np.trace(covariance)
    assert np.linalg.norm(covariance[:3, 3:]) > 0


def test_degenerate_reference_geometry_and_invalid_calibration_rejected():
    tags, _, detections = scene()
    with pytest.raises(ValueError):
        solve_tag_board([(tags[0].marker_id, np.zeros((4, 2)))], tags, CAMERA_MATRIX, np.zeros(5))
    with pytest.raises(ValueError):
        solve_tag_board(detections, tags, CAMERA_MATRIX, np.array([np.nan] * 5))
    with pytest.raises(ValueError, match="SE"):
        SurveyedTag(1, .1, np.diag([1., 1., -1., 1.]))


def test_pixel_noise_assumption_changes_covariance_not_the_geometric_result():
    tags, _, detections = scene()
    precise = solve_tag_board(detections, tags, CAMERA_MATRIX, np.zeros(5), pixel_sigma_px=.5)
    noisy = solve_tag_board(detections, tags, CAMERA_MATRIX, np.zeros(5), pixel_sigma_px=2.)
    np.testing.assert_allclose(noisy.transform_camera_reference, precise.transform_camera_reference, atol=1e-6)
    np.testing.assert_allclose(noisy.covariance, precise.covariance * 16, rtol=1e-5)
