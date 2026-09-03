"""Joint tag-board PnP and surveyed-map camera/body uncertainty propagation."""

from dataclasses import dataclass, replace

import cv2
import numpy as np

from .detector import marker_object_points, validate_camera_parameters


def rigid_transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix[3], [0., 0., 0., 1.])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6)):
        raise ValueError("reference transform must be finite SE(3)")
    return matrix.copy()


def pose_covariance(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (6, 6) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix, matrix.T, atol=1e-9) or np.min(np.linalg.eigvalsh(matrix)) < -1e-12):
        raise ValueError("pose covariance must be finite symmetric positive semidefinite 6 x 6")
    return matrix.copy()


def perturb_transform(transform, axis, delta):
    result = transform.copy()
    if axis < 3:
        result[axis, 3] += delta
    else:
        rotation = np.zeros(3)
        rotation[axis - 3] = delta
        result[:3, :3] = result[:3, :3] @ cv2.Rodrigues(rotation)[0]
    return result


def _skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


@dataclass(frozen=True)
class SurveyedTag:
    marker_id: int
    size_m: float
    transform_reference_tag: np.ndarray

    def __post_init__(self):
        marker_object_points(self.size_m)
        if int(self.marker_id) != self.marker_id or self.marker_id < 0:
            raise ValueError("tag ID must be a nonnegative integer")
        object.__setattr__(self, "transform_reference_tag", rigid_transform(self.transform_reference_tag))

    def corners(self):
        transform = self.transform_reference_tag
        return marker_object_points(self.size_m) @ transform[:3, :3].T + transform[:3, 3]


@dataclass(frozen=True)
class BoardPose:
    transform_camera_reference: np.ndarray
    covariance: np.ndarray  # translation in camera, right rotation in reference
    reprojection_error_px: float
    inlier_ratio: float
    inlier_count: int
    tag_ids: tuple
    inlier_indices: tuple = ()


def solve_tag_board(detections, tags, camera_matrix, distortion, *, pixel_sigma_px=1.,
                    ransac_threshold_px=3., minimum_inlier_ratio=.6, maximum_reprojection_px=3.):
    """Joint PnP for a map board or a tagged target; unknown tags are ignored.

    Corner order is the existing marker_object_points/AprilTag detector order.
    No detected pose or target ground truth is consumed, only image corners.
    """
    matrix, distortion = validate_camera_parameters(camera_matrix, distortion)
    if (not np.all(np.isfinite(distortion)) or not np.all(np.isfinite(
            [pixel_sigma_px, ransac_threshold_px, minimum_inlier_ratio, maximum_reprojection_px]))
            or min(pixel_sigma_px, ransac_threshold_px, maximum_reprojection_px) <= 0
            or not 0 < minimum_inlier_ratio <= 1):
        raise ValueError("invalid camera distortion or board quality thresholds")
    registry = {tag.marker_id: tag for tag in tags}
    if len(registry) != len(tags):
        raise ValueError("duplicate configured tag ID")
    object_points, image_points, ids = [], [], []
    for marker_id, corners in detections:
        if marker_id not in registry:
            continue
        if marker_id in ids:
            raise ValueError("duplicate detected reference tag")
        points = np.asarray(corners, dtype=float)
        if points.shape != (4, 2) or not np.all(np.isfinite(points)):
            raise ValueError("each tag requires four finite image corners")
        object_points.extend(registry[marker_id].corners())
        image_points.extend(points)
        ids.append(marker_id)
    if not ids:
        raise ValueError("no_surveyed_reference_visible")
    objects, pixels = np.array(object_points, dtype=float), np.array(image_points, dtype=float)
    result = solve_keypoint_pose(objects, pixels, matrix, distortion, pixel_sigma_px=pixel_sigma_px,
                                 ransac_threshold_px=ransac_threshold_px, minimum_inlier_ratio=minimum_inlier_ratio,
                                 maximum_reprojection_px=maximum_reprojection_px)
    used_tags = tuple(ids[index] for index in sorted({index // 4 for index in result.inlier_indices}))
    return replace(result, tag_ids=used_tags)


def solve_keypoint_pose(object_points_m, image_points_px, camera_matrix, distortion, *, pixel_sigma_px=1.,
                        ransac_threshold_px=3., minimum_inlier_ratio=.6, maximum_reprojection_px=3.):
    """Known metric 3D/2D correspondences, robust PnP and local uncertainty.

    The object coordinate origin defines the reported target origin. No metric
    scale can be inferred from a bounding box or unlabeled texture alone.
    Covariance order is [translation in camera, right rotation in object].
    """
    matrix, distortion = validate_camera_parameters(camera_matrix, distortion)
    thresholds = [pixel_sigma_px, ransac_threshold_px, minimum_inlier_ratio, maximum_reprojection_px]
    if (not np.all(np.isfinite(distortion)) or not np.all(np.isfinite(thresholds))
            or min(pixel_sigma_px, ransac_threshold_px, maximum_reprojection_px) <= 0
            or not 0 < minimum_inlier_ratio <= 1):
        raise ValueError("invalid camera distortion or keypoint quality thresholds")
    objects, pixels = np.asarray(object_points_m, dtype=float), np.asarray(image_points_px, dtype=float)
    if (objects.ndim != 2 or objects.shape[1] != 3 or len(objects) < 4 or pixels.shape != (len(objects), 2)
            or not np.all(np.isfinite(objects)) or not np.all(np.isfinite(pixels))):
        raise ValueError("at least four finite metric 3D/2D keypoint pairs are required")
    singular = np.linalg.svd(objects - objects.mean(axis=0), compute_uv=False)
    if singular[1] < max(1e-9, singular[0] * 1e-7) or len(np.unique(pixels, axis=0)) < 4:
        raise ValueError("keypoint_geometry_degenerate")
    planar = singular[-1] < max(1e-9, singular[0] * 1e-7)
    if len(objects) > 4:
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            objects, pixels, matrix, distortion, iterationsCount=200,
            reprojectionError=float(ransac_threshold_px), confidence=.999, flags=cv2.SOLVEPNP_EPNP)
        if not ok or inliers is None:
            raise ValueError("reference_pnp_ransac_failed")
        indices = inliers.reshape(-1)
    else:
        indices = np.arange(4)
        method = cv2.SOLVEPNP_IPPE if planar else cv2.SOLVEPNP_AP3P
        ok, rvec, tvec = cv2.solvePnP(objects, pixels, matrix, distortion, flags=method)
        if not ok:
            raise ValueError("reference_pnp_failed")
    if len(indices) < 4:
        raise ValueError("reference_insufficient_inliers")
    retained, observed = objects[indices], pixels[indices]
    # Planar boards have two pose candidates. Reject genuinely distinct poses
    # with nearly indistinguishable image evidence rather than picking by order.
    singular = np.linalg.svd(retained - retained.mean(axis=0), compute_uv=False)
    if singular[-1] < max(1e-9, singular[0] * 1e-7) or len(retained) == 4:
        planar_inliers = singular[-1] < max(1e-9, singular[0] * 1e-7)
        hypotheses = cv2.solvePnPGeneric(retained, observed, matrix, distortion,
                                         flags=cv2.SOLVEPNP_IPPE if planar_inliers else cv2.SOLVEPNP_AP3P)
        candidates = []
        if hypotheses[0]:
            for candidate_r, candidate_t in zip(hypotheses[1], hypotheses[2]):
                candidate_rotation = cv2.Rodrigues(candidate_r)[0]
                if not np.all((retained @ candidate_rotation.T + candidate_t.reshape(3))[:, 2] > 0):
                    continue
                projection = cv2.projectPoints(retained, candidate_r, candidate_t, matrix, distortion)[0].reshape(-1, 2)
                error = float(np.sqrt(np.mean(np.sum((projection - observed) ** 2, axis=1))))
                candidates.append((error, candidate_r, candidate_t))
        candidates.sort(key=lambda item: item[0])
        if not candidates:
            raise ValueError("reference_has_no_positive_depth_pose")
        if len(candidates) > 1:
            first, second = candidates[:2]
            difference = cv2.Rodrigues(cv2.Rodrigues(first[1])[0].T @ cv2.Rodrigues(second[1])[0])[0]
            distinct = np.linalg.norm(difference) > np.radians(5) or np.linalg.norm(first[2] - second[2]) > .05
            if distinct and second[0] - first[0] < .2:
                raise ValueError("ambiguous_planar_reference_pose" if planar_inliers else "ambiguous_keypoint_pose")
        _, rvec, tvec = candidates[0]
    rvec, tvec = cv2.solvePnPRefineLM(retained, observed, matrix, distortion, rvec, tvec)
    # RANSAC's initial EPNP consensus can miss good planar points. Recompute
    # consensus with the refined geometry; only its retained points contribute
    # to the reported covariance and quality features.
    for _ in range(3):
        all_projected = cv2.projectPoints(objects, rvec, tvec, matrix, distortion)[0].reshape(-1, 2)
        depths = (objects @ cv2.Rodrigues(rvec)[0].T + tvec.reshape(3))[:, 2]
        refined_indices = np.flatnonzero((np.linalg.norm(all_projected - pixels, axis=1) <= ransac_threshold_px) & (depths > 0))
        if len(refined_indices) < 4:
            raise ValueError("reference_insufficient_inliers")
        if np.array_equal(indices, refined_indices):
            break
        indices = refined_indices
        retained, observed = objects[indices], pixels[indices]
        rvec, tvec = cv2.solvePnPRefineLM(retained, observed, matrix, distortion, rvec, tvec)
    if len(indices) / len(objects) < minimum_inlier_ratio:
        raise ValueError("reference_inlier_ratio_too_low")
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.reshape(3)
    if not np.all((retained @ transform[:3, :3].T + transform[:3, 3])[:, 2] > 0):
        raise ValueError("reference_has_negative_depth")

    def project(pose):
        return cv2.projectPoints(retained, cv2.Rodrigues(pose[:3, :3])[0], pose[:3, 3], matrix, distortion)[0].reshape(-1)

    residual = project(transform).reshape(-1, 2) - observed
    error = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    if not np.isfinite(error) or error > maximum_reprojection_px:
        raise ValueError("reference_reprojection_error_too_high")
    jacobian = np.column_stack([(project(perturb_transform(transform, axis, 1e-5))
                                 - project(perturb_transform(transform, axis, -1e-5))) / 2e-5
                                for axis in range(6)])
    information = jacobian.T @ jacobian / pixel_sigma_px ** 2
    if np.linalg.matrix_rank(information, tol=np.linalg.norm(information, 2) * 1e-10) < 6:
        raise ValueError("reference_pose_geometrically_unobservable")
    covariance = np.linalg.solve(information, np.eye(6))
    return BoardPose(transform, (covariance + covariance.T) / 2, error,
                     len(indices) / len(objects), len(indices), (), tuple(int(index) for index in indices))


def board_to_map_body(board_pose, transform_body_camera, extrinsic_covariance, reference_covariance):
    """Invert T_camera_map and camera extrinsic; return map body pose and ROS covariance."""
    camera_reference = rigid_transform(board_pose.transform_camera_reference)
    body_camera = rigid_transform(transform_body_camera)
    covariances = (pose_covariance(board_pose.covariance), pose_covariance(extrinsic_covariance))

    def compose(first, second):
        return np.linalg.inv(first) @ np.linalg.inv(second)

    output = compose(camera_reference, body_camera)
    jacobian = np.zeros((6, 12))
    epsilon = 1e-5
    for component in range(2):
        for axis in range(6):
            plus = [camera_reference, body_camera]
            minus = list(plus)
            plus[component] = perturb_transform(plus[component], axis, epsilon)
            minus[component] = perturb_transform(minus[component], axis, -epsilon)
            a, b = compose(*plus), compose(*minus)
            angle = cv2.Rodrigues(a[:3, :3] @ b[:3, :3].T)[0].reshape(3)
            jacobian[:, component * 6 + axis] = np.r_[a[:3, 3] - b[:3, 3], angle] / (2 * epsilon)
    covariance = sum(jacobian[:, index * 6:(index + 1) * 6] @ value
                     @ jacobian[:, index * 6:(index + 1) * 6].T for index, value in enumerate(covariances))
    reference_jacobian = np.eye(6)
    reference_jacobian[:3, 3:] = -_skew(output[:3, 3])
    covariance += reference_jacobian @ pose_covariance(reference_covariance) @ reference_jacobian.T
    return output, (covariance + covariance.T) / 2
