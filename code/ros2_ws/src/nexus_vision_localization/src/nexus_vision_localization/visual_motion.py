"""Calibrated two-view motion from static matches, with explicit scale limits."""

from dataclasses import dataclass

import cv2
import numpy as np

from .detector import validate_camera_parameters
from .reference_geometry import rigid_transform
from .superpoint_frontend import feature_quality, match_features


def _skew(value):
    x, y, z = value
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


@dataclass(frozen=True)
class MotionConfig:
    pixel_sigma_px: float = 1.
    ransac_threshold_px: float = 1.5
    maximum_reprojection_px: float = 2.5
    minimum_matches: int = 12
    minimum_inlier_ratio: float = .6
    minimum_coverage: float = .02
    minimum_parallax_deg: float = .2
    maximum_rotation_disagreement_rad: float = .25
    maximum_information_condition: float = 1e10

    def __post_init__(self):
        if (not all(np.isfinite(value) and value > 0 for value in self.__dict__.values())
                or self.minimum_matches < 8 or int(self.minimum_matches) != self.minimum_matches
                or self.minimum_inlier_ratio > 1 or self.minimum_coverage > 1):
            raise ValueError("invalid two-view geometry configuration")


@dataclass(frozen=True)
class VisualMotion:
    rotation_camera_i_j: np.ndarray
    direction_camera_i_j: np.ndarray
    covariance: np.ndarray  # camera_i direction and camera_j right angle; no metric scale
    previous_indices: np.ndarray
    current_indices: np.ndarray
    median_parallax_deg: float
    quality: dict

    def body_observation(self, previous_stamp_ns, stamp_ns, transform_body_camera):
        if not 0 < previous_stamp_ns < stamp_ns:
            raise ValueError("visual motion requires increasing positive sample timestamps")
        transform = rigid_transform(transform_body_camera)
        rotation = transform[:3, :3]
        axes = np.zeros((6, 6))
        axes[:3, :3] = axes[3:, 3:] = rotation
        return {
            "schema_version": 1, "frame_id": "base_link", "previous_stamp_ns": previous_stamp_ns,
            "sample_timestamp_ns": stamp_ns, "metric_translation": False,
            "rotation_i_j": (rotation @ self.rotation_camera_i_j @ rotation.T).tolist(),
            "translation_i_j": (rotation @ self.direction_camera_i_j).tolist(),
            "sensor_body_m": transform[:3, 3].tolist(),
            "covariance": (axes @ self.covariance @ axes.T).reshape(-1).tolist(),
        }


def _triangulate(rotation_j_i, translation_j_i, first, second):
    p1 = np.column_stack((np.eye(3), np.zeros(3)))
    p2 = np.column_stack((rotation_j_i, translation_j_i))
    homogeneous = cv2.triangulatePoints(p1, p2, first.T, second.T)
    finite = np.abs(homogeneous[3]) > 1e-10
    points = np.zeros((len(first), 3))
    points[finite] = (homogeneous[:3, finite] / homogeneous[3, finite]).T
    transformed = points @ rotation_j_i.T + translation_j_i
    positive = finite & (points[:, 2] > 1e-8) & (transformed[:, 2] > 1e-8)
    errors = np.full(len(first), np.inf)
    errors[positive] = np.sqrt((
        np.sum((points[positive, :2] / points[positive, 2:3] - first[positive]) ** 2, axis=1)
        + np.sum((transformed[positive, :2] / transformed[positive, 2:3] - second[positive]) ** 2, axis=1)) / 2)
    return positive, errors


def _direction_covariance(rotation_i_j, direction, first, second, sigma_normalized, maximum_condition):
    # Five observable parameters: three rotation increments and two tangent
    # directions on the unit sphere. Metric translation is absent by design.
    _, _, vh = np.linalg.svd(direction[None])
    tangent = vh[1:].T
    rays_i, rays_j = np.column_stack((first, np.ones(len(first)))), np.column_stack((second, np.ones(len(second))))

    def residual(delta):
        translation = direction + tangent @ delta[3:]
        translation /= np.linalg.norm(translation)
        rotation = rotation_i_j @ cv2.Rodrigues(delta[:3])[0]
        essential = _skew(translation) @ rotation
        line_i, line_j = rays_j @ essential.T, rays_i @ essential
        denominator = np.sqrt(np.sum(line_i[:, :2] ** 2 + line_j[:, :2] ** 2, axis=1))
        if np.any(denominator < 1e-10):
            raise ValueError("degenerate epipolar geometry")
        return np.sum(rays_i * line_i, axis=1) / denominator

    jacobian = np.zeros((len(first), 5))
    for axis in range(5):
        delta = np.zeros(5)
        delta[axis] = 1e-5
        jacobian[:, axis] = (residual(delta) - residual(-delta)) / 2e-5
    information = jacobian.T @ jacobian / sigma_normalized ** 2
    eigenvalues = np.linalg.eigvalsh(information)
    if eigenvalues[0] <= 0 or eigenvalues[-1] / eigenvalues[0] > maximum_condition:
        raise ValueError("two-view rotation/direction is geometrically ill-conditioned")
    covariance = np.linalg.solve(information, np.eye(5))
    mapping = np.zeros((6, 5))
    mapping[:3, 3:] = tangent
    mapping[3:, :3] = np.eye(3)
    result = mapping @ covariance @ mapping.T
    # The radial direction has no first-order unit-direction residual; give it
    # finite weak information for a 6x6 interface, never a metric length claim.
    result[:3, :3] += np.outer(direction, direction)
    return (result + result.T) / 2


def estimate_static_motion(previous, current, camera_matrix, distortion, *, config=None, predicted_rotation_i_j=None):
    config = config or MotionConfig()
    matrix, distortion = validate_camera_parameters(camera_matrix, distortion)
    if not np.all(np.isfinite(distortion)) or previous.image_size_wh != current.image_size_wh:
        raise ValueError("two-view camera geometry changed or is invalid")
    matches = match_features(previous, current)
    if len(matches.distances) < config.minimum_matches:
        raise ValueError("insufficient mutual static matches")
    original_first, original_second = previous.points_px[matches.previous_indices], current.points_px[matches.current_indices]
    first = cv2.undistortPoints(original_first.reshape(-1, 1, 2), matrix, distortion).reshape(-1, 2).astype(float)
    second = cv2.undistortPoints(original_second.reshape(-1, 1, 2), matrix, distortion).reshape(-1, 2).astype(float)
    focal = float(np.sqrt(matrix[0, 0] * matrix[1, 1]))
    essential, inlier_mask = cv2.findEssentialMat(
        first, second, np.eye(3), method=cv2.RANSAC, prob=.999, threshold=config.ransac_threshold_px / focal)
    if essential is None or inlier_mask is None:
        raise ValueError("essential matrix estimation failed")
    ransac_inliers = inlier_mask.reshape(-1).astype(bool)
    candidates = []
    for candidate in essential.reshape(-1, 3, 3):
        r1, r2, direction = cv2.decomposeEssentialMat(candidate)
        for rotation in (r1, r2):
            for sign in (1., -1.):
                translation = direction.reshape(3) * sign
                positive, errors = _triangulate(rotation, translation, first, second)
                keep = ransac_inliers & positive & (errors * focal <= config.maximum_reprojection_px)
                candidates.append((int(keep.sum()), rotation, translation, keep, errors))
    count, rotation_j_i, translation_j_i, keep, errors = max(candidates, key=lambda item: item[0])
    if count < config.minimum_matches or count / len(matches.distances) < config.minimum_inlier_ratio:
        raise ValueError("insufficient positive-depth geometric inliers")
    rotation_i_j = rotation_j_i.T
    direction = -rotation_i_j @ translation_j_i
    direction /= np.linalg.norm(direction)
    rays_i, rays_j = np.column_stack((first[keep], np.ones(count))), np.column_stack((second[keep], np.ones(count)))
    rays_i /= np.linalg.norm(rays_i, axis=1, keepdims=True)
    rays_j /= np.linalg.norm(rays_j, axis=1, keepdims=True)
    angles = np.rad2deg(np.arccos(np.clip(np.sum(rays_i * (rays_j @ rotation_i_j.T), axis=1), -1., 1.)))
    parallax = float(np.median(angles))
    if parallax < config.minimum_parallax_deg:
        raise ValueError("insufficient translation parallax")
    if predicted_rotation_i_j is not None:
        predicted = np.eye(4)
        predicted[:3, :3] = predicted_rotation_i_j
        rigid_transform(predicted)
        angle = np.arccos(np.clip((np.trace(predicted[:3, :3].T @ rotation_i_j) - 1) / 2, -1., 1.))
        if angle > config.maximum_rotation_disagreement_rad:
            raise ValueError("visual rotation conflicts with IMU prediction")
    selected = current.select(matches.current_indices[keep])
    quality = feature_quality(selected)
    if quality["feature_coverage"] < config.minimum_coverage:
        raise ValueError("static inliers have insufficient spatial coverage")
    quality.update({"inlier_ratio": count / len(matches.distances),
                    "match_distance": float(np.median(matches.distances[keep])),
                    "reprojection_error_px": float(np.sqrt(np.mean((errors[keep] * focal) ** 2)))})
    covariance = _direction_covariance(rotation_i_j, direction, first[keep], second[keep],
                                       config.pixel_sigma_px / focal, config.maximum_information_condition)
    return VisualMotion(rotation_i_j, direction, covariance, matches.previous_indices[keep],
                        matches.current_indices[keep], parallax, quality)
