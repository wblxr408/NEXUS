"""Robust metric reconstruction of a fixed reference feature on a target.

Static 3D and constant-velocity 6D states have separate observability checks.
Neither a detection box center nor camera motion alone supplies target depth.
"""

from dataclasses import dataclass
from itertools import combinations
from math import comb

import cv2
import numpy as np
from scipy.optimize import least_squares

from .detector import validate_camera_parameters
from .pose_timeline import TimedPose
from .reference_geometry import pose_covariance, rigid_transform
from .target_geometry import compose_target_pose, _skew


@dataclass(frozen=True)
class BearingObservation:
    platform: TimedPose
    pixel: np.ndarray
    camera_matrix: np.ndarray
    distortion: np.ndarray
    pixel_sigma_px: float = 1.

    def __post_init__(self):
        matrix, distortion = validate_camera_parameters(self.camera_matrix, self.distortion)
        pixel = np.asarray(self.pixel, dtype=float)
        if (pixel.shape != (2,) or not np.all(np.isfinite(pixel)) or not np.all(np.isfinite(distortion))
                or not np.isfinite(self.pixel_sigma_px) or self.pixel_sigma_px <= 0
                or not np.allclose(matrix[2], [0., 0., 1.]) or not isinstance(self.platform, TimedPose)):
            raise ValueError("invalid target pixel/calibration/platform observation")
        object.__setattr__(self, "pixel", pixel.copy())
        object.__setattr__(self, "camera_matrix", matrix.copy())
        object.__setattr__(self, "distortion", distortion.copy())


@dataclass(frozen=True)
class MultiviewConfig:
    minimum_views: int = 3
    minimum_inlier_ratio: float = .6
    maximum_reprojection_px: float = 3.
    minimum_parallax_deg: float = .25
    maximum_condition_number: float = 1e7
    maximum_speed_mps: float = 10.
    minimum_depth_m: float = .05
    maximum_depth_m: float = 100.
    ransac_iterations: int = 64

    def __post_init__(self):
        if (any(not np.isfinite(v) or v <= 0 for v in self.__dict__.values())
                or not 0 < self.minimum_inlier_ratio <= 1 or self.minimum_views < 2
                or self.maximum_depth_m <= self.minimum_depth_m
                or any(not isinstance(v, int) for v in (self.minimum_views, self.ransac_iterations))):
            raise ValueError("invalid multiview geometry thresholds")


@dataclass(frozen=True)
class MultiviewEstimate:
    stamp_ns: int
    position_m: np.ndarray
    velocity_mps: np.ndarray | None
    covariance: np.ndarray  # [p,v] when velocity observed; otherwise position only
    inlier_stamps: tuple
    reprojection_rmse_px: float
    parallax_deg: float
    condition_number: float
    motion_model: str


def _prepared(observation, extrinsic):
    zero = np.zeros((6, 6))
    camera, covariance = compose_target_pose(
        observation.platform.transform, extrinsic, np.eye(4), observation.platform.covariance, zero, zero)

    def undistort(pixel):
        return cv2.undistortPoints(
            np.asarray(pixel).reshape(1, 1, 2), observation.camera_matrix,
            observation.distortion, P=observation.camera_matrix).reshape(2)

    measured = undistort(observation.pixel)
    epsilon = 1e-3
    derivative = np.column_stack([(undistort(observation.pixel + np.eye(2)[axis] * epsilon)
                                  - undistort(observation.pixel - np.eye(2)[axis] * epsilon)) / (2 * epsilon) for axis in range(2)])
    pixel_covariance = derivative @ derivative.T * observation.pixel_sigma_px ** 2
    return camera, covariance, measured, pixel_covariance


def solve_multiview(observations, transform_body_camera, extrinsic_covariance, *, motion_model="static", config=None):
    config = config or MultiviewConfig()
    observations = list(observations)
    if motion_model not in {"static", "constant_velocity"}:
        raise ValueError("target motion_model must be static or constant_velocity")
    dimension = 3 if motion_model == "static" else 6
    minimum = max(config.minimum_views, 4 if dimension == 6 else 2)
    if len(observations) < minimum:
        raise ValueError("insufficient_target_views")
    stamps = [item.platform.stamp_ns for item in observations]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("target view timestamps must be strictly increasing")
    extrinsic = rigid_transform(transform_body_camera)
    extrinsic_covariance = pose_covariance(extrinsic_covariance)
    prepared = [_prepared(item, extrinsic) for item in observations]
    times = (np.array([stamp - stamps[-1] for stamp in stamps], dtype=float)) / 1e9
    # Time normalization makes rank/conditioning independent of seconds vs
    # milliseconds. The final velocity and covariance are converted to SI.
    duration = max(-times[0], 1e-9)
    designs = [np.eye(3) if dimension == 3 else np.column_stack((np.eye(3), np.eye(3) * dt / duration)) for dt in times]
    rotations = [item[0][:3, :3] for item in prepared]
    centers = [item[0][:3, 3] for item in prepared]
    linear, rhs = [], []
    for observation, (_, _, measured, _), rotation, center, design in zip(observations, prepared, rotations, centers, designs):
        ray = np.linalg.solve(observation.camera_matrix, np.r_[measured, 1.])
        projection = np.array([[1., 0., -ray[0]], [0., 1., -ray[1]]]) @ rotation.T
        linear.append(projection @ design)
        rhs.append(projection @ center)

    def project(state, index, derivatives=False):
        point = designs[index] @ state
        camera_point = rotations[index].T @ (point - centers[index])
        z = camera_point[2]
        # Keep the objective finite if an optimizer trial crosses the camera
        # plane; final depth/inlier checks still reject such a solution.
        safe_z = max(z, config.minimum_depth_m * .1)
        matrix = observations[index].camera_matrix
        prediction = matrix[:2, :2] @ (camera_point[:2] / safe_z) + matrix[:2, 2]
        if not derivatives:
            return prediction, z
        derivative = matrix[:2, :2] @ np.array([[1 / safe_z, 0., -camera_point[0] / safe_z ** 2],
                                                [0., 1 / safe_z, -camera_point[1] / safe_z ** 2]])
        state_jacobian = derivative @ rotations[index].T @ designs[index]
        pose_jacobian = derivative @ np.column_stack((-rotations[index].T, rotations[index].T @ _skew(point - centers[index])))
        return prediction, z, state_jacobian, pose_jacobian

    def consensus(state):
        errors, keep = [], []
        for index, (_, _, measured, _) in enumerate(prepared):
            predicted, depth = project(state, index)
            error = float(np.linalg.norm(predicted - measured))
            errors.append(error)
            keep.append(config.minimum_depth_m < depth < config.maximum_depth_m and error <= config.maximum_reprojection_px)
        return np.flatnonzero(keep), np.array(errors)

    minimal = 2 if dimension == 3 else 4
    if comb(len(observations), minimal) <= config.ransac_iterations:
        samples = list(combinations(range(len(observations)), minimal))
    else:
        rng = np.random.default_rng(0)
        unique = set()
        for _ in range(config.ransac_iterations * 5):
            unique.add(tuple(sorted(int(i) for i in rng.choice(len(observations), minimal, replace=False))))
            if len(unique) == config.ransac_iterations:
                break
        samples = sorted(unique)
    samples.insert(0, tuple(range(len(observations))))
    best = None
    for sample in samples:
        matrix = np.vstack([linear[index] for index in sample])
        values = np.concatenate([rhs[index] for index in sample])
        state, _, rank, singular = np.linalg.lstsq(matrix, values, rcond=1 / config.maximum_condition_number)
        if rank != dimension:
            continue
        if dimension == 6 and np.linalg.norm(state[3:] / duration) > config.maximum_speed_mps:
            continue
        indices, errors = consensus(state)
        score = (len(indices), -float(np.sum(np.minimum(errors, config.maximum_reprojection_px) ** 2)))
        if best is None or score > best[0]:
            best = (score, state, indices)
    if best is None:
        raise ValueError("target_motion_geometrically_unobservable")
    _, state, indices = best
    if len(indices) < minimum or len(indices) / len(observations) < config.minimum_inlier_ratio:
        raise ValueError("target_geometric_consensus_insufficient")
    # Iterate covariance-dependent weighting, then independently verify the
    # final current frame and all retained constraints.
    for _ in range(2):
        whitening = []
        for index in indices:
            _, _, _, pose_jacobian = project(state, index, True)
            noise = prepared[index][3] + pose_jacobian @ prepared[index][1] @ pose_jacobian.T
            whitening.append(np.linalg.solve(np.linalg.cholesky(noise), np.eye(2)))

        def residual(value):
            return np.concatenate([w @ (project(value, index)[0] - prepared[index][2]) for index, w in zip(indices, whitening)])

        def derivative(value):
            return np.vstack([w @ project(value, index, True)[2] for index, w in zip(indices, whitening)])

        result = least_squares(residual, state, jac=derivative, loss="huber", f_scale=1.5, max_nfev=40)
        if not result.success or not np.all(np.isfinite(result.x)):
            raise ValueError("target_multiview_optimization_failed")
        state = result.x
        indices, errors = consensus(state)
        if len(indices) < minimum or len(indices) / len(observations) < config.minimum_inlier_ratio:
            raise ValueError("target_geometric_consensus_insufficient")
    if len(observations) - 1 not in indices:
        raise ValueError("current_target_observation_is_outlier")
    if dimension == 6 and np.linalg.norm(state[3:] / duration) > config.maximum_speed_mps:
        raise ValueError("target_speed_exceeds_limit")
    rays = [designs[index] @ state - centers[index] for index in indices]
    rays = np.array([ray / np.linalg.norm(ray) for ray in rays])
    parallax = float(np.degrees(np.arccos(np.clip(np.min(rays @ rays.T), -1., 1.))))
    if parallax < config.minimum_parallax_deg:
        raise ValueError("target_parallax_insufficient")
    information = np.zeros((dimension, dimension))
    jacobians, weights = [], []
    for index in indices:
        predicted, _, j_state, j_pose = project(state, index, True)
        noise = prepared[index][3] + j_pose @ prepared[index][1] @ j_pose.T
        weight = np.linalg.solve(noise, np.eye(2))
        distance = np.sqrt((predicted - prepared[index][2]) @ weight @ (predicted - prepared[index][2]))
        weight *= min(1., 1.5 / max(distance, 1e-12))
        information += j_state.T @ weight @ j_state
        jacobians.append((j_state, j_pose))
        weights.append(weight)
    eigenvalues = np.linalg.eigvalsh(information)
    condition = float(np.sqrt(eigenvalues[-1] / max(eigenvalues[0], 1e-30)))
    if eigenvalues[0] <= 0 or condition > config.maximum_condition_number:
        raise ValueError("target_motion_geometrically_unobservable")
    inverse = np.linalg.solve(information, np.eye(dimension))
    pixel_part, platform_part, shared_extrinsic = np.zeros_like(inverse), np.zeros_like(inverse), np.zeros((dimension, 6))
    for index, (j_state, j_pose), weight in zip(indices, jacobians, weights):
        gain = inverse @ j_state.T @ weight
        pixel_part += gain @ prepared[index][3] @ gain.T
        platform_part += gain @ j_pose @ prepared[index][1] @ j_pose.T @ gain.T
        extrinsic_jacobian = np.zeros((6, 6))
        extrinsic_jacobian[:3, :3] = observations[index].platform.transform[:3, :3]
        extrinsic_jacobian[3:, 3:] = rotations[index]
        shared_extrinsic += gain @ j_pose @ extrinsic_jacobian
    # Platform estimates share history, so do not shrink their common error as
    # if all poses were independent. n*sum(P_i) is a conservative upper bound
    # for unknown cross-correlation. Extrinsic error is one shared variable.
    covariance = pixel_part + len(indices) * platform_part + shared_extrinsic @ extrinsic_covariance @ shared_extrinsic.T
    conversion = np.eye(dimension)
    if dimension == 6:
        conversion[3:, 3:] /= duration
    covariance = conversion @ covariance @ conversion.T
    return MultiviewEstimate(stamps[-1], state[:3].copy(), None if dimension == 3 else state[3:] / duration,
                             (covariance + covariance.T) / 2, tuple(stamps[index] for index in indices),
                             float(np.sqrt(np.mean(errors[indices] ** 2))), parallax, condition, motion_model)
