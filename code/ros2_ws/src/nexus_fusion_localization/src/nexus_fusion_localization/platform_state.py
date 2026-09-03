"""Platform state on SO(3); local covariance order p, v, theta, ba, bg."""

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} requires {size} finite values")
    return result.copy()


def rotation_matrix(value):
    result = np.asarray(value, dtype=float)
    if (result.shape != (3, 3) or not np.all(np.isfinite(result))
            or not np.allclose(result.T @ result, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(result), 1., atol=1e-6)):
        raise ValueError("rotation must be a finite proper orthonormal matrix")
    return result.copy()


def covariance_matrix(value, size):
    result = np.asarray(value, dtype=float)
    if (result.shape != (size, size) or not np.all(np.isfinite(result))
            or not np.allclose(result, result.T, atol=1e-9)):
        raise ValueError(f"covariance must be finite symmetric {size} x {size}")
    try:
        np.linalg.cholesky(result)
    except np.linalg.LinAlgError as error:
        raise ValueError("measurement covariance must be positive definite") from error
    return result.copy()


def skew(value):
    x, y, z = value
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_so3(value):
    return Rotation.from_rotvec(value).as_matrix()


def log_so3(value):
    return Rotation.from_matrix(value).as_rotvec()


def right_jacobian_so3(value):
    """Map additive rotation-vector changes to right local SO(3) increments."""
    angle_squared = float(np.dot(value, value))
    cross = skew(value)
    if angle_squared < 1e-8:
        a = .5 - angle_squared / 24 + angle_squared ** 2 / 720
        b = 1 / 6 - angle_squared / 120 + angle_squared ** 2 / 5040
    else:
        angle = np.sqrt(angle_squared)
        a = (1 - np.cos(angle)) / angle_squared
        b = (angle - np.sin(angle)) / (angle_squared * angle)
    return np.eye(3) - a * cross + b * cross @ cross


def inverse_right_jacobian_so3(value):
    """Derivative of Log(R Exp(delta)) at delta=0, away from the pi cut."""
    angle_squared = float(np.dot(value, value))
    cross = skew(value)
    if angle_squared < 1e-8:
        coefficient = 1 / 12 + angle_squared / 720 + angle_squared ** 2 / 30240
    else:
        angle = np.sqrt(angle_squared)
        coefficient = (1 - .5 * angle / np.tan(.5 * angle)) / angle_squared
    return np.eye(3) + .5 * cross + coefficient * cross @ cross


@dataclass(frozen=True)
class BodyState:
    stamp_ns: int
    position_m: np.ndarray
    velocity_mps: np.ndarray
    rotation_map_body: np.ndarray
    accel_bias_mps2: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias_rps: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        if isinstance(self.stamp_ns, bool) or int(self.stamp_ns) != self.stamp_ns or self.stamp_ns <= 0:
            raise ValueError("platform sample timestamp must be a positive integer")
        for name in ("position_m", "velocity_mps", "accel_bias_mps2", "gyro_bias_rps"):
            object.__setattr__(self, name, vector(getattr(self, name), 3, name))
        object.__setattr__(self, "rotation_map_body", rotation_matrix(self.rotation_map_body))

    def retract(self, delta):
        delta = vector(delta, 15, "state increment")
        # Both the reference and increment have already been validated. Their
        # SO(3) product remains a proper rotation; repeating an orthogonality
        # test for every optimizer trial is unnecessary. Public construction
        # still validates all incoming sensor/initialization state components.
        values = (self.position_m + delta[:3], self.velocity_mps + delta[3:6],
                  self.rotation_map_body @ exp_so3(delta[6:9]),
                  self.accel_bias_mps2 + delta[9:12], self.gyro_bias_rps + delta[12:15])
        if not all(np.all(np.isfinite(value)) for value in values):
            raise ValueError("nonfinite retracted state")
        result = object.__new__(BodyState)
        object.__setattr__(result, "stamp_ns", self.stamp_ns)
        for name, value in zip(("position_m", "velocity_mps", "rotation_map_body",
                                "accel_bias_mps2", "gyro_bias_rps"), values):
            object.__setattr__(result, name, value)
        return result

    def local(self, other):
        return np.concatenate((other.position_m - self.position_m, other.velocity_mps - self.velocity_mps,
                               log_so3(self.rotation_map_body.T @ other.rotation_map_body),
                               other.accel_bias_mps2 - self.accel_bias_mps2,
                               other.gyro_bias_rps - self.gyro_bias_rps))


def pose_covariance_to_ros(state, covariance):
    """Internal body-angle covariance to ROS fixed-map position/angle axes."""
    indices = [0, 1, 2, 6, 7, 8]
    pose = np.asarray(covariance)[np.ix_(indices, indices)]
    transform = np.eye(6)
    transform[3:, 3:] = state.rotation_map_body
    return transform @ pose @ transform.T


def pose_covariance_from_ros(rotation_map_body, covariance):
    transform = np.eye(6)
    transform[3:, 3:] = rotation_matrix(rotation_map_body).T
    matrix = covariance_matrix(covariance, 6)
    return transform @ matrix @ transform.T
