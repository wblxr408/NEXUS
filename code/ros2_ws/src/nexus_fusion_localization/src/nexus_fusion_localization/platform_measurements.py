"""SI/map observations of the platform, including monocular direction factors."""

from dataclasses import dataclass, field

import numpy as np

from .platform_state import covariance_matrix, inverse_right_jacobian_so3, log_so3, rotation_matrix, skew, vector


def _finish(measurement, dimension):
    if (isinstance(measurement.stamp_ns, bool) or int(measurement.stamp_ns) != measurement.stamp_ns
            or measurement.stamp_ns <= 0 or not measurement.source):
        raise ValueError("measurement requires sample time and source")
    if not np.isfinite(measurement.covariance_scale) or not 1 <= measurement.covariance_scale <= 100:
        raise ValueError("measurement covariance scale must be in [1, 100]")
    covariance = covariance_matrix(measurement.covariance, dimension)
    object.__setattr__(measurement, "covariance", covariance)
    object.__setattr__(measurement, "whitener", np.linalg.solve(np.linalg.cholesky(covariance), np.eye(dimension)))


class PlatformMeasurement:
    @property
    def stamps(self):
        return (self.stamp_ns,)

    @property
    def key(self):
        return type(self).__name__, self.source, self.stamps

    @property
    def degrees_of_freedom(self):
        return self.covariance.shape[0]

    def residual(self, states):
        return self.whitener @ self.raw_residual(states) / np.sqrt(self.covariance_scale)

    def jacobians(self, states):
        return {stamp: self.whitener @ matrix / np.sqrt(self.covariance_scale)
                for stamp, matrix in self.raw_jacobians(states).items()}


@dataclass(frozen=True)
class PositionMeasurement(PlatformMeasurement):
    stamp_ns: int
    position_m: np.ndarray
    covariance: np.ndarray
    tag_body_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    covariance_scale: float = 1.0
    source: str = "uwb_position"

    def __post_init__(self):
        _finish(self, 3)
        object.__setattr__(self, "position_m", vector(self.position_m, 3, "map UWB position"))
        object.__setattr__(self, "tag_body_m", vector(self.tag_body_m, 3, "body UWB lever arm"))

    def raw_residual(self, states):
        state = states[self.stamp_ns]
        return state.position_m + state.rotation_map_body @ self.tag_body_m - self.position_m

    def raw_jacobians(self, states):
        matrix = np.zeros((3, 15))
        matrix[:, :3] = np.eye(3)
        matrix[:, 6:9] = -states[self.stamp_ns].rotation_map_body @ skew(self.tag_body_m)
        return {self.stamp_ns: matrix}


@dataclass(frozen=True)
class PoseMeasurement(PlatformMeasurement):
    stamp_ns: int
    position_m: np.ndarray
    rotation_map_body: np.ndarray
    covariance: np.ndarray
    covariance_scale: float = 1.0
    source: str = "fixed_tag"

    def __post_init__(self):
        _finish(self, 6)
        object.__setattr__(self, "position_m", vector(self.position_m, 3, "map body position"))
        object.__setattr__(self, "rotation_map_body", rotation_matrix(self.rotation_map_body))

    def raw_residual(self, states):
        state = states[self.stamp_ns]
        return np.r_[state.position_m - self.position_m,
                     log_so3(self.rotation_map_body.T @ state.rotation_map_body)]

    def raw_jacobians(self, states):
        matrix = np.zeros((6, 15))
        matrix[:3, :3] = np.eye(3)
        matrix[3:, 6:9] = inverse_right_jacobian_so3(self.raw_residual(states)[3:])
        return {self.stamp_ns: matrix}


@dataclass(frozen=True)
class RangeMeasurement(PlatformMeasurement):
    stamp_ns: int
    anchors_map_m: np.ndarray
    ranges_m: np.ndarray
    covariance: np.ndarray
    tag_body_m: np.ndarray = field(default_factory=lambda: np.zeros(3))
    covariance_scale: float = 1.0
    source: str = "uwb_ranges"

    def __post_init__(self):
        anchors = np.asarray(self.anchors_map_m, dtype=float)
        if (anchors.ndim != 2 or anchors.shape[1] != 3 or len(anchors) < 4
                or not np.all(np.isfinite(anchors)) or len(np.unique(anchors, axis=0)) != len(anchors)):
            raise ValueError("range packet requires at least four distinct surveyed anchors")
        distances = vector(self.ranges_m, len(anchors), "UWB ranges in metres")
        if np.any(distances <= 0):
            raise ValueError("UWB ranges must be positive")
        _finish(self, len(anchors))
        object.__setattr__(self, "anchors_map_m", anchors.copy())
        object.__setattr__(self, "ranges_m", distances)
        object.__setattr__(self, "tag_body_m", vector(self.tag_body_m, 3, "body UWB lever arm"))

    def raw_residual(self, states):
        state = states[self.stamp_ns]
        tag = state.position_m + state.rotation_map_body @ self.tag_body_m
        return np.linalg.norm(tag - self.anchors_map_m, axis=1) - self.ranges_m

    def raw_jacobians(self, states):
        state = states[self.stamp_ns]
        difference = state.position_m + state.rotation_map_body @ self.tag_body_m - self.anchors_map_m
        distances = np.linalg.norm(difference, axis=1)
        if np.any(distances < 1e-9):
            raise ValueError("range derivative is undefined at an anchor")
        direction = difference / distances[:, None]
        matrix = np.zeros((len(distances), 15))
        matrix[:, :3] = direction
        matrix[:, 6:9] = -direction @ state.rotation_map_body @ skew(self.tag_body_m)
        return {self.stamp_ns: matrix}


@dataclass(frozen=True)
class RelativeMotionMeasurement(PlatformMeasurement):
    previous_stamp_ns: int
    stamp_ns: int
    rotation_i_j: np.ndarray
    translation_i_j: np.ndarray
    covariance: np.ndarray
    metric_translation: bool = False
    covariance_scale: float = 1.0
    source: str = "superpoint"
    sensor_body_m: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        _finish(self, 6)
        if not 0 < self.previous_stamp_ns < self.stamp_ns:
            raise ValueError("relative motion requires increasing sample times")
        if not isinstance(self.metric_translation, (bool, np.bool_)):
            raise ValueError("metric_translation must explicitly be boolean")
        object.__setattr__(self, "rotation_i_j", rotation_matrix(self.rotation_i_j))
        translation = vector(self.translation_i_j, 3, "relative translation")
        if not self.metric_translation:
            if np.linalg.norm(translation) < 1e-9:
                raise ValueError("monocular direction cannot be zero")
            translation /= np.linalg.norm(translation)
        object.__setattr__(self, "translation_i_j", translation)
        object.__setattr__(self, "sensor_body_m", vector(self.sensor_body_m, 3, "visual sensor lever arm"))

    @property
    def stamps(self):
        return self.previous_stamp_ns, self.stamp_ns

    @property
    def degrees_of_freedom(self):
        return 6 if self.metric_translation else 5

    def raw_residual(self, states):
        first, second = (states[stamp] for stamp in self.stamps)
        relative_rotation = first.rotation_map_body.T @ second.rotation_map_body
        translation = (first.rotation_map_body.T @ (second.position_m - first.position_m)
                       + relative_rotation @ self.sensor_body_m - self.sensor_body_m)
        if not self.metric_translation:
            if np.linalg.norm(translation) < 1e-8:
                raise ValueError("visual translation direction is unobservable at zero baseline")
            translation /= np.linalg.norm(translation)
        return np.r_[translation - self.translation_i_j,
                     log_so3(self.rotation_i_j.T @ first.rotation_map_body.T @ second.rotation_map_body)]

    def raw_jacobians(self, states):
        first, second = (states[stamp] for stamp in self.stamps)
        relative_rotation = first.rotation_map_body.T @ second.rotation_map_body
        translation = (first.rotation_map_body.T @ (second.position_m - first.position_m)
                       + relative_rotation @ self.sensor_body_m - self.sensor_body_m)
        projection = np.eye(3)
        if not self.metric_translation:
            distance = np.linalg.norm(translation)
            if distance < 1e-8:
                raise ValueError("visual translation direction is unobservable at zero baseline")
            direction = translation / distance
            projection = (np.eye(3) - np.outer(direction, direction)) / distance
        left, right = np.zeros((6, 15)), np.zeros((6, 15))
        left[:3, :3] = -projection @ first.rotation_map_body.T
        right[:3, :3] = -left[:3, :3]
        left[:3, 6:9] = projection @ skew(translation + self.sensor_body_m)
        right[:3, 6:9] = -projection @ relative_rotation @ skew(self.sensor_body_m)
        angle = self.raw_residual(states)[3:]
        left[3:, 6:9] = -inverse_right_jacobian_so3(-angle) @ self.rotation_i_j.T
        right[3:, 6:9] = inverse_right_jacobian_so3(angle)
        return {self.previous_stamp_ns: left, self.stamp_ns: right}
