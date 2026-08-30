"""EKF and UKF UWB positioning adapted from the MATLAB project.

Source: https://github.com/cliansang/positioning-algorithms-for-uwb-matlab
The state is ``[x, y, z, vx, vy, vz]`` and each range is a 3D Euclidean
measurement to a fixed anchor. State is retained between sequential calls,
matching the MATLAB demos' filter behavior.
"""

from __future__ import annotations

import time

import numpy as np

from algorithms.contracts import AlgorithmResult


DT = 0.1
STATE_SIZE = 6


def _initial_position(initial_xyz, initial_xy, anchors):
    if initial_xy is not None:
        xy = np.asarray(initial_xy, dtype=float).reshape(-1)
        if xy.size != 2 or not np.all(np.isfinite(xy)):
            raise ValueError("initial_xy must contain two finite coordinates")
        supplied = np.asarray(initial_xyz, dtype=float).reshape(-1)
        z = supplied[2] if supplied.size == 3 else float(np.mean(anchors[:, 2]))
        position = np.array([xy[0], xy[1], z], dtype=float)
    else:
        position = np.asarray(initial_xyz, dtype=float).reshape(-1)
    if position.size != 3 or not np.all(np.isfinite(position)):
        raise ValueError("initial_xyz must contain three finite coordinates")
    return position


def _measurement_arrays(observation_frame):
    ranges = np.asarray([sample.range_m for sample in observation_frame.ranges], dtype=float)
    anchors = np.asarray([sample.anchor_position_m for sample in observation_frame.ranges], dtype=float)
    sigmas = np.asarray([sample.stddev_m for sample in observation_frame.ranges], dtype=float)
    if ranges.size < 4 or anchors.shape != (ranges.size, 3):
        raise ValueError("at least four 3D anchor-range pairs are required")
    if not np.all(np.isfinite(anchors)) or not np.all(np.isfinite(ranges)):
        raise ValueError("anchors and ranges must be finite")
    if np.any(ranges <= 0.0) or not np.all(np.isfinite(sigmas)) or np.any(sigmas < 0.0):
        raise ValueError("ranges must be positive and standard deviations non-negative")
    if np.linalg.matrix_rank(anchors[1:] - anchors[0]) < 3:
        raise ValueError("3D filter anchors are coplanar or rank deficient")
    return ranges, anchors, sigmas


class RangeFilterBase:
    def __init__(self, initial_xyz, dt=DT):
        self.x = np.zeros(STATE_SIZE, dtype=float)
        self.x[:3] = np.asarray(initial_xyz, dtype=float)
        self.P = np.eye(STATE_SIZE, dtype=float) * 2.0
        self.dt = float(dt)
        self.last_timestamp_ns = None
        self.A = np.eye(STATE_SIZE, dtype=float)
        self.Q = np.zeros((STATE_SIZE, STATE_SIZE), dtype=float)
        self.set_dt(self.dt)

    def set_dt(self, dt):
        self.dt = max(float(dt), 1e-4)
        self.A = np.eye(STATE_SIZE, dtype=float)
        self.A[:3, 3:] = np.eye(3) * self.dt
        q_position = self.dt ** 4 / 4.0
        q_cross = self.dt ** 3 / 2.0
        q_velocity = self.dt ** 2
        self.Q = np.zeros((STATE_SIZE, STATE_SIZE), dtype=float)
        self.Q[:3, :3] = np.eye(3) * q_position
        self.Q[:3, 3:] = np.eye(3) * q_cross
        self.Q[3:, :3] = np.eye(3) * q_cross
        self.Q[3:, 3:] = np.eye(3) * q_velocity

    @staticmethod
    def _h(x_state, anchors):
        diff = x_state[:3] - anchors[:, :3]
        return np.maximum(np.linalg.norm(diff, axis=1), 1e-9)

    @staticmethod
    def _H_jacobian(x_state, anchors):
        diff = x_state[:3] - anchors[:, :3]
        norms = np.maximum(np.linalg.norm(diff, axis=1), 1e-9)
        return np.hstack([diff / norms[:, None], np.zeros((len(anchors), 3))])

    def predict(self, timestamp_ns):
        if self.last_timestamp_ns is None:
            self.last_timestamp_ns = int(timestamp_ns)
            return self.x.copy(), self.P.copy()
        dt = max((int(timestamp_ns) - self.last_timestamp_ns) / 1e9, 1e-4)
        self.set_dt(dt)
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)
        self.last_timestamp_ns = int(timestamp_ns)
        return self.x.copy(), self.P.copy()


def _measurement_covariance(sigmas):
    # A zero reported sigma is treated as a very precise but non-singular
    # measurement so deterministic CARLA frames remain numerically valid.
    return np.diag(np.maximum(sigmas, 0.01) ** 2)


def run_ekf(*, observation_frame, initial_xyz=(0.0, 0.0, 2.0), initial_xy=None, **_):
    """Run one sequential 3D extended Kalman filter update."""
    start_ns = time.monotonic_ns()
    try:
        ranges, anchors, sigmas = _measurement_arrays(observation_frame)
        state = getattr(run_ekf, "_state", None)
        if state is None:
            state = RangeFilterBase(_initial_position(initial_xyz, initial_xy, anchors))
            run_ekf._state = state

        x_pred, p_pred = state.predict(observation_frame.timestamp_ns)
        H = state._H_jacobian(x_pred, anchors)
        innovation = ranges - state._h(x_pred, anchors)
        R = _measurement_covariance(sigmas)
        S = H @ p_pred @ H.T + R
        gain = np.linalg.solve(S, H @ p_pred).T
        state.x = x_pred + gain @ innovation
        identity = np.eye(STATE_SIZE)
        state.P = (identity - gain @ H) @ p_pred
        state.P = 0.5 * (state.P + state.P.T)
        estimate = state.x[:3].copy()
        covariance = state.P[:3, :3].copy()
        metadata = {"dimension": "3D", "timestamp_ns": observation_frame.timestamp_ns}
        valid = bool(np.all(np.isfinite(estimate)))
    except (ValueError, np.linalg.LinAlgError, FloatingPointError) as exc:
        estimate = np.full(3, np.nan)
        covariance = np.full((3, 3), np.nan)
        metadata = {"error": str(exc), "timestamp_ns": observation_frame.timestamp_ns}
        valid = False
    metadata["runtime_ms"] = (time.monotonic_ns() - start_ns) / 1e6
    return AlgorithmResult(
        estimate=np.asarray(estimate), covariance=np.asarray(covariance),
        algorithm="uwb.matlab.ekf", family="uwb", valid=valid, metadata=metadata,
    )


def _sigma_points(mean, covariance, denominator):
    covariance = 0.5 * (covariance + covariance.T)
    try:
        root = np.linalg.cholesky(denominator * covariance)
    except np.linalg.LinAlgError:
        root = np.linalg.cholesky(
            denominator * (covariance + np.eye(STATE_SIZE, dtype=float) * 1e-9)
        )
    points = np.empty((2 * STATE_SIZE + 1, STATE_SIZE), dtype=float)
    points[0] = mean
    for index in range(STATE_SIZE):
        points[1 + index] = mean + root[:, index]
        points[1 + STATE_SIZE + index] = mean - root[:, index]
    return points


def run_ukf(*, observation_frame, initial_xyz=(0.0, 0.0, 2.0), initial_xy=None, **_):
    """Run one sequential 3D unscented Kalman filter update."""
    start_ns = time.monotonic_ns()
    try:
        ranges, anchors, sigmas = _measurement_arrays(observation_frame)
        state = getattr(run_ukf, "_state", None)
        if state is None:
            state = RangeFilterBase(_initial_position(initial_xyz, initial_xy, anchors))
            run_ukf._state = state

        # Predict sigma points from the previous posterior with the same
        # constant-velocity model used by the EKF.
        nx = STATE_SIZE
        alpha, beta, kappa = 1e-3, 2.0, 0.0
        lam = alpha ** 2 * (nx + kappa) - nx
        denominator = nx + lam
        weights_mean = np.full(2 * nx + 1, 1.0 / (2.0 * denominator))
        weights_covariance = weights_mean.copy()
        weights_mean[0] = lam / denominator
        weights_covariance[0] = weights_mean[0] + (1.0 - alpha ** 2 + beta)

        previous_mean = state.x.copy()
        previous_covariance = state.P.copy()
        first_measurement = state.last_timestamp_ns is None
        if not first_measurement:
            dt = max((int(observation_frame.timestamp_ns) - state.last_timestamp_ns) / 1e9, 1e-4)
            state.set_dt(dt)
        sigma_previous = _sigma_points(previous_mean, previous_covariance, denominator)
        sigma_predicted = (state.A @ sigma_previous.T).T if not first_measurement else sigma_previous
        x_pred = weights_mean @ sigma_predicted
        p_pred = state.Q.copy() if not first_measurement else np.zeros_like(state.Q)
        for index in range(2 * nx + 1):
            delta = sigma_predicted[index] - x_pred
            p_pred += weights_covariance[index] * np.outer(delta, delta)
        state.last_timestamp_ns = int(observation_frame.timestamp_ns)

        sigma_ranges = np.asarray([state._h(point, anchors) for point in sigma_predicted])
        z_pred = weights_mean @ sigma_ranges
        p_zz = _measurement_covariance(sigmas)
        p_xz = np.zeros((nx, ranges.size), dtype=float)
        for index in range(2 * nx + 1):
            delta_z = sigma_ranges[index] - z_pred
            delta_x = sigma_predicted[index] - x_pred
            p_zz += weights_covariance[index] * np.outer(delta_z, delta_z)
            p_xz += weights_covariance[index] * np.outer(delta_x, delta_z)
        gain = np.linalg.solve(p_zz, p_xz.T).T
        state.x = x_pred + gain @ (ranges - z_pred)
        state.P = p_pred - gain @ p_zz @ gain.T
        state.P = 0.5 * (state.P + state.P.T)
        estimate = state.x[:3].copy()
        covariance = state.P[:3, :3].copy()
        metadata = {"dimension": "3D", "timestamp_ns": observation_frame.timestamp_ns}
        valid = bool(np.all(np.isfinite(estimate)))
    except (ValueError, np.linalg.LinAlgError, FloatingPointError) as exc:
        estimate = np.full(3, np.nan)
        covariance = np.full((3, 3), np.nan)
        metadata = {"error": str(exc), "timestamp_ns": observation_frame.timestamp_ns}
        valid = False
    metadata["runtime_ms"] = (time.monotonic_ns() - start_ns) / 1e6
    return AlgorithmResult(
        estimate=np.asarray(estimate), covariance=np.asarray(covariance),
        algorithm="uwb.matlab.ukf", family="uwb", valid=valid, metadata=metadata,
    )
