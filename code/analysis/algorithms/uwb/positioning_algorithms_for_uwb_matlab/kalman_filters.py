"""EKF and UKF UWB positioning with constant-velocity motion model.

Source: https://github.com/cliansang/positioning-algorithms-for-uwb-matlab
State vector is [x, y, vx, vy]; measurement model predicts ranges to anchors.
Both filters maintain state across frames when called sequentially.
"""

from __future__ import annotations

import numpy as np

from algorithms.contracts import AlgorithmResult


DT = 0.1


class RangeFilterBase:
    def __init__(self, initial_xy, dt=DT):
        self.dt = dt
        self.x = np.array([initial_xy[0], initial_xy[1], 0.0, 0.0])
        self.P = np.eye(4) * 2.0
        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=float)
        self.Q = np.diag([dt**4/4, dt**4/4, dt**2/2, dt**2/2])
        self.R_scale = 0.123

    @staticmethod
    def _h(x_state, anchors):
        diff = x_state[:2] - anchors[:, :2]
        return np.linalg.norm(diff, axis=1)

    @staticmethod
    def _H_jacobian(x_state, anchors):
        diff = x_state[:2] - anchors[:, :2]
        norms = np.linalg.norm(diff, axis=1)
        return np.hstack([
            diff / norms[:, None], np.zeros((len(anchors), 2))
        ])


def run_ekf(*, observation_frame, initial_xy=(2.0, 2.35), **_):
    """One-step EKF; caller must reuse the returned filter for sequential data."""
    import time as time_module

    ranges = np.array([s.range_m for s in observation_frame.ranges])
    anchors = np.array([s.anchor_position_m for s in observation_frame.ranges])
    n = len(ranges)
    R = np.eye(n) * 0.123
    Q = np.diag([DT**4/4, DT**4/4, DT**2/2, DT**2/2])

    start_ns = time_module.monotonic_ns()
    try:
        ekf = getattr(run_ekf, "_state", None)
        if ekf is None:
            ekf = RangeFilterBase(initial_xy)
            run_ekf._state = ekf

        x_pred = ekf.A @ ekf.x
        P_pred = ekf.A @ ekf.P @ ekf.A.T + Q
        H = RangeFilterBase._H_jacobian(x_pred, anchors)
        z_pred = RangeFilterBase._h(x_pred, anchors)
        innovation = ranges - z_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        ekf.x = x_pred + K @ innovation
        ekf.P = (np.eye(4) - K @ H) @ P_pred

        xy = ekf.x[:2].copy()
        valid = True
        metadata = {"dimension": "2D", "timestamp_ns": observation_frame.timestamp_ns}
    except (ValueError, np.linalg.LinAlgError) as exc:
        xy = np.full(2, np.nan)
        valid = False
        metadata = {"error": str(exc)}
    runtime_ms = (time_module.monotonic_ns() - start_ns) / 1e6
    metadata["runtime_ms"] = runtime_ms
    return AlgorithmResult(
        estimate=np.asarray(xy), covariance=np.diag([np.nan, np.nan]),
        algorithm="uwb.matlab.ekf", family="uwb", valid=valid, metadata=metadata,
    )


def run_ukf(*, observation_frame, initial_xy=(2.0, 2.35), **_):
    """One-step UKF; same sequential-state caveat as EKF."""
    import time as time_module

    ranges = np.array([s.range_m for s in observation_frame.ranges])
    anchors = np.array([s.anchor_position_m for s in observation_frame.ranges])
    n = len(ranges)
    R = np.eye(n) * 0.123
    Q = np.diag([DT**4/4, DT**4/4, DT**2/2, DT**2/2])

    start_ns = time_module.monotonic_ns()
    try:
        ukf = getattr(run_ukf, "_state", None)
        if ukf is None:
            ukf = RangeFilterBase(initial_xy)
            run_ukf._state = ukf

        nx = 4
        alpha, beta, kappa = 1e-3, 2.0, 0.0
        lam = alpha ** 2 * (nx + kappa) - nx
        Wm = np.full(2 * nx + 1, 1.0 / (2 * (nx + lam)))
        Wc = Wm.copy()
        Wm[0] = lam / (nx + lam)
        Wc[0] = Wm[0] + (1 - alpha**2 + beta)

        sqrt_P = np.linalg.cholesky((nx + lam) * ukf.P)
        sigma_pts = np.zeros((2 * nx + 1, nx))
        sigma_pts[0] = ukf.x
        for i in range(nx):
            sigma_pts[1 + i] = ukf.x + sqrt_P[i]
            sigma_pts[1 + nx + i] = ukf.x - sqrt_P[i]

        sig_pred = (ukf.A @ sigma_pts.T).T
        x_pred = Wm @ sig_pred
        P_pred = Q.copy()
        for i in range(2 * nx + 1):
            d = sig_pred[i] - x_pred
            P_pred += Wc[i] * np.outer(d, d)

        sig_ranges = np.array([RangeFilterBase._h(s, anchors) for s in sig_pred])
        z_pred = Wm @ sig_ranges
        Pzz = R.copy()
        Pxz = np.zeros((nx, n))
        for i in range(2 * nx + 1):
            dz = sig_ranges[i] - z_pred
            dx = sig_pred[i] - x_pred
            Pzz += Wc[i] * np.outer(dz, dz)
            Pxz += Wc[i] * np.outer(dx, dz)

        K = Pxz @ np.linalg.inv(Pzz)
        innovation = ranges - z_pred
        ukf.x = x_pred + K @ innovation
        ukf.P = P_pred - K @ Pzz @ K.T

        xy = ukf.x[:2].copy()
        valid = True
        metadata = {"dimension": "2D", "timestamp_ns": observation_frame.timestamp_ns}
    except (ValueError, np.linalg.LinAlgError) as exc:
        xy = np.full(2, np.nan)
        valid = False
        metadata = {"error": str(exc)}
    runtime_ms = (time_module.monotonic_ns() - start_ns) / 1e6
    metadata["runtime_ms"] = runtime_ms
    return AlgorithmResult(
        estimate=np.asarray(xy), covariance=np.diag([np.nan, np.nan]),
        algorithm="uwb.matlab.ukf", family="uwb", valid=valid, metadata=metadata,
    )
