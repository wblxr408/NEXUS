"""Iterative Taylor-series position solver with optional KF smoothing.

Source: https://github.com/cliansang/positioning-algorithms-for-uwb-matlab
Port of demo_Taylor_series_algo.m (without the KF smoothing stage; the raw
iterative solution is returned so the algorithm output is deterministic).
"""

from __future__ import annotations

import numpy as np

from algorithms.contracts import AlgorithmResult


def solve_taylor_2d(anchor_positions, ranges, initial_xy=(0.0, 0.0), max_iter=20, tol=1e-12):
    anchors = np.asarray(anchor_positions, dtype=float)
    distances = np.asarray(ranges, dtype=float)
    n = anchors.shape[0]
    if n < 3 or n != distances.size:
        raise ValueError("at least three matching anchor-range pairs are required")

    xy = np.array(initial_xy, dtype=float)
    for _ in range(max_iter):
        diff = anchors[:, :2] - xy
        ri = np.linalg.norm(diff, axis=1)
        H = -diff / ri[:, None]
        delta_r = distances - ri
        try:
            delta_xy = np.linalg.solve(H.T @ H, H.T @ delta_r)
        except np.linalg.LinAlgError:
            raise ValueError("singular Jacobian in Taylor iteration")
        xy += delta_xy
        if np.linalg.norm(delta_xy) < tol:
            break
    return xy


def run_taylor(*, observation_frame, initial_xy=(2.0, 2.35), **_):
    import time as time_module

    ranges = [sample.range_m for sample in observation_frame.ranges]
    positions = [sample.anchor_position_m for sample in observation_frame.ranges]
    start_ns = time_module.monotonic_ns()
    try:
        xy = solve_taylor_2d(positions, ranges, initial_xy=initial_xy)
        valid = True
        metadata = {"dimension": "2D", "timestamp_ns": observation_frame.timestamp_ns}
    except ValueError as exc:
        xy = np.full(2, np.nan)
        valid = False
        metadata = {"error": str(exc), "timestamp_ns": observation_frame.timestamp_ns}
    runtime_ms = (time_module.monotonic_ns() - start_ns) / 1e6
    metadata["runtime_ms"] = runtime_ms
    return AlgorithmResult(
        estimate=np.asarray(xy),
        covariance=np.diag([np.nan, np.nan]),
        algorithm="uwb.matlab.taylor", family="uwb",
        valid=valid, metadata=metadata,
    )
