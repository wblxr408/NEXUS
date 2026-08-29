"""Iterative Taylor-series position solver with optional KF smoothing.

Source: https://github.com/cliansang/positioning-algorithms-for-uwb-matlab
Port of demo_Taylor_series_algo.m (without the KF smoothing stage; the raw
iterative solution is returned so the algorithm output is deterministic).
The active runner uses the same incremental range Jacobian in 3D.
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


def solve_taylor_3d(anchor_positions, ranges, initial_xyz=(0.0, 0.0, 2.0), max_iter=30, tol=1e-10):
    """Iteratively solve the full 3D range equations."""
    anchors = np.asarray(anchor_positions, dtype=float)
    distances = np.asarray(ranges, dtype=float).reshape(-1)
    if anchors.ndim != 2 or anchors.shape[1] < 3:
        raise ValueError("anchor_positions must have shape (N, >=3)")
    if anchors.shape[0] < 4 or anchors.shape[0] != distances.size:
        raise ValueError("at least four matching anchor-range pairs are required")
    if not np.all(np.isfinite(anchors[:, :3])):
        raise ValueError("anchor positions must be finite")
    if np.any(distances <= 0.0) or not np.all(np.isfinite(distances)):
        raise ValueError("ranges must be positive and finite")
    xyz = np.asarray(initial_xyz, dtype=float).reshape(-1)
    if xyz.size != 3 or not np.all(np.isfinite(xyz)):
        raise ValueError("initial_xyz must contain three finite coordinates")
    anchors = anchors[:, :3]
    if np.linalg.matrix_rank(anchors[1:] - anchors[0]) < 3:
        raise ValueError("3D Taylor anchors are coplanar or rank deficient")

    for _ in range(max_iter):
        diff = anchors - xyz
        predicted = np.linalg.norm(diff, axis=1)
        if np.any(predicted <= 1e-12):
            raise ValueError("Taylor iteration reached an anchor")
        jacobian = -diff / predicted[:, None]
        delta = distances - predicted
        step, _, rank, _ = np.linalg.lstsq(jacobian, delta, rcond=None)
        if rank < 3:
            raise ValueError("singular Jacobian in 3D Taylor iteration")
        xyz += step
        if np.linalg.norm(step) < tol:
            break
    if not np.all(np.isfinite(xyz)):
        raise ValueError("3D Taylor iteration failed")
    return xyz


def run_taylor(*, observation_frame, initial_xyz=(0.0, 0.0, 2.0), initial_xy=None, **_):
    import time as time_module

    ranges = [sample.range_m for sample in observation_frame.ranges]
    positions = [sample.anchor_position_m for sample in observation_frame.ranges]
    start_ns = time_module.monotonic_ns()
    try:
        if initial_xy is not None:
            initial_xyz = (float(initial_xy[0]), float(initial_xy[1]), float(initial_xyz[2]))
        xyz = solve_taylor_3d(positions, ranges, initial_xyz=initial_xyz)
        valid = True
        metadata = {"dimension": "3D", "timestamp_ns": observation_frame.timestamp_ns}
    except (ValueError, np.linalg.LinAlgError) as exc:
        xyz = np.full(3, np.nan)
        valid = False
        metadata = {"error": str(exc), "timestamp_ns": observation_frame.timestamp_ns}
    runtime_ms = (time_module.monotonic_ns() - start_ns) / 1e6
    metadata["runtime_ms"] = runtime_ms
    return AlgorithmResult(
        estimate=np.asarray(xyz),
        covariance=np.diag([np.nan, np.nan, np.nan]),
        algorithm="uwb.matlab.taylor", family="uwb",
        valid=valid, metadata=metadata,
    )
