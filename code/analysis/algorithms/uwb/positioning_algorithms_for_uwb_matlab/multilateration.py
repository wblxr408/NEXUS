"""Weighted least-squares multilateration adapted from the MATLAB project.

Source: https://github.com/cliansang/positioning-algorithms-for-uwb-matlab
Port of ``performMultilateration.m``.  The active runner uses the source
project's 3D branch with four or more non-coplanar anchors.
"""

from __future__ import annotations

import numpy as np

from algorithms.contracts import AlgorithmResult


def solve_multilateration_2d(anchor_positions, ranges, weights=None):
    anchors = np.asarray(anchor_positions, dtype=float)
    distances = np.asarray(ranges, dtype=float)
    n = anchors.shape[0]
    if n < 3 or n != distances.size:
        raise ValueError("at least three matching anchor-range pairs are required")

    anchors_2d = anchors[:, :2]
    first = anchors_2d[0]
    deltas = anchors_2d[1:] - first
    b_const = np.sum(anchors_2d ** 2, axis=1)
    # When anchors share a common height, the 3D range differs from the
    # projected 2D range by a constant offset that the linear system absorbs
    # through the range-difference formulation.  The sign convention follows
    # performMultilateration.m: b_i = (r_1^2 - r_i^2 + ||A_i||^2 - ||A_1||^2)/2.
    b = distances[0] ** 2 - distances[1:] ** 2 + b_const[1:] - b_const[0]
    b = b / 2.0

    if weights is None:
        weights = np.ones(n - 1)
    else:
        weights = np.asarray(weights, dtype=float)
        if weights.size != n - 1:
            weights = weights[1:]
    W = np.diag(np.asarray(weights, dtype=float))
    C = W.T @ W
    solution = np.linalg.solve(deltas.T @ C @ deltas, deltas.T @ C @ b)
    return solution


def solve_multilateration_3d(anchor_positions, ranges, weights=None):
    """Return the source-style weighted 3D linearized solution."""
    anchors = np.asarray(anchor_positions, dtype=float)
    distances = np.asarray(ranges, dtype=float).reshape(-1)
    if anchors.ndim != 2 or anchors.shape[1] < 3:
        raise ValueError("anchor_positions must have shape (N, >=3)")
    n = anchors.shape[0]
    if n < 4 or n != distances.size:
        raise ValueError("at least four matching anchor-range pairs are required")
    if not np.all(np.isfinite(anchors[:, :3])):
        raise ValueError("anchor positions must be finite")
    if np.any(distances <= 0.0) or not np.all(np.isfinite(distances)):
        raise ValueError("ranges must be positive and finite")

    anchors = anchors[:, :3]
    deltas = anchors[1:] - anchors[0]
    if np.linalg.matrix_rank(deltas) < 3:
        raise ValueError("3D multilateration anchors are coplanar or rank deficient")
    b = (
        distances[0] ** 2
        - distances[1:] ** 2
        + np.sum(anchors[1:] ** 2, axis=1)
        - np.sum(anchors[0] ** 2)
    ) / 2.0

    if weights is None:
        difference_weights = np.ones(n - 1, dtype=float)
    else:
        supplied = np.asarray(weights, dtype=float).reshape(-1)
        if supplied.size == n:
            supplied = supplied[1:]
        if supplied.size != n - 1 or np.any(supplied <= 0.0) or not np.all(np.isfinite(supplied)):
            raise ValueError("weights must contain one positive finite value per range difference")
        difference_weights = supplied
    normal = deltas.T @ (difference_weights[:, None] ** 2 * deltas)
    estimate = np.linalg.solve(
        normal,
        deltas.T @ (difference_weights ** 2 * b),
    )
    if not np.all(np.isfinite(estimate)):
        raise ValueError("3D multilateration failed")
    return estimate


def run_multilateration(*, observation_frame, **_):
    import time as time_module

    ranges = [sample.range_m for sample in observation_frame.ranges]
    positions = [sample.anchor_position_m for sample in observation_frame.ranges]
    stddevs = [sample.stddev_m for sample in observation_frame.ranges]
    start_ns = time_module.monotonic_ns()
    try:
        inverse_variance = [
            1.0 / max(sigma, 1e-12) if sigma > 0 else 1.0
            for sigma in stddevs
        ]
        xyz = solve_multilateration_3d(positions, ranges, weights=inverse_variance)
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
        algorithm="uwb.matlab.multilateration",
        family="uwb", valid=valid, metadata=metadata,
    )
