"""Least-squares trilateration adapted from the MATLAB UWB positioning project.

Source: https://github.com/cliansang/positioning-algorithms-for-uwb-matlab
The original implementation solves a 2D position from three or more ranges.
This port keeps the same linear least-squares formulation but accepts 3D
anchor coordinates and reports the horizontal (XY) estimate explicitly as
a 2D result; it does not fabricate a z value.
"""

from __future__ import annotations

import numpy as np

from algorithms.contracts import AlgorithmResult


def solve_trilateration_2d(anchor_positions, ranges):
    """Return the least-squares XY estimate from >=3 range measurements."""
    anchors = np.asarray(anchor_positions, dtype=float)
    distances = np.asarray(ranges, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] < 2:
        raise ValueError("anchor_positions must have shape (N, >=2)")
    if anchors.shape[0] != distances.size:
        raise ValueError("number of anchors must match number of ranges")
    if anchors.shape[0] < 3:
        raise ValueError("at least three anchors are required for 2D trilateration")
    if np.any(distances <= 0.0) or not np.all(np.isfinite(distances)):
        raise ValueError("ranges must be positive and finite")

    first = anchors[0]
    rows = []
    rhs = []
    d0_sq = distances[0] ** 2
    # Take only the first two columns so that 3D anchor heights are projected
    # into the horizontal plane, consistent with the original 2D solver.
    deltas = anchors[1:] - first
    for row in deltas:
        rows.append(row[:2])
        rhs.append(0.5 * (d0_sq - distances[len(rows)] ** 2 + np.dot(row[:2], row[:2])))

    matrix = np.asarray(rows)
    vector = np.asarray(rhs)
    solution, _, _, _ = np.linalg.lstsq(matrix, vector, rcond=None)
    return np.asarray(first[:2], dtype=float) + solution


def run_trilateration(*, observation_frame, **_):
    """Algorithm runner registered with the NEXUS router."""
    ranges = [sample.range_m for sample in observation_frame.ranges]
    positions = [sample.anchor_position_m for sample in observation_frame.ranges]
    start_ns = __import__("time").monotonic_ns()
    try:
        xy = solve_trilateration_2d(positions, ranges)
        valid = True
        metadata = {"dimension": "2D", "timestamp_ns": observation_frame.timestamp_ns}
    except (ValueError, np.linalg.LinAlgError) as exc:
        xy = np.full(2, np.nan)
        valid = False
        metadata = {"error": str(exc), "timestamp_ns": observation_frame.timestamp_ns}
    runtime_ms = (__import__("time").monotonic_ns() - start_ns) / 1e6
    metadata["runtime_ms"] = runtime_ms

    covariance = np.diag([np.nan, np.nan])
    return AlgorithmResult(
        estimate=np.asarray(xy),
        covariance=covariance,
        algorithm="uwb.matlab.trilateration",
        family="uwb",
        valid=valid,
        metadata=metadata,
    )
