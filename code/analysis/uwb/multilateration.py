"""Small least-squares multilateration runner for synthetic UWB ranges."""

from __future__ import annotations

from typing import Any

import numpy as np


def solve_multilateration(anchors_m: Any, ranges_m: Any) -> np.ndarray:
    anchors = np.asarray(anchors_m, dtype=np.float64)
    ranges = np.asarray(ranges_m, dtype=np.float64).reshape(-1)
    if anchors.ndim != 2 or anchors.shape[1] != 3 or anchors.shape[0] < 4:
        raise ValueError("anchors_m must have shape Nx3 with N >= 4")
    if ranges.shape != (anchors.shape[0],) or not np.all(np.isfinite(anchors)) or not np.all(np.isfinite(ranges)):
        raise ValueError("anchors and ranges must be finite and have matching lengths")
    if np.any(ranges <= 0):
        raise ValueError("ranges_m must be positive")
    reference = anchors[0]
    matrix = 2.0 * (anchors[1:] - reference)
    rhs = ranges[0] ** 2 - ranges[1:] ** 2 + np.sum(anchors[1:] ** 2, axis=1) - np.sum(reference ** 2)
    if np.linalg.matrix_rank(matrix) < 3:
        raise ValueError("anchor geometry is rank deficient")
    estimate, _, _, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
    # Refine the linear solution against the original sphere equations. This
    # keeps the runner dependency-free while removing the first-order bias of
    # the linearized solve under noisy ranges.
    for _ in range(20):
        deltas = estimate[None, :] - anchors
        distances = np.linalg.norm(deltas, axis=1)
        if np.any(distances <= 1e-12):
            break
        residual = distances - ranges
        jacobian = deltas / distances[:, None]
        step, _, _, _ = np.linalg.lstsq(jacobian, -residual, rcond=None)
        estimate = estimate + step
        if np.linalg.norm(step) < 1e-10:
            break
    if not np.all(np.isfinite(estimate)):
        raise ValueError("multilateration failed")
    return estimate


def run_multilateration(**inputs: Any) -> AlgorithmResult:
    from algorithms.contracts import AlgorithmResult
    estimate = solve_multilateration(inputs["anchors_m"], inputs["ranges_m"])
    covariance = np.eye(3, dtype=np.float64) * float(inputs.get("covariance_diag", 1.0))
    return AlgorithmResult(
        estimate=estimate,
        covariance=covariance,
        algorithm="uwb.multilateration",
        family="uwb",
        metadata={"solver": "linearized_least_squares", "unit": "m", "anchor_count": len(inputs["anchors_m"])},
    )


__all__ = ["solve_multilateration", "run_multilateration"]
