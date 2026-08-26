import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[1]
for directory in ("", "evaluation", "uwb"):
    sys.path.insert(0, str(ROOT / directory) if directory else str(ROOT))
sys.path.insert(0, str(ROOT.parent.parent / "simulation"))

from algorithms.router import AlgorithmRouter, register_default_algorithms
from evaluation.metrics import position_metrics_2d
from generators.range_simulation import Anchor, simulate_ranges
from algorithms.uwb.positioning_algorithms_for_uwb_matlab.trilateration import (
    run_trilateration,
    solve_trilateration_2d,
)


ANCHORS = [
    Anchor("A1", (0.30, 0.30, 2.20)),
    Anchor("A2", (3.70, 0.30, 2.20)),
    Anchor("A3", (3.70, 4.40, 2.20)),
    Anchor("A4", (0.30, 4.40, 2.20)),
]


def _make_frames(positions, stamps):
    return simulate_ranges(ANCHORS, positions, stamps, sigma_m=0.0, random_seed=42)


def test_static_point_sigma_zero_recovers_position():
    position = [[2.00, 2.35, 1.20]]
    frames = _make_frames(position, [1000])
    result = run_trilateration(observation_frame=frames[0])
    assert result.valid
    assert np.linalg.norm(result.estimate[:2] - np.array([2.00, 2.35])) < 1e-9


def test_trajectory_sigma_zero_rmse_near_zero():
    t = np.linspace(0.0, 1.0, 50)
    positions = np.column_stack([
        0.5 + 3.0 * t,
        0.5 + 3.5 * t,
        np.full_like(t, 1.20),
    ])
    stamps = (t * int(1e9)).astype(np.int64)
    frames = _make_frames(positions, stamps)
    estimates = []
    references = []
    for frame in frames:
        result = run_trilateration(observation_frame=frame)
        assert result.valid
        estimates.append(result.estimate)
        references.append(positions[len(estimates) - 1, :2])
    metrics = position_metrics_2d(estimates, references)
    assert metrics["dimension"] == "2D"
    assert metrics["rmse_m"] < 1e-8
    assert metrics["mae_m"] < 1e-8
    assert metrics["cep95_m"] < 1e-8


def test_router_selects_registered_algorithm():
    register_default_algorithms()
    router = AlgorithmRouter("uwb.matlab.trilateration")
    frames = _make_frames([[2.0, 2.35, 1.20]], [2000])
    result = router.run(observation_frame=frames[0])
    assert result.algorithm == "uwb.matlab.trilateration"
    assert result.valid


def test_metrics_2d_rejects_mismatched_shapes():
    with __import__("pytest").raises(ValueError):
        position_metrics_2d([[0, 0]], [[0, 0], [1, 1]])
