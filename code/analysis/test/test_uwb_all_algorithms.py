import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent.parent / "simulation"))

from algorithms.router import AlgorithmRouter, register_default_algorithms
from generators.range_simulation import Anchor, simulate_ranges

ANCHORS = [
    Anchor("A1", (0.30, 0.30, 0.50)),
    Anchor("A2", (3.70, 0.30, 4.50)),
    Anchor("A3", (3.70, 4.40, 0.50)),
    Anchor("A4", (0.30, 4.40, 4.50)),
]

ALGORITHMS = [
    "uwb.matlab.trilateration",
    "uwb.matlab.multilateration",
    "uwb.matlab.taylor",
    "uwb.matlab.ekf",
    "uwb.matlab.ukf",
    "uwb.awesome_uwb",
]


def _frames(positions, stamps):
    return simulate_ranges(
        ANCHORS, positions, stamps, sigma_m=0.0, random_seed=42, dimensions=3
    )


def test_static_point_all_algorithms():
    register_default_algorithms()
    gt = [2.00, 2.35, 1.20]
    frames = _frames([gt], [1000])
    for name in ALGORITHMS:
        if name in ("uwb.matlab.ekf", "uwb.matlab.ukf"):
            runner = AlgorithmRouter(name).spec.runner
            if hasattr(runner, "_state"):
                del runner._state
        # Filters are stateful; initialize this one-frame check at the known
        # test pose so it checks the 3D measurement update rather than prior
        # covariance tuning.
        result = AlgorithmRouter(name).run(
            observation_frame=frames[0], initial_xyz=gt
        )
        assert result.valid, f"{name} failed: {result.metadata}"
        error = np.linalg.norm(result.estimate - np.array(gt))
        threshold = 1e-4 if name in ("uwb.matlab.ekf", "uwb.matlab.ukf") else 1e-6
        assert error < threshold, f"{name} static error={error}"


def test_trajectory_all_algorithms():
    register_default_algorithms()
    t = np.linspace(0.0, 1.0, 50)
    positions = np.column_stack([
        0.5 + 3.0 * t,
        0.5 + 3.5 * t,
        np.full_like(t, 1.20),
    ])
    stamps = (t * int(1e9)).astype(np.int64)
    frames = _frames(positions, stamps)

    for name in ALGORITHMS:
        runner = AlgorithmRouter(name).spec.runner
        if hasattr(runner, "_state"):
            del runner._state
        errors = []
        for index, frame in enumerate(frames):
            result = AlgorithmRouter(name).run(observation_frame=frame)
            assert result.valid, f"{name} frame {index} failed: {result.metadata}"
            errors.append(np.linalg.norm(result.estimate - positions[index, :3]))
        rmse = float(np.sqrt(np.mean(np.array(errors) ** 2)))
        filter_thresholds = {
            "uwb.matlab.taylor": 0.2,
            "uwb.matlab.ekf": 0.15,
            "uwb.matlab.ukf": 0.25,
            "uwb.awesome_uwb": 0.30,
        }
        threshold = filter_thresholds.get(name, 1e-5)
        assert rmse < threshold, f"{name} trajectory RMSE={rmse} (threshold={threshold})"
