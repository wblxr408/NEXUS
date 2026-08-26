import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).parents[5]
sys.path.insert(0, str(REPO / "code" / "analysis"))
sys.path.insert(0, str(REPO / "simulation"))
sys.path.insert(0, str(REPO / "code" / "ros2_ws" / "src" / "nexus_uwb_simulation" / "src"))

from generators.range_simulation import Anchor, simulate_ranges
from algorithms.router import AlgorithmRouter, register_default_algorithms
from algorithms.contracts import AlgorithmResult
from nexus_uwb_simulation.observation_frame import ObservationFrame as _ObservationFrame


ANCHORS = [
    Anchor("A1", (0.30, 0.30, 2.20)),
    Anchor("A2", (3.70, 0.30, 2.20)),
    Anchor("A3", (3.70, 4.40, 2.20)),
    Anchor("A4", (0.30, 4.40, 2.20)),
]


def test_gt_to_ranges_sigma_zero():
    gt = [2.00, 2.35, 1.20]
    frames = simulate_ranges(ANCHORS, [gt], [1000], sigma_m=0.0)
    for sample in frames[0].ranges:
        expected = float(np.linalg.norm(np.array(gt) - np.array(sample.anchor_position_m)))
        assert abs(sample.range_m - expected) < 1e-12


def test_full_chain_no_noise():
    register_default_algorithms()
    gt = [2.00, 2.35, 1.20]
    frames = simulate_ranges(ANCHORS, [gt], [1000], sigma_m=0.0)
    router = AlgorithmRouter("uwb.matlab.trilateration")
    result = router.run(observation_frame=frames[0])
    assert result.valid
    assert np.linalg.norm(result.estimate - np.array(gt[:2])) < 1e-9


def test_ground_truth_isolation():
    frames = simulate_ranges(ANCHORS, [[1.0, 1.0, 1.0]], [2000], sigma_m=0.0)
    frame_dict = vars(frames[0])
    for key in ("ground_truth", "gt", "gt_position", "position"):
        assert key not in frame_dict, f"GT leaked via '{key}'"
    assert all(not hasattr(s, "gt") for s in frames[0].ranges)


def test_observation_frame_conversion():
    ranges = [2.84473197] * 4
    positions = [(0.30, 0.30, 2.20), (3.70, 0.30, 2.20), (3.70, 4.40, 2.20), (0.30, 4.40, 2.20)]
    frame = _ObservationFrame(ranges, positions)
    assert len(frame.ranges) == 4
    assert all(hasattr(s, "range_m") and hasattr(s, "anchor_position_m") for s in frame.ranges)
