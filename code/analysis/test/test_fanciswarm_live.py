import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent.parent / "simulation"))

from algorithms.uwb.live_protocol import (convert_fc_position, horizontal_error,
                                           observation_from_battery, parse_battery_distances)
from algorithms.uwb.run_fanciswarm_live import ALGORITHMS, LiveAlgorithmRunner


def test_fanciswarm_battery_ranges_and_frame_contract():
    assert parse_battery_distances([0, 0, 341, 354, 488, 443]) == [3.41, 3.54, 4.88, 4.43]
    frame = observation_from_battery([0, 0, 341, 354, 488, 443], 123)
    assert frame.tag_id == "2"
    assert [sample.anchor_id for sample in frame.ranges] == ["1", "2", "3", "4"]
    assert all(sample.stddev_m == 0.05 for sample in frame.ranges)
    with pytest.raises(ValueError): parse_battery_distances([0, 0, 0, 354, 488, 443])
    with pytest.raises(ValueError): parse_battery_distances([0, 0, 341, 65535, 488, 443])


def test_live_routes_are_xy_only_and_stateful():
    frame = observation_from_battery([0, 0, 341, 354, 488, 443], 1)
    runner = LiveAlgorithmRunner()
    results = runner.run_all(frame, (0.5, 2.0, 0.0))
    assert set(results) == set(ALGORITHMS)
    assert all(result.valid and result.estimate.shape == (2,) for result in results.values())
    assert all(result.metadata["dimension"] == "2D/XY-only" for result in results.values())
    assert results["uwb.matlab.ekf"].metadata["state"] == "[x,y,vx,vy]"
    assert "error_vs_fc_m" in results["uwb.matlab.trilateration"].metadata

    bad_order = frame.__class__(frame.timestamp_ns + 1, frame.tag_id, list(reversed(frame.ranges)))
    failed = runner.run_all(bad_order)
    assert all(not result.valid for result in failed.values())
    recovered = runner.run_all(observation_from_battery([0, 0, 341, 354, 488, 443], 3))
    assert all(result.valid for result in recovered.values())


def test_fc_conversion_is_comparison_only():
    msg = type("Msg", (), {"x": 100, "y": -250, "z": 300})()
    position = convert_fc_position(msg)
    assert position == (1.0, -2.5, -3.0)
    assert horizontal_error((1.3, -2.1), position) == pytest.approx(0.5)
