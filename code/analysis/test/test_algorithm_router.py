import sys
from pathlib import Path

import pytest
import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))

from algorithms import AlgorithmRouter, available_algorithms, register_algorithm
from algorithms.contracts import AlgorithmResult


def test_router_exposes_awesome_uwb_after_default_registration():
    from algorithms.router import register_default_algorithms

    register_default_algorithms()
    names = {item.name: item for item in available_algorithms()}
    assert names["uwb.awesome_uwb"].available is True


def test_router_dispatches_registered_runner():
    def runner(**inputs):
        assert inputs == {"sample": 1}
        return AlgorithmResult(np.zeros(3), np.eye(3), "test.example", "test")

    register_algorithm("test.example", "test", runner)
    result = AlgorithmRouter("test.example").run(sample=1)
    assert result.algorithm == "test.example"


def test_unported_matlab_aggregate_slot_remains_unavailable():
    names = {item.name: item for item in available_algorithms()}
    assert names["uwb.matlab_positioning"].available is False


def test_gdr_net_dense_correspondence_runner_recovers_metric_translation():
    height, width = 16, 20
    camera = np.array([[400.0, 0.0, 9.5], [0.0, 400.0, 7.5], [0.0, 0.0, 1.0]])
    grid_y, grid_x = np.indices((height, width), dtype=float)
    depth = 1.0 + 0.05 * np.sin(grid_x) * np.cos(grid_y)
    coordinates = np.stack(
        ((grid_x - 9.5) * depth / 400.0, (grid_y - 7.5) * depth / 400.0, depth), axis=-1
    )
    result = AlgorithmRouter("vision.gdr_net").run(
        coordinates=coordinates,
        visibility=np.ones((height, width)),
        camera_matrix=camera,
        max_points=128,
    )
    assert result.valid
    assert result.algorithm == "vision.gdr_net"
    assert np.allclose(result.estimate[:, :3], np.eye(3), atol=1e-5)
    assert np.allclose(result.estimate[:, 3], [0.0, 0.0, 0.0], atol=1e-5)
    assert result.metadata["correspondence_count"] == 128


def test_uwb_multilateration_runner_recovers_position_without_noise():
    anchors = np.array([[0.0, 0.0, 2.0], [4.0, 0.0, 2.4], [4.0, 4.0, 2.8], [0.0, 4.0, 2.2]])
    target = np.array([1.2, 1.7, 2.8])
    ranges = np.linalg.norm(anchors - target, axis=1)
    result = AlgorithmRouter("uwb.multilateration").run(anchors_m=anchors, ranges_m=ranges)
    assert np.allclose(result.estimate, target, atol=1e-9)
