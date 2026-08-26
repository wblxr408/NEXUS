import sys
from pathlib import Path

import pytest
import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))

from algorithms import AlgorithmRouter, available_algorithms, register_algorithm
from algorithms.contracts import AlgorithmResult


def test_router_requires_concrete_implementation():
    with pytest.raises(RuntimeError, match="unavailable"):
        AlgorithmRouter("uwb.awesome_uwb")


def test_router_dispatches_registered_runner():
    def runner(**inputs):
        assert inputs == {"sample": 1}
        return AlgorithmResult(np.zeros(3), np.eye(3), "test.example", "test")

    register_algorithm("test.example", "test", runner)
    result = AlgorithmRouter("test.example").run(sample=1)
    assert result.algorithm == "test.example"


def test_external_algorithm_is_registered_but_not_falsely_available():
    names = {item.name: item for item in available_algorithms()}
    assert names["uwb.awesome_uwb"].available is False
