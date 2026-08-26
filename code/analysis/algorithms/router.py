"""Algorithm registry and routing boundary.

The future dashboard can pass the same algorithm name to a ROS2 service or to
the offline runner.  It must never select an output file from another run.
"""

from dataclasses import dataclass
from typing import Any, Callable

from .contracts import AlgorithmResult


@dataclass(frozen=True)
class AlgorithmSpec:
    name: str
    family: str
    source: str
    available: bool
    runner: Callable[..., AlgorithmResult] | None = None


_REGISTRY = {}


def register_algorithm(name, family, runner, source="project"):
    """Register an implemented algorithm runner."""
    if not name or "." not in name:
        raise ValueError("algorithm name must use '<family>.<name>'")
    if not callable(runner):
        raise TypeError("runner must be callable")
    _REGISTRY[name] = AlgorithmSpec(name, family, source, True, runner)


def register_algorithm_slot(name, family, source):
    """Reserve a route before its concrete implementation is written."""
    _REGISTRY[name] = AlgorithmSpec(name, family, source, False, None)


for _name, _family, _source in (
    ("vision.pvnet", "vision", "docs:markerless_visual_simulation_preparation"),
    ("vision.gdr_net", "vision", "docs:markerless_visual_simulation_preparation"),
    ("vision.foundationpose", "vision", "docs:markerless_visual_simulation_preparation"),
    ("vision.fixed_map_feature_pnp", "vision", "docs:markerless_visual_simulation_preparation"),
    ("uwb.awesome_uwb", "uwb", "https://github.com/qxiaofan/awesome-uwb-localization"),
    ("uwb.matlab_positioning", "uwb", "https://github.com/cliansang/positioning-algorithms-for-uwb-matlab"),
):
    register_algorithm_slot(_name, _family, _source)


def register_default_algorithms():
    """Register concrete runners that ship with the project."""
    from algorithms.uwb.positioning_algorithms_for_uwb_matlab.trilateration import (
        run_trilateration,
    )
    from algorithms.uwb.positioning_algorithms_for_uwb_matlab.multilateration import (
        run_multilateration,
    )
    from algorithms.uwb.positioning_algorithms_for_uwb_matlab.taylor_series import (
        run_taylor,
    )
    from algorithms.uwb.positioning_algorithms_for_uwb_matlab.kalman_filters import (
        run_ekf,
        run_ukf,
    )

    register_algorithm(
        "uwb.matlab.trilateration", "uwb", run_trilateration,
        source="https://github.com/cliansang/positioning-algorithms-for-uwb-matlab",
    )
    register_algorithm(
        "uwb.matlab.multilateration", "uwb", run_multilateration,
        source="https://github.com/cliansang/positioning-algorithms-for-uwb-matlab",
    )
    register_algorithm(
        "uwb.matlab.taylor", "uwb", run_taylor,
        source="https://github.com/cliansang/positioning-algorithms-for-uwb-matlab",
    )
    register_algorithm(
        "uwb.matlab.ekf", "uwb", run_ekf,
        source="https://github.com/cliansang/positioning-algorithms-for-uwb-matlab",
    )
    register_algorithm(
        "uwb.matlab.ukf", "uwb", run_ukf,
        source="https://github.com/cliansang/positioning-algorithms-for-uwb-matlab",
    )


def available_algorithms(family=None):
    return [spec for spec in _REGISTRY.values() if (family is None or spec.family == family)]


class AlgorithmRouter:
    def __init__(self, algorithm: str):
        if algorithm not in _REGISTRY:
            choices = ", ".join(sorted(_REGISTRY))
            raise ValueError(f"unknown algorithm '{algorithm}'. choices: {choices}")
        self.spec = _REGISTRY[algorithm]
        if not self.spec.available or self.spec.runner is None:
            raise RuntimeError(
                f"algorithm '{algorithm}' is registered but unavailable; "
                "register its concrete runner before running simulation")

    def run(self, **inputs: Any) -> AlgorithmResult:
        result = self.spec.runner(**inputs)
        if not isinstance(result, AlgorithmResult):
            raise TypeError(f"algorithm '{self.spec.name}' returned an invalid result")
        return result
