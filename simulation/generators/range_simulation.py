"""Generate simulated UWB range observations from ground-truth positions.

The caller supplies anchor positions and a tag position for each timestep;
this module only computes ranges and adds configurable noise.  It never
receives or returns the ground truth as part of the algorithm input.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Anchor:
    id: str
    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class RangeSample:
    anchor_id: str
    anchor_position_m: tuple[float, float, float]
    range_m: float
    stddev_m: float


@dataclass(frozen=True)
class UwbObservationFrame:
    """One observation epoch; this is the algorithm-facing input format."""

    timestamp_ns: int
    tag_id: str
    ranges: list[RangeSample]


def load_anchors(config_path):
    import yaml

    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    return [
        Anchor(entry["id"], tuple(float(value) for value in entry["position_m"]))
        for entry in config["anchors"]
    ]


def simulate_ranges(
    anchors,
    positions_m,
    timestamps_ns,
    *,
    sigma_m=0.0,
    random_seed=42,
    tag_id="target",
):
    """Return one UwbObservationFrame per timestamp with noisy anchor ranges."""
    positions = np.asarray(positions_m, dtype=float)
    stamps = np.asarray(timestamps_ns, dtype=np.int64)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("positions must have shape (N, 3)")
    if stamps.size != positions.shape[0]:
        raise ValueError("timestamps and positions must have the same length")
    if not anchors:
        raise ValueError("at least one anchor is required")
    if sigma_m < 0.0:
        raise ValueError("sigma_m must be non-negative")

    rng = np.random.default_rng(random_seed)
    frames = []
    for index in range(stamps.size):
        samples = []
        for anchor in anchors:
            ideal = float(np.linalg.norm(positions[index] - np.asarray(anchor.position_m)))
            noise = rng.normal(0.0, sigma_m) if sigma_m > 0.0 else 0.0
            samples.append(RangeSample(
                anchor_id=anchor.id,
                anchor_position_m=anchor.position_m,
                range_m=max(0.0, ideal + noise),
                stddev_m=float(sigma_m),
            ))
        frames.append(UwbObservationFrame(
            timestamp_ns=int(stamps[index]),
            tag_id=tag_id,
            ranges=samples,
        ))
    return frames
