"""FanciSwarm live MAVLink/UWB protocol helpers.

The live path deliberately keeps flight-controller position out of the UWB
observation frame.  It is only a comparison/safety signal.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from generators.range_simulation import RangeSample, UwbObservationFrame

ANCHORS = (
    ("1", (0.90, -0.90, 2.56)),
    ("2", (0.00, 5.00, 2.71)),
    ("3", (4.00, 5.00, 2.71)),
    ("4", (4.00, 0.00, 2.71)),
)
ANCHOR_ORDER = tuple(item[0] for item in ANCHORS)
TAG_ID = "2"
COORDINATE_YAW_DEG = 9.0
LIVE_STDDEV_M = 0.05


def parse_battery_distances(values):
    """Decode MAVLink BATTERY_STATUS centimetre ranges at indexes 2..5."""
    values = list(values or [])
    if len(values) < 6:
        raise ValueError("BATTERY_STATUS.voltages must contain at least six values")
    distances = []
    for value in values[2:6]:
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("distance value is not numeric") from exc
        if value in (0.0, 65535.0) or not math.isfinite(value) or value <= 0.0:
            raise ValueError("distance must be positive and not a MAVLink sentinel")
        distances.append(value / 100.0)
    return distances


def observation_from_battery(values, timestamp_ns, *, tag_id=TAG_ID):
    distances = parse_battery_distances(values)
    if tuple(anchor_id for anchor_id, _ in ANCHORS) != ANCHOR_ORDER:
        raise RuntimeError("FanciSwarm anchor order is corrupted")
    return UwbObservationFrame(
        timestamp_ns=int(timestamp_ns), tag_id=str(tag_id),
        ranges=[RangeSample(anchor_id, position, distance, LIVE_STDDEV_M)
                for (anchor_id, position), distance in zip(ANCHORS, distances)],
    )


def convert_fc_position(msg):
    """Convert GLOBAL_VISION_POSITION_ESTIMATE centimetres NED to metres."""
    return (float(msg.x) / 100.0, float(msg.y) / 100.0, -float(msg.z) / 100.0)


def horizontal_error(estimate, fc_position):
    if fc_position is None:
        return None
    return math.hypot(float(estimate[0]) - fc_position[0], float(estimate[1]) - fc_position[1])


@dataclass
class FcPosition:
    position_m: tuple[float, float, float]
    timestamp_ns: int

