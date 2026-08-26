"""Minimal observation frame compatible with the analysis-side runners."""

import types


class ObservationFrame:
    def __init__(self, ranges, anchor_positions, timestamp_ns=0, tag_id="uav_tag"):
        self.timestamp_ns = timestamp_ns
        self.tag_id = tag_id
        self.ranges = [
            types.SimpleNamespace(range_m=r, anchor_position_m=p, stddev_m=0.0)
            for r, p in zip(ranges, anchor_positions)
        ]
