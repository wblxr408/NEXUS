"""Convert labelled six-column UWB LOS/NLOS ranges into replayable range packets.

The converter requires an explicit surveyed/simulation anchor layout.  It does
not infer geometry from the third-party range file, so a generated packet keeps
its coordinate frame, units, source file and condition label auditable.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ranges", type=Path, required=True, help="TSV: timestamp, tag id, four millimetre ranges")
    parser.add_argument("--anchor-layout", type=Path, required=True)
    parser.add_argument("--condition", choices=("los", "nlos"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timestamp-scale-ns", type=int, default=1_000_000,
                        help="multiply raw timestamp by this value (default milliseconds to nanoseconds)")
    parser.add_argument("--variance-m2", type=float, default=.04)
    parser.add_argument("--allow-simulation-layout", action="store_true")
    args = parser.parse_args()
    if (not args.ranges.is_file() or not args.anchor_layout.is_file() or args.output.exists()
            or args.timestamp_scale_ns < 1 or not np.isfinite(args.variance_m2) or args.variance_m2 <= 0):
        raise ValueError("NLOS packet conversion arguments are invalid")
    layout = yaml.safe_load(args.anchor_layout.read_text(encoding="utf-8"))
    if not isinstance(layout, dict) or layout.get("coordinate_frame") != "map" or layout.get("unit") != "m":
        raise ValueError("anchor layout must explicitly use map/metres")
    if layout.get("status") != "surveyed" and not args.allow_simulation_layout:
        raise ValueError("non-surveyed anchor layout requires --allow-simulation-layout")
    anchors = layout.get("anchors")
    if not isinstance(anchors, list) or len(anchors) != 4:
        raise ValueError("six-column range input requires exactly four ordered anchors")
    anchor_ids = [item.get("id") for item in anchors]
    if any(not isinstance(identifier, str) or not identifier for identifier in anchor_ids) or len(set(anchor_ids)) != 4:
        raise ValueError("anchor IDs must be unique nonempty strings")
    rows = np.loadtxt(args.ranges, dtype=np.int64, ndmin=2)
    if rows.ndim != 2 or rows.shape[1] != 6 or np.any(rows[:, 0] <= 0) or np.any(rows[:, 2:] <= 0):
        raise ValueError("range file must contain positive six-column integer rows")
    stamps = rows[:, 0] * args.timestamp_scale_ns
    if np.any(np.diff(stamps) <= 0):
        raise ValueError("range timestamps must strictly increase")
    with args.output.open("x", encoding="utf-8") as stream:
        for stamp, row in zip(stamps, rows):
            packet = {"schema_version": 1, "sample_timestamp_ns": int(stamp), "frame_id": "map", "unit": "m",
                      "anchor_ids": anchor_ids, "ranges_m": (row[2:].astype(float) / 1000.).tolist(),
                      "variances_m2": [float(args.variance_m2)] * 4, "tag_id": int(row[1]),
                      "condition": args.condition, "source_file": str(args.ranges),
                      "anchor_layout_status": layout.get("status")}
            stream.write(json.dumps(packet, allow_nan=False) + "\n")
    print(json.dumps({"packets": int(len(rows)), "condition": args.condition, "anchor_layout": layout.get("layout_id"),
                      "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
