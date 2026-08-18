import argparse
import csv
import json
from pathlib import Path

import numpy as np

from metrics import latency_metrics, position_metrics, update_frequency_hz


def load_records(path):
    """Read the registered JSON schema or a simple CSV export schema."""
    path = Path(path)
    if path.suffix.lower() != ".csv":
        with path.open("r", encoding="utf-8") as stream:
            return json.load(stream)
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("CSV input must contain at least one row")
    required = ("x_m", "y_m", "z_m", "ref_x_m", "ref_y_m", "ref_z_m")
    if any(field not in rows[0] for field in required):
        raise ValueError("CSV requires x_m,y_m,z_m,ref_x_m,ref_y_m,ref_z_m columns")
    document = {
        "estimates": [[float(row["x_m"]), float(row["y_m"]), float(row["z_m"])] for row in rows],
        "references": [[float(row["ref_x_m"]), float(row["ref_y_m"]), float(row["ref_z_m"])] for row in rows],
        "frame": rows[0].get("frame", "TBD"),
        "is_test_sample": False,
    }
    if "timestamp_ns" in rows[0]:
        document["timestamps_ns"] = [int(row["timestamp_ns"]) for row in rows]
    if "receive_timestamp_ns" in rows[0]:
        document["receive_timestamps_ns"] = [int(row["receive_timestamp_ns"]) for row in rows]
        if "timestamp_ns" in document:
            document["sample_timestamps_ns"] = document["timestamps_ns"]
    return document


def main(argv=None):
    parser = argparse.ArgumentParser(description="Evaluate target localization records")
    parser.add_argument("--input", required=True, help="JSON record or CSV export with estimates/references")
    parser.add_argument("--output", required=True, help="JSON output path")
    args = parser.parse_args(argv)
    document = load_records(args.input)
    result = {
        "coordinate_frame": document.get("frame", "TBD"),
        "is_test_sample": bool(document.get("is_test_sample", False)),
        "statistic_definition": "Euclidean position errors over finite estimate/reference pairs",
        "outlier_rule": document.get("outlier_rule", "none; no samples removed by default"),
        "outlier_count": int(document.get("outlier_count", 0)),
        "dropped_frames": int(document.get("dropped_frames", 0)),
        "position": position_metrics(
            document["estimates"], document["references"],
            document.get("outlier_threshold_m")),
    }
    result["samples"] = result["position"]["samples"]
    result["unit"] = result["position"]["unit"]
    if "outlier_threshold_m" in document:
        result["outlier_count"] = result["position"]["outlier_count"]
    if "timestamps_ns" in document:
        result["update_frequency_hz"] = update_frequency_hz(document["timestamps_ns"])
    if "sample_timestamps_ns" in document and "receive_timestamps_ns" in document:
        result["latency"] = latency_metrics(document["sample_timestamps_ns"], document["receive_timestamps_ns"])
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
