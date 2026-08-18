import argparse
import json

import numpy as np

from time_alignment import weighted_position


def main(argv=None):
    parser = argparse.ArgumentParser(description="Fuse target observations with covariance weighting")
    parser.add_argument("--input", required=True, help="JSON file with positions and covariances")
    parser.add_argument("--output", required=True, help="JSON output path")
    args = parser.parse_args(argv)
    with open(args.input, "r", encoding="utf-8") as stream:
        document = json.load(stream)
    position, covariance = weighted_position(document["positions"], document["covariances"])
    result = {
        "position_m": position.tolist(),
        "covariance_m2": covariance.tolist(),
        "source_mode": "SOURCE_FUSED",
        "coordinate_frame": document.get("frame", "TBD"),
        "samples": len(document["positions"]),
        "unit": "m",
        "statistic_definition": "inverse-covariance information weighted position",
        "outlier_rule": document.get("outlier_rule", "none; observations are not silently filtered"),
        "outlier_count": int(document.get("outlier_count", 0)),
        "dropped_frames": int(document.get("dropped_frames", 0)),
        "is_test_sample": bool(document.get("is_test_sample", False)),
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
