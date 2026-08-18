import argparse
import json

import numpy as np

from gdop import layout_summary


def main(argv=None):
    parser = argparse.ArgumentParser(description="Compute UWB anchor-layout GDOP")
    parser.add_argument("--input", required=True, help="JSON file with anchors and points arrays")
    parser.add_argument("--output", required=True, help="JSON output path")
    args = parser.parse_args(argv)
    with open(args.input, "r", encoding="utf-8") as stream:
        document = json.load(stream)
    summary = layout_summary(np.asarray(document["anchors"], dtype=float), np.asarray(document["points"], dtype=float))
    result = {
        "metric": "gdop", "unit": "dimensionless", "summary": summary,
        "coordinate_frame": document.get("frame", "TBD"),
        "samples": int(summary["samples"]),
        "statistic_definition": "trace of the pseudoinverse of the anchor geometry normal matrix",
        "outlier_rule": "none; geometry points are evaluated as supplied",
        "outlier_count": 0, "dropped_frames": 0,
        "is_test_sample": bool(document.get("is_test_sample", False)),
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
