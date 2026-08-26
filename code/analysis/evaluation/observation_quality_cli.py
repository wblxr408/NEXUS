#!/usr/bin/env python3
import argparse
import csv
import json

from observation_quality import observation_quality


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Summarize target-observation validity, sources, timing and visual quality")
    parser.add_argument("--input", required=True, help="normalized NEXUS observation CSV")
    parser.add_argument("--output", required=True, help="quality report JSON")
    parser.add_argument("--expected-rate-hz", type=float)
    args = parser.parse_args(argv)
    with open(args.input, "r", encoding="utf-8", newline="") as stream:
        records = list(csv.DictReader(stream))
    result = observation_quality(records, args.expected_rate_hz)
    result.update({
        "data_type": records[0].get("data_type", "unknown") if records else "unknown",
        "coordinate_frame": records[0].get("frame_id", "TBD") if records else "TBD",
        "unit": "m",
        "accuracy_claim": "none; this report describes observation quality only",
    })
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
