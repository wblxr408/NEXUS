#!/usr/bin/env python3
"""Evaluate a GDR-Net prediction JSON against the generated BOP-style data."""
import argparse
import json

from pose_metrics import evaluate_bop_predictions


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, help="nexus_sandbox_gdr_net_v01 directory")
    parser.add_argument("--predictions", required=True, help="JSON keyed by frame id")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    with open(args.predictions, "r", encoding="utf-8") as stream:
        predictions = json.load(stream)
    result = evaluate_bop_predictions(args.dataset, predictions)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
