#!/usr/bin/env python3
"""Evaluate map -> target pose predictions against isolated map truth."""
import argparse
import json

from pose_metrics import evaluate_map_target_predictions


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--predictions", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    with open(args.predictions, encoding="utf-8") as stream:
        predictions = json.load(stream)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(evaluate_map_target_predictions(args.dataset, predictions), stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
