#!/usr/bin/env python3
"""Run pre-hardware functional replay cases; never use it for experiment data."""

import argparse
import json
from pathlib import Path

from pre_hardware_replay import replay_document


def main(argv=None):
    parser = argparse.ArgumentParser(description="Replay declared pre-hardware test samples")
    parser.add_argument("--input", required=True, help="JSON file marked data_type=test_sample")
    parser.add_argument("--output", required=True, help="functional test report JSON")
    args = parser.parse_args(argv)
    with Path(args.input).open("r", encoding="utf-8") as stream:
        report = replay_document(json.load(stream))
    with Path(args.output).open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    if not report["passed"]:
        raise SystemExit("one or more functional replay cases failed")


if __name__ == "__main__":
    main()
