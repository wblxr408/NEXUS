#!/usr/bin/env python3
"""Convert measured UWB logs to NEXUS's canonical observations.csv."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

try:
    from .real_data import normalize_observations, sha256_file
except ImportError:  # executed as a file, where the package context is absent
    from real_data import normalize_observations, sha256_file


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="raw one-row-per-anchor CSV")
    parser.add_argument("--output", required=True, help="normalized observations.csv")
    parser.add_argument("--range-unit", required=True, choices=("m", "cm", "mm"),
                        help="unit documented by the device interface")
    parser.add_argument("--anchors", help="YAML anchor coordinate file when raw CSV omits coordinates")
    parser.add_argument("--manifest", help="optional JSON data manifest path")
    args = parser.parse_args(argv)
    output = normalize_observations(args.input, args.output, range_unit=args.range_unit,
                                     anchors_path=args.anchors)
    print(f"Wrote {output}")
    output_sha256 = sha256_file(output)
    print(f"sha256: {output_sha256}")
    if args.manifest:
        manifest = {
            "input_path": str(Path(args.input)), "input_sha256": sha256_file(args.input),
            "normalized_path": str(output), "normalized_sha256": output_sha256,
            "range_unit_source": args.range_unit, "normalized_range_unit": "m",
            "data_type": "measured", "is_measured_result": True,
        }
        Path(args.manifest).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
