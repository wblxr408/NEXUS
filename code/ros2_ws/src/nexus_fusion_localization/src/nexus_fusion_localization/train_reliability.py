#!/usr/bin/env python3
"""Train on labelled CSV with collection_group and the four output columns."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from nexus_fusion_localization.reliability import (
    FEATURE_NAMES, OUTPUT_NAMES, ReliabilityMLP, feature_row,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--label-provenance", required=True,
                        help="annotation/ground-truth run ID; explicitly mark synthetic tests")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    path = Path(args.input)
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"collection_group", *OUTPUT_NAMES}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"training CSV requires {sorted(required)}")
        rows = list(reader)
    features = np.array([feature_row({name: row.get(name) or None for name in FEATURE_NAMES}) for row in rows])
    labels = np.array([[float(row[name]) for name in OUTPUT_NAMES] for row in rows])
    model = ReliabilityMLP(seed=args.seed)
    report = model.fit(features, labels, [row["collection_group"] for row in rows],
                       epochs=args.epochs, seed=args.seed, label_provenance=args.label_provenance)
    report["input_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    model.save(args.output)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
