#!/usr/bin/env python3
"""Run one localization chain over a bundle JSON and write the record contract.

The three chains are selected by name only, so A/B/C can be run separately over
the same input and compared side by side.  Ground truth in the input document is
never read as a solver input; it is only echoed into the summary when present.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from algorithms.router import AlgorithmRouter

from localization.bundle_io import load_problem
from localization.gating import QualityMetrics
from localization.records import result_records, write_records


CHAIN_ALGORITHMS = {"chain_a_precision": "vision.bundle_a", "chain_b_robust": "vision.bundle_b", "chain_c_adaptive": "vision.bundle_c"}


def _truth_summary(document: dict, records: list) -> dict:
    """Report translation error only when the document carries map truth."""
    truth = document.get("truth") or {}
    errors = []
    for record in records:
        entry = truth.get(str(record.obj_id))
        if entry is None:
            continue
        reference = np.asarray(entry["translation_m"], dtype=float)
        errors.append(float(np.linalg.norm(np.array([record.x_map_m, record.y_map_m, record.z_map_m]) - reference)))
    if not errors:
        return {"truth_available": False}
    return {"truth_available": True, "compared_targets": len(errors),
            "translation_rmse_3d_m": float(np.sqrt(np.mean(np.square(errors)))),
            "translation_max_3d_m": float(np.max(errors))}


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, help="bundle problem JSON")
    parser.add_argument("--chain", required=True, choices=sorted(CHAIN_ALGORITHMS))
    parser.add_argument("--config", help="override the chain config YAML")
    parser.add_argument("--out", required=True, help="output record CSV path")
    parser.add_argument("--class-names", help="JSON mapping of object id to class name")
    args = parser.parse_args(argv)

    problem, document = load_problem(args.bundle)
    inputs = {"problem": problem}
    if args.config:
        inputs["config_path"] = args.config
    quality = document.get("quality")
    if args.chain == "chain_c_adaptive":
        if quality is None:
            raise ValueError("chain C needs a 'quality' block in the bundle document")
        inputs["quality"] = QualityMetrics(**quality)
    elif quality is not None:
        inputs["quality"] = QualityMetrics(**quality)

    result = AlgorithmRouter(CHAIN_ALGORITHMS[args.chain]).run(**inputs)
    class_names = {int(key): str(value) for key, value in json.loads(Path(args.class_names).read_text(encoding="utf-8")).items()} if args.class_names else {}
    records = result_records(result, frame_id=str(document.get("frame_id", "0")), chain=args.chain, class_names=class_names)
    if records:
        write_records(args.out, records)
    summary = {"bundle": args.bundle, "chain": args.chain, "algorithm": result.algorithm,
               "valid": bool(result.valid), "records": len(records), "records_path": args.out if records else None,
               "metadata": result.metadata, **_truth_summary(document, records)}
    print(json.dumps(summary, indent=2, default=str))


if __name__ == "__main__":
    main()
