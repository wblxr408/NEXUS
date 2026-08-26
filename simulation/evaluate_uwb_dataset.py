#!/usr/bin/env python3
"""Run the project UWB multilateration baseline on generated ranges."""
from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import yaml

from algorithms import AlgorithmRouter
from evaluation.metrics import position_metrics, update_frequency_hz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="data/processed/nexus_sandbox_gdr_net_v01")
    parser.add_argument("--config", default="simulation/gdr_net_dataset.yaml")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root, config = Path(args.dataset), yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    anchors = np.asarray([item["position_m"] for item in config["uwb"]["anchors_m"]], dtype=float)
    anchor_names = [item["id"] for item in config["uwb"]["anchors_m"]]
    ranges = {}
    with (root / "trajectory.csv").open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            ranges.setdefault((row["sequence"], row["sample_timestamp_ns"]), {})[row["uwb_anchor_id"]] = float(row["uwb_range_m"])
    truth = {}
    with (root / "ground_truth/platform_pose.csv").open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            truth[(row["sequence"], row["sample_timestamp_ns"])] = np.array([float(row["base_x_m"]), float(row["base_y_m"]), float(row["base_z_m"])])
    estimates, references, timestamps, rows, runtimes_ms, residuals = [], [], [], [], [], []
    for key in sorted(ranges, key=lambda value: int(value[0])):
        measurements = ranges[key]
        if any(name not in measurements for name in anchor_names):
            continue
        start = time.perf_counter()
        estimate = AlgorithmRouter("uwb.multilateration").run(anchors_m=anchors, ranges_m=[measurements[name] for name in anchor_names]).estimate
        runtime_ms = (time.perf_counter() - start) * 1000.0
        runtimes_ms.append(runtime_ms)
        residuals.extend(np.linalg.norm(estimate - anchors, axis=1) - np.array([measurements[name] for name in anchor_names]))
        reference = truth[key]
        estimates.append(estimate); references.append(reference); timestamps.append(int(key[1]))
        rows.append({"sequence": key[0], "sample_timestamp_ns": key[1], "estimate_x_m": estimate[0], "estimate_y_m": estimate[1], "estimate_z_m": estimate[2], "reference_x_m": reference[0], "reference_y_m": reference[1], "reference_z_m": reference[2], "runtime_ms": runtime_ms, "frame": "map"})
    result = position_metrics(estimates, references)
    distances = np.linalg.norm(np.asarray(estimates) - np.asarray(references), axis=1)
    errors = np.asarray(estimates) - np.asarray(references)
    horizontal_errors = np.linalg.norm(errors[:, :2], axis=1)
    result.update({
        "position_rmse_3d_m": result.pop("rmse_m"),
        "position_mae_3d_m": float(np.mean(distances)),
        "position_p50_3d_m": result.pop("cep50_m"),
        "position_p95_3d_m": result.pop("cep95_m"),
        "position_max_3d_m": result.pop("max_error_m"),
        "position_bias_x_m": float(np.mean(errors[:, 0])),
        "position_bias_y_m": float(np.mean(errors[:, 1])),
        "position_bias_z_m": float(np.mean(errors[:, 2])),
        "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal_errors ** 2))),
        "height_rmse_m": float(np.sqrt(np.mean(errors[:, 2] ** 2))),
        "availability": float(len(estimates) / len(truth)) if truth else 0.0,
        "failure_rate": float(1.0 - len(estimates) / len(truth)) if truth else 1.0,
        "update_rate_hz": update_frequency_hz(timestamps),
        "runtime_mean_ms": float(np.mean(runtimes_ms)),
        "runtime_p95_ms": float(np.percentile(runtimes_ms, 95)),
        "latency_mean_ms": None,
        "latency_p95_ms": None,
        "recovery_time_ms": 0.0 if len(estimates) == len(truth) else None,
        "range_residual_rmse_m": float(np.sqrt(np.mean(np.asarray(residuals) ** 2))),
        "range_residual_p95_m": float(np.percentile(np.abs(residuals), 95)),
        "algorithm_name": "uwb.multilateration",
        "coordinate_frame": "map",
        "sample_count": len(truth),
        "valid_sample_count": len(estimates),
        "invalid_sample_count": len(truth) - len(estimates),
        "unit": "m",
        "statistic_definition": "Euclidean map -> base_link position error; no alignment or truth fallback",
    })
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2); stream.write("\n")
    with output.with_name(output.stem + "_estimates.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
