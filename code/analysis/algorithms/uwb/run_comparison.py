#!/usr/bin/env python3
"""Run all registered UWB algorithms on a sandbox trajectory at multiple noise levels."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).parents[4]
sys.path.insert(0, str(REPO_ROOT / "code" / "analysis"))
sys.path.insert(0, str(REPO_ROOT / "simulation"))

from algorithms.router import AlgorithmRouter, register_default_algorithms
from evaluation.metrics import position_metrics, success_metrics
from generators.range_simulation import load_anchors, simulate_ranges


def main():
    register_default_algorithms()
    anchors = load_anchors(REPO_ROOT / "simulation" / "configs" / "uwb_simulation_config.yaml")

    t = np.linspace(0.0, 10.0, 100)
    positions = np.column_stack([
        0.5 + 3.0 * (1 - np.cos(2 * np.pi * t / 10)) / 2,
        0.5 + 3.5 * (1 - np.cos(4 * np.pi * t / 10)) / 2,
        np.full_like(t, 1.20),
    ])
    stamps = (t * int(1e9)).astype(np.int64)

    algorithm_names = [
        "uwb.matlab.trilateration",
        "uwb.matlab.multilateration",
        "uwb.matlab.taylor",
        "uwb.matlab.ekf",
        "uwb.matlab.ukf",
        "uwb.awesome_uwb",
    ]
    sigmas = [0.0, 0.02, 0.05, 0.10]
    results = {}

    for sigma in sigmas:
        frames = simulate_ranges(
            anchors, positions, stamps, sigma_m=sigma, random_seed=42, dimensions=3
        )
        for name in algorithm_names:
            runner = AlgorithmRouter(name).spec.runner
            if hasattr(runner, "_state"):
                del runner._state
            estimates = []
            references = []
            valid_timestamps = []
            valid_count = 0
            runtimes_ms = []
            for index, frame in enumerate(frames):
                result = AlgorithmRouter(name).run(observation_frame=frame)
                if result.valid:
                    valid_count += 1
                    estimates.append(result.estimate)
                    references.append(positions[index, :3])
                    valid_timestamps.append(stamps[index])
                runtimes_ms.append(result.metadata.get("runtime_ms", float("nan")))

            if estimates:
                metrics = position_metrics(estimates, references)
            else:
                metrics = {"rmse_m": float("nan"), "mae_m": float("nan")}
            success = success_metrics(len(frames), valid_count, valid_timestamps)
            results[f"{name}@sigma={sigma}"] = {
                **metrics,
                **success,
                "mean_runtime_ms": float(np.nanmean(runtimes_ms)),
            }

    output_path = REPO_ROOT / "outputs" / "tables" / "tbl_uwb_algorithm_comparison_v01.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as stream:
        json.dump(results, stream, indent=2, ensure_ascii=False)

    print(f"{'Algorithm':<30} {'Sigma':>6} {'RMSE':>10} {'MAE':>10} {'P95':>10} {'Success':>8} {'Runtime(ms)':>12}")
    print("-" * 90)
    for key, row in sorted(results.items()):
        name, _, sigma_str = key.partition("@")
        print(f"{name:<30} {sigma_str:>6} {row['rmse_m']:>10.6f} {row['mae_m']:>10.6f}"
              f" {row.get('cep95_m', float('nan')):>10.6f} {row['success_rate']:>8.1%} {row['mean_runtime_ms']:>12.4f}")
    print(f"\nSaved: {output_path}")


if __name__ == "__main__":
    main()
