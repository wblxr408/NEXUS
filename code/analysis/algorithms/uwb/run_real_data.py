#!/usr/bin/env python3
"""Run all five Python UWB algorithms on measured sandbox observations."""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import Counter, defaultdict
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
REPO_ROOT = HERE.parents[4]
sys.path.insert(0, str(REPO_ROOT / "code" / "analysis"))
sys.path.insert(0, str(REPO_ROOT / "simulation"))

from algorithms.router import AlgorithmRouter, register_default_algorithms
from evaluation.metrics import latency_metrics, position_metrics_2d
from algorithms.uwb.real_data import load_observation_frames, load_truth_points, reset_runner_state, sha256_file


ALGORITHMS = (
    "uwb.matlab.trilateration", "uwb.matlab.multilateration", "uwb.matlab.taylor",
    "uwb.matlab.ekf", "uwb.matlab.ukf",
)
DISPLAY_NAMES = {
    "uwb.matlab.trilateration": "Trilateration", "uwb.matlab.multilateration": "Multilateration",
    "uwb.matlab.taylor": "Taylor", "uwb.matlab.ekf": "EKF", "uwb.matlab.ukf": "UKF",
}


def _config(path):
    if not path:
        return {}
    import yaml
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _run_algorithm(name, frames, config):
    runner = AlgorithmRouter(name).spec.runner
    reset_runner_state(runner)
    rows = []
    previous_group = None
    params = config.get(name, config.get(DISPLAY_NAMES[name].lower(), {})) or {}
    initial_xy = params.get("initial_xy", params.get("initial_xy_m"))
    if initial_xy is None:
        initial_xy = (config.get("filter_initialization") or {}).get("initial_xy_m")
    for frame in frames:
        group = (frame.point_id, frame.repeat_id)
        if group != previous_group:
            reset_runner_state(runner)
            previous_group = group
        kwargs = {"observation_frame": frame}
        if initial_xy is not None:
            try:
                kwargs["initial_xy"] = tuple(float(value) for value in initial_xy)
            except (TypeError, ValueError) as exc:
                raise ValueError("filter initial_xy must be two numeric metre values") from exc
            if len(kwargs["initial_xy"]) != 2:
                raise ValueError("filter initial_xy must contain exactly two values")
        started = time.monotonic_ns()
        try:
            if len(frame.ranges) < 3:
                raise ValueError(f"insufficient anchors: {len(frame.ranges)} (minimum 3)")
            result = AlgorithmRouter(name).run(**kwargs)
            elapsed_ms = (time.monotonic_ns() - started) / 1e6
            estimate = np.asarray(result.estimate, dtype=float)
            valid = bool(result.valid and estimate.size >= 2 and np.all(np.isfinite(estimate[:2])))
            reason = "" if valid else str(result.metadata.get("error", "invalid result"))
            if not valid:
                estimate = np.full(2, np.nan)
        except Exception as exc:  # keep one bad frame visible in the result table
            elapsed_ms = (time.monotonic_ns() - started) / 1e6
            estimate = np.full(2, np.nan)
            valid, reason = False, str(exc)
        rows.append({
            "algorithm": name, "algorithm_display": DISPLAY_NAMES[name],
            "point_id": frame.point_id, "repeat_id": frame.repeat_id,
            "frame_seq": frame.frame_seq, "sample_timestamp_ns": frame.timestamp_ns,
            "receive_timestamp_ns": frame.receive_timestamp_ns,
            "estimate_x_m": float(estimate[0]), "estimate_y_m": float(estimate[1]),
            "valid": str(valid).lower(), "runtime_ms": float(elapsed_ms), "error_reason": reason,
        })
    return rows


def _nearest_grid(rows, truth, frequency_hz=10.0, tolerance_ms=50.0):
    by_group = defaultdict(list)
    valid_by_group = defaultdict(list)
    for row in rows:
        if row["point_id"] in truth:
            by_group[(row["point_id"], row["repeat_id"])].append(row)
        if row["valid"] == "true" and row["point_id"] in truth:
            valid_by_group[(row["point_id"], row["repeat_id"])].append(row)
    matched = []
    expected = 0
    for group, group_rows in by_group.items():
        group_rows.sort(key=lambda row: row["sample_timestamp_ns"])
        start, end = group_rows[0]["sample_timestamp_ns"], group_rows[-1]["sample_timestamp_ns"]
        step = int(round(1e9 / frequency_hz))
        grid = range(start, end + 1, step)
        expected += len(range(start, end + 1, step))
        candidates = valid_by_group.get(group, [])
        for stamp in grid:
            if not candidates:
                continue
            nearest = min(candidates, key=lambda row: abs(row["sample_timestamp_ns"] - stamp))
            if abs(nearest["sample_timestamp_ns"] - stamp) <= tolerance_ms * 1e6:
                matched.append((nearest, truth[nearest["point_id"]]))
    return matched, expected


def _metrics(rows, truth, *, frequency_hz=10.0, tolerance_ms=50.0, outlier_threshold_m=0.30):
    matched, expected = _nearest_grid(rows, truth, frequency_hz, tolerance_ms)
    estimates = [[row["estimate_x_m"], row["estimate_y_m"]] for row, _ in matched]
    references = [[point["x_m"], point["y_m"]] for _, point in matched]
    update_stamps = sorted({row["sample_timestamp_ns"] for row in rows if row["valid"] == "true"})
    duration_s = (update_stamps[-1] - update_stamps[0]) / 1e9 if len(update_stamps) >= 2 else 0.0
    result = {"dimension": "2D", "unit": "m", "time_grid_hz": frequency_hz,
              "matching_tolerance_ms": tolerance_ms, "total_expected": expected,
              "valid_count": len(matched), "success_rate": len(matched) / expected if expected else 0.0,
              "failure_rate": 1.0 - (len(matched) / expected if expected else 0.0),
              "update_frequency_hz": len(update_stamps) / duration_s if duration_s > 0 else None,
              "mean_runtime_ms": float(np.mean([row["runtime_ms"] for row in rows])) if rows else float("nan"),
              "invalid_output_count": sum(row["valid"] != "true" for row in rows)}
    result["error_reasons"] = dict(Counter(row["error_reason"] for row in rows if row["valid"] != "true"))
    if estimates:
        result.update(position_metrics_2d(estimates, references))
        distances = np.linalg.norm(np.asarray(estimates) - np.asarray(references), axis=1)
        result["outlier_threshold_m"] = outlier_threshold_m
        result["outlier_count"] = int(np.count_nonzero(distances > outlier_threshold_m))
        sample_stamps = [row["sample_timestamp_ns"] for row, _ in matched]
        receive_stamps = [row["receive_timestamp_ns"] for row, _ in matched]
        result["latency"] = latency_metrics(sample_stamps, receive_stamps)
    else:
        result.update({"samples": 0, "rmse_m": float("nan"), "mae_m": float("nan"),
                       "cep50_m": float("nan"), "cep95_m": float("nan"), "max_error_m": float("nan"),
                       "axis_bias_m": {"x": float("nan"), "y": float("nan")},
                       "outlier_threshold_m": outlier_threshold_m, "outlier_count": 0})
    result["p50_definition"] = "2D horizontal Euclidean error percentile; not strict CEP"
    return result


def _breakdowns(rows, truth, *, frequency_hz=10.0, tolerance_ms=50.0, outlier_threshold_m=0.30):
    zones = defaultdict(list)
    points = defaultdict(list)
    repeats = defaultdict(list)
    for row in rows:
        if row["point_id"] not in truth:
            continue
        points[row["point_id"]].append(row)
        zones[truth[row["point_id"]]["zone"]].append(row)
        repeats[row["repeat_id"]].append(row)
    return {
        "by_zone": {key: _metrics(value, truth, frequency_hz=frequency_hz, tolerance_ms=tolerance_ms, outlier_threshold_m=outlier_threshold_m) for key, value in sorted(zones.items())},
        "by_point": {key: _metrics(value, truth, frequency_hz=frequency_hz, tolerance_ms=tolerance_ms, outlier_threshold_m=outlier_threshold_m) for key, value in sorted(points.items())},
        "by_repeat": {key: _metrics(value, truth, frequency_hz=frequency_hz, tolerance_ms=tolerance_ms, outlier_threshold_m=outlier_threshold_m) for key, value in sorted(repeats.items())},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--truth", required=True)
    parser.add_argument("--config", required=False)
    parser.add_argument("--output", required=True)
    parser.add_argument("--run-id", help="experiment run identifier; defaults to the parent run directory name")
    parser.add_argument("--range-unit", choices=("m", "cm", "mm"), default="m")
    args = parser.parse_args(argv)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    run_id = args.run_id or output.parent.name
    frames = load_observation_frames(args.input, range_unit=args.range_unit)
    truth = load_truth_points(args.truth)
    if all(frame.point_id == "unknown" for frame in frames) and len(truth) == 1:
        point_id = next(iter(truth))
        frames = [replace(frame, point_id=point_id) for frame in frames]
    unknown_points = sorted({frame.point_id for frame in frames if frame.point_id not in truth})
    if unknown_points:
        raise ValueError("observations reference point IDs absent from truth CSV: " + ", ".join(unknown_points))
    config = _config(args.config)
    provenance = config.get("provenance") or {}
    config_sha256 = sha256_file(args.config) if args.config else None
    evaluation = config.get("evaluation") or {}
    frequency_hz = float(evaluation.get("grid_frequency_hz", 10.0))
    tolerance_ms = float(evaluation.get("matching_tolerance_ms", 50.0))
    outlier_threshold_m = float(evaluation.get("outlier_threshold_m", 0.30))
    register_default_algorithms()

    all_rows, summary = [], {}
    for name in ALGORITHMS:
        rows = _run_algorithm(name, frames, config)
        for row in rows:
            row["run_id"] = run_id
        all_rows.extend(rows)
        summary[name] = {"global": _metrics(rows, truth, frequency_hz=frequency_hz, tolerance_ms=tolerance_ms,
                                             outlier_threshold_m=outlier_threshold_m),
                         **_breakdowns(rows, truth, frequency_hz=frequency_hz, tolerance_ms=tolerance_ms,
                                       outlier_threshold_m=outlier_threshold_m)}
        (output / f"{DISPLAY_NAMES[name].lower()}.json").write_text(
            json.dumps({"algorithm": name, "run_id": run_id, "data_type": "measured", "is_measured_result": True,
                        "code_commit": provenance.get("code_commit"), "config_sha256": config_sha256,
                        "metrics": summary[name]}, indent=2, allow_nan=True) + "\n", encoding="utf-8")

    fields = ["algorithm", "algorithm_display", "run_id", "point_id", "repeat_id", "frame_seq",
              "sample_timestamp_ns", "receive_timestamp_ns", "estimate_x_m", "estimate_y_m",
              "valid", "runtime_ms", "error_reason"]
    with (output / "results.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(all_rows)
    (output / "summary.json").write_text(json.dumps({"run_id": run_id, "data_type": "measured", "is_measured_result": True,
        "code_commit": provenance.get("code_commit"), "config_sha256": config_sha256,
        "input_sha256": sha256_file(args.input), "truth_sha256": sha256_file(args.truth),
        "algorithms": summary}, indent=2, allow_nan=True) + "\n", encoding="utf-8")
    with (output / "summary.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["algorithm", "rmse_m", "mae_m", "p50_m", "p95_m", "max_error_m", "success_rate", "mean_runtime_ms"])
        for name in ALGORITHMS:
            row = summary[name]["global"]
            writer.writerow([DISPLAY_NAMES[name], row["rmse_m"], row["mae_m"], row["cep50_m"], row["cep95_m"],
                             row["max_error_m"], row["success_rate"], row["mean_runtime_ms"]])
    print(f"Saved measured UWB results to {output}")


if __name__ == "__main__":
    main()
