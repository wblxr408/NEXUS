#!/usr/bin/env python3
"""Analyze operator-labelled raw MAVLink captures; never infer physical labels."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def overlapping_allan(values, period_s):
    """Allan deviation of rate samples, with overlapping adjacent mean blocks."""
    values = np.asarray(values, dtype=float)
    cumulative = np.vstack((np.zeros(values.shape[1]), np.cumsum(values, axis=0)))
    sizes = np.unique(np.geomspace(1, max(1, len(values) // 20), 24).astype(int))
    result = []
    for size in sizes:
        means = (cumulative[size:] - cumulative[:-size]) / size
        differences = means[size:] - means[:-size]
        if len(differences):
            result.append({"tau_s": float(size * period_s),
                           "deviation": np.sqrt(np.mean(differences ** 2, axis=0) / 2).tolist(),
                           "overlapping_pairs": len(differences)})
    return result


def load_capture(path):
    messages = []
    with Path(path).open() as stream:
        for line in stream:
            row = json.loads(line)
            if "message" in row:
                messages.append(row)
    return messages


def analyze(path, condition):
    rows = load_capture(path)
    counts = {}
    for row in rows:
        name = row["message"]["mavpackettype"]
        counts[name] = counts.get(name, 0) + 1
    imu = [r for r in rows if r["message"]["mavpackettype"] == "SCALED_IMU"]
    if len(imu) < 100:
        raise ValueError("at least 100 real SCALED_IMU samples are required")
    source = np.array([r["message"]["time_boot_ms"] for r in imu], dtype=np.int64)
    received = np.array([r["receive_unix_ns"] for r in imu], dtype=np.int64)
    dt = np.diff(source) / 1000
    if np.any(dt <= 0):
        raise ValueError("source timestamp duplicate/reset: split the capture before analysis")
    axes = ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro"]
    raw = np.array([[r["message"][a] for a in axes] for r in imu], dtype=float)
    if not np.isfinite(raw).all():
        raise ValueError("nonfinite raw IMU")
    # This is the existing bridge transform, not a claim of physical axis calibration.
    si = raw * np.array([1, -1, -1, 1, -1, -1]) / 1000
    time_s = (source - source[0]) / 1000
    receive_s = (received - received[0]) / 1e9
    slope, intercept = np.polyfit(time_s, receive_s, 1)
    fit_residual = receive_s - (slope * time_s + intercept)
    nominal_dt = (source[-1] - source[0]) / 1000 / (len(source) - 1)
    result = {
        "condition_operator_label": condition,
        "source_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        "counts": counts,
        "imu": {
            "samples": len(imu), "duration_s": float(time_s[-1]), "average_hz": 1 / nominal_dt,
            "source_dt_ms_quantiles": np.quantile(dt * 1000, [0, .5, .95, .99, 1]).tolist(),
            "gaps_over_50ms": int(np.sum(dt > .05)),
            "clock_fit_receive_per_source": float(slope),
            "relative_clock_drift_ppm": float((slope - 1) * 1e6),
            "receive_fit_residual_ms_quantiles": np.quantile(fit_residual * 1000, [0, .5, .95, .99, 1]).tolist(),
            "accel_mean_mps2": si[:, :3].mean(axis=0).tolist(),
            "accel_std_mps2": si[:, :3].std(axis=0, ddof=1).tolist(),
            "gyro_mean_rps": si[:, 3:].mean(axis=0).tolist(),
            "gyro_std_rps": si[:, 3:].std(axis=0, ddof=1).tolist(),
            "accel_norm_mean_mps2": float(np.linalg.norm(si[:, :3], axis=1).mean()),
            "gyro_integral_rad": np.trapz(si[:, 3:], time_s, axis=0).tolist(),
            "gyro_peak_abs_rps": np.abs(si[:, 3:]).max(axis=0).tolist(),
        },
        "limitations": [
            "Source-to-receive clock fit measures transport jitter/drift, not absolute sensor latency.",
            "Battery UWB slots have receive timestamps only; source measurement delay remains unknown.",
            "Existing diag(1,-1,-1) conversion requires operator-labelled axis verification.",
        ],
    }
    if condition == "stationary_confirmed":
        # Interpolate to a uniform grid only for spectral/Allan summary, and disclose it.
        grid = np.arange(0, time_s[-1], nominal_dt)
        uniform = np.column_stack([np.interp(grid, time_s, si[:, i]) for i in range(6)])
        result["imu"]["allan_uniform_resampled"] = overlapping_allan(uniform, nominal_dt)
        result["imu"]["white_noise_density_estimate"] = (
            np.std(np.diff(uniform, axis=0), axis=0, ddof=1) * np.sqrt(nominal_dt / 2)).tolist()
        result["limitations"].append(
            "Short stationary run characterizes sample noise and gyro offset; full accelerometer bias, "
            "temperature drift and long-term bias random walk are not identifiable from this run alone.")
    batteries = [r for r in rows if r["message"]["mavpackettype"] == "BATTERY_STATUS"]
    if batteries:
        ranges = np.array([r["message"]["voltages"][2:6] for r in batteries], dtype=float)
        valid = (ranges > 0) & (ranges < 65535)
        slots = []
        for i in range(4):
            good = ranges[valid[:, i], i] / 100
            pairs = valid[:-1, i] & valid[1:, i]
            slots.append({"slot": i + 1, "valid_samples": len(good),
                          "invalid_samples": int((~valid[:, i]).sum()),
                          "mean_m": float(good.mean()) if len(good) else None,
                          "std_m": float(good.std()) if len(good) else None,
                          "min_m": float(good.min()) if len(good) else None,
                          "max_m": float(good.max()) if len(good) else None,
                          "adjacent_jumps_over_025m": int(np.sum((np.abs(np.diff(ranges[:, i])) > 25) & pairs))})
        result["uwb"] = {"samples": len(batteries), "slots": slots}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--condition", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.input, args.condition)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
