"""Summarize a static read-only UWB/IMU capture without inventing ground truth."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def unique_samples(path):
    imu, uwb = [], []
    seen_imu, seen_uwb = set(), set()
    for line in path.read_text().splitlines():
        record = json.loads(line)
        imu_record = record.get("imu") or {}
        imu_stamp = imu_record.get("sample_timestamp_us")
        if imu_stamp and imu_stamp not in seen_imu:
            acceleration = imu_record.get("acceleration_mps2") or {}
            angular_velocity = imu_record.get("angular_velocity_rps") or {}
            values = [acceleration.get(axis) for axis in "xyz"] + [angular_velocity.get(axis) for axis in "xyz"]
            if all(value is not None and np.isfinite(float(value)) for value in values):
                imu.append([float(imu_stamp), *map(float, values)])
                seen_imu.add(imu_stamp)
        uwb_record = record.get("uwb") or {}
        uwb_stamp = uwb_record.get("sample_timestamp_ns")
        ranges = uwb_record.get("anchor_ranges_m") or []
        if uwb_stamp and uwb_stamp not in seen_uwb and len(ranges) == 4:
            values = np.asarray(ranges, dtype=float)
            if np.isfinite(values).all() and np.all(values > 0):
                uwb.append([float(uwb_stamp), *values])
                seen_uwb.add(uwb_stamp)
    return np.asarray(imu, dtype=float), np.asarray(uwb, dtype=float)


def estimate_position(anchors, ranges):
    position = np.mean(anchors, axis=0)
    lower = anchors.min(axis=0) - 5.0
    upper = anchors.max(axis=0) + 5.0
    for _ in range(64):
        delta = position - anchors
        distance = np.linalg.norm(delta, axis=1)
        jacobian = delta / distance[:, None]
        update = np.linalg.lstsq(jacobian, ranges - distance, rcond=None)[0]
        update_norm = np.linalg.norm(update)
        if update_norm > .5:
            update *= .5 / update_norm
        position = np.clip(position + update, lower, upper)
        if np.linalg.norm(update) < 1e-7:
            break
    delta = position - anchors
    distance = np.linalg.norm(delta, axis=1)
    residual = distance - ranges
    geometry = np.column_stack(((anchors - position) / distance[:, None], np.ones(len(anchors))))
    gdop = float(np.sqrt(np.trace(np.linalg.pinv(geometry.T @ geometry))))
    return position, residual, gdop


def sample_rate_hz(stamps, unit_scale):
    intervals = np.diff(stamps) * unit_scale
    intervals = intervals[intervals > 0]
    return float(1.0 / np.median(intervals)) if len(intervals) else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--hardware-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--condition", required=True,
                        help="operator-recorded condition, e.g. static_los or static_nlos_body_occlusion")
    parser.add_argument("--jump-threshold-m", type=float, default=.25)
    parser.add_argument("--fit-residual-threshold-m", type=float, default=.25)
    args = parser.parse_args()
    if args.jump_threshold_m <= 0 or args.fit_residual_threshold_m <= 0:
        raise ValueError("UWB thresholds must be positive")
    config = yaml.safe_load(args.hardware_config.read_text())
    uwb_config = config["uwb"]
    anchor_ids = list(uwb_config["range_slot_anchor_ids"])
    anchors = np.asarray([uwb_config["anchors_map_m"][identifier] for identifier in anchor_ids], dtype=float)
    if anchors.shape != (4, 3) or not np.isfinite(anchors).all():
        raise ValueError("four finite map-frame anchor coordinates are required")
    imu, uwb = unique_samples(args.input)
    if len(uwb) < 10:
        raise ValueError("at least 10 unique complete UWB samples are required")
    positions, residuals, gdops = zip(*(estimate_position(anchors, ranges) for ranges in uwb[:, 1:]))
    positions, residuals, gdops = np.asarray(positions), np.asarray(residuals), np.asarray(gdops)
    fit_rms = np.sqrt(np.mean(residuals ** 2, axis=1))
    fit_valid = fit_rms <= args.fit_residual_threshold_m
    range_steps = np.linalg.norm(np.diff(uwb[:, 1:], axis=0), axis=1)
    imu_result = {
        "unique_samples": int(len(imu)),
        "status": "unavailable_from_capture",
        "reason": "no SCALED_IMU samples with a usable source timestamp were received",
    }
    if len(imu) >= 100:
        imu_result = {
            "unique_samples": int(len(imu)),
            "status": "measured",
            "median_hz": sample_rate_hz(imu[:, 0], 1e-6),
            "acceleration_mean_mps2": np.mean(imu[:, 1:4], axis=0).tolist(),
            "acceleration_std_mps2": np.std(imu[:, 1:4], axis=0).tolist(),
            "angular_velocity_mean_rps": np.mean(imu[:, 4:7], axis=0).tolist(),
            "angular_velocity_std_rps": np.std(imu[:, 4:7], axis=0).tolist(),
        }
    result = {
        "schema_version": 1,
        "source_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "condition": args.condition,
        "coordinate_frame": "map",
        "anchor_ids": anchor_ids,
        "imu": imu_result,
        "uwb": {
            "unique_complete_samples": int(len(uwb)),
            "median_hz": sample_rate_hz(uwb[:, 0], 1e-9),
            "range_mean_m": np.mean(uwb[:, 1:], axis=0).tolist(),
            "range_std_m": np.std(uwb[:, 1:], axis=0).tolist(),
            "jump_threshold_m": args.jump_threshold_m,
            "jump_count": int(np.sum(range_steps > args.jump_threshold_m)),
            "fit_residual_threshold_m": args.fit_residual_threshold_m,
            "geometrically_consistent_samples": int(np.sum(fit_valid)),
            "geometrically_inconsistent_samples": int(np.sum(~fit_valid)),
            "internal_range_fit_residual_rms_m": float(np.sqrt(np.mean(residuals ** 2))),
            "internal_range_fit_residual_median_m": float(np.median(fit_rms)),
            "multilateration_position_mean_m": (np.mean(positions[fit_valid], axis=0).tolist()
                                               if np.any(fit_valid) else None),
            "multilateration_position_std_m": (np.std(positions[fit_valid], axis=0).tolist()
                                              if np.any(fit_valid) else None),
            "gdop_mean": float(np.mean(gdops[fit_valid])) if np.any(fit_valid) else None,
            "gdop_max": float(np.max(gdops[fit_valid])) if np.any(fit_valid) else None,
        },
        "limitations": [
            "No external ground truth was supplied; residual and static spread are consistency metrics, not accuracy.",
            "IMU noise density and bias walk require longer stationary recordings and Allan-deviation analysis.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
