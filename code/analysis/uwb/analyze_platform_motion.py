#!/usr/bin/env python3
"""Operator-labelled axis excursions and IMU/vendor attitude timing checks."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from platform_acceptance import load_capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--stationary-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = load_capture(args.input)
    bias = np.array(json.loads(args.stationary_summary.read_text())["imu"]["gyro_mean_rps"])
    results = {}
    for phase in dict.fromkeys(r["role"] for r in rows):
        selected = [r for r in rows if r["role"] == phase]
        imu = [r for r in selected if r["message"]["mavpackettype"] == "SCALED_IMU"]
        pose = [r for r in selected if r["message"]["mavpackettype"] == "GLOBAL_VISION_POSITION_ESTIMATE"]
        if len(imu) < 2:
            continue
        t = np.array([r["message"]["time_boot_ms"] for r in imu], dtype=np.int64) / 1000
        w = np.array([[r["message"][k] for k in ["xgyro", "ygyro", "zgyro"]] for r in imu]) * [1, -1, -1] / 1000 - bias
        dt = np.diff(t)
        if np.any(dt <= 0):
            raise ValueError("nonmonotonic IMU")
        increments = (w[:-1] + w[1:]) * dt[:, None] / 2
        increments[dt > .05] = 0  # Missing intervals contribute no invented rotation.
        integral = np.vstack((np.zeros(3), np.cumsum(increments, axis=0)))
        item = {"imu_samples": len(imu), "duration_s": float(t[-1] - t[0]),
                "gaps_over_50ms": int(np.sum(dt > .05)),
                "integral_incomplete_due_to_gaps": bool(np.any(dt > .05)),
                "bias_corrected_integral_min_deg": np.rad2deg(integral.min(axis=0)).tolist(),
                "bias_corrected_integral_max_deg": np.rad2deg(integral.max(axis=0)).tolist(),
                "peak_abs_gyro_rps": np.abs(w).max(axis=0).tolist()}
        if phase.startswith("axis_"):
            axis, sign = {"axis_pitch": (1, -1), "axis_roll": (0, 1), "axis_yaw": (2, 1)}[phase]
            signed = integral[:, axis] * sign
            item.update(expected_axis="xyz"[axis], expected_initial_sign=sign,
                        expected_direction_excursion_deg=float(np.rad2deg(signed.max())),
                        reverse_direction_excursion_deg=float(np.rad2deg(-signed.min())))
        if len(pose) > 4:
            # Validate the observed vendor field scale before treating its misleading 'usec' name as ms.
            pt = np.array([r["message"]["usec"] for r in pose], dtype=np.int64) / 1000
            rx = np.array([r["receive_unix_ns"] for r in pose], dtype=np.int64)
            scale = (pt[-1] - pt[0]) / ((rx[-1] - rx[0]) / 1e9)
            item["vendor_pose_field_ms_clock_ratio"] = float(scale)
            if not .98 < scale < 1.02 or np.any(np.diff(pt) <= 0):
                item["timing_status"] = "vendor_pose_clock_not_confirmed"
            else:
                euler = np.array([[r["message"][k] for k in ["roll", "pitch", "yaw"]] for r in pose]) * [1, -1, -1]
                rotations = Rotation.from_euler("xyz", euler)
                delta = (rotations[:-1].inv() * rotations[1:]).as_rotvec()
                rate = delta / np.diff(pt)[:, None]
                midpoint = (pt[:-1] + pt[1:]) / 2
                comparisons = []
                for axis in range(3):
                    if np.std(rate[:, axis]) < .02:
                        comparisons.append({"axis": "xyz"[axis], "status": "insufficient_rotation"})
                        continue
                    scores = []
                    for lag in np.arange(-.3, .301, .005):
                        q = midpoint - lag
                        valid = (q >= t[0]) & (q <= t[-1])
                        right = np.clip(np.searchsorted(t, q), 1, len(t) - 1)
                        valid &= (dt[right - 1] <= .05) & (np.diff(pt) <= .2)
                        observed = np.interp(q[valid], t, w[:, axis])
                        expected = rate[valid, axis]
                        scores.append((float(np.corrcoef(observed, expected)[0, 1]), float(lag)))
                    corr, lag = max(scores)
                    comparisons.append({"axis": "xyz"[axis], "correlation": corr,
                                        "pose_lag_relative_to_imu_s": lag})
                item["imu_vs_vendor_attitude"] = comparisons
        results[phase] = item
    args.output.write_text(json.dumps({"phases": results, "limitations": [
        "Angles are operator approximate; integrated component excursions are not surveyed rotations.",
        "Integrals omit intervals with gaps over 50 ms and are incomplete wherever flagged.",
        "Vendor attitude is a related FCU estimate, not independent ground truth.",
        "IMU/attitude timing does not identify the timestamp-free UWB measurement delay.",
        "First translation repetition changed height per operator; do not label the full phase constant-height.",
    ]}, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
