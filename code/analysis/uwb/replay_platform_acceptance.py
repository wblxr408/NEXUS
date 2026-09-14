#!/usr/bin/env python3
"""Replay real UWB/IMU into the production platform window, retaining failures."""

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from platform_acceptance import load_capture
from nexus_fusion_localization.imu_preintegration import ImuNoise, ImuReading
from nexus_fusion_localization.platform_measurements import RangeMeasurement
from nexus_fusion_localization.platform_state import BodyState
from nexus_fusion_localization.platform_window import PlatformConfig, PlatformWindow


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--phase", required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--stationary-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-duration-s", type=float, default=0)
    args = parser.parse_args()
    config = yaml.safe_load(args.calibration.read_text())
    rows = [r for r in load_capture(args.input) if r["role"] == args.phase]
    imu = [r for r in rows if r["message"]["mavpackettype"] == "SCALED_IMU"]
    uwb = [r for r in rows if r["message"]["mavpackettype"] == "BATTERY_STATUS"]
    if len(imu) < 100 or len(uwb) < 3:
        raise ValueError("insufficient actual data")
    source = np.array([r["message"]["time_boot_ms"] for r in imu], dtype=np.int64) * 1_000_000
    received = np.array([r["receive_unix_ns"] for r in imu], dtype=np.int64)
    if np.any(np.diff(source) <= 0):
        raise ValueError("source clock reset: split capture")
    scale, offset = np.polyfit((source - source[0]) / 1e9, (received - received[0]) / 1e9, 1)

    def stamp(row):
        return int(source[0] + (((row["receive_unix_ns"] - received[0]) / 1e9 - offset) / scale) * 1e9)

    uwb = [(stamp(r), np.asarray(r["message"]["voltages"][2:6], dtype=float) / 100) for r in uwb]
    uwb = [(t, r) for t, r in uwb if source[0] < t < source[-1] and np.all((r > 0) & (r < 655.35))]
    if args.maximum_duration_s > 0:
        uwb = [(t, r) for t, r in uwb if (t - source[0]) / 1e9 <= args.maximum_duration_s]
    anchors = np.array(list(config["uwb"]["anchors_map_m"].values()))
    height = float(np.mean(config["operator_tag_height_interval_m"]))
    ranges = np.median([r for _, r in uwb[:5]], axis=0)
    fit = least_squares(lambda xy: np.linalg.norm(anchors - np.r_[xy, height], axis=1) - ranges,
                        anchors[:, :2].mean(axis=0), loss="soft_l1", f_scale=.1)
    gravity_body = np.array(json.loads(args.stationary_summary.read_text())["imu"]["accel_mean_mps2"])
    poses = [r["message"] for r in rows if r["message"]["mavpackettype"] == "GLOBAL_VISION_POSITION_ESTIMATE"]
    yaw = -poses[0]["yaw"] + np.deg2rad(config["map_yaw_offset_deg"])
    roll = np.arctan2(gravity_body[1], gravity_body[2])
    pitch = np.arctan2(-gravity_body[0], np.hypot(gravity_body[1], gravity_body[2]))
    rotation = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    noise = ImuNoise(**config["imu"]["noise"])
    window = PlatformWindow(PlatformConfig(**config["window"], imu_noise=noise))
    bias_a = np.array(config["imu"]["initial_accel_bias_mps2"])
    bias_g = np.array(config["imu"]["initial_gyro_bias_rps"])
    state = BodyState(uwb[0][0], np.r_[fit.x, height], np.zeros(3), rotation, bias_a, bias_g)
    covariance = np.diag(np.r_[np.full(3, .5 ** 2), np.full(3, .1 ** 2),
                              [.1 ** 2, .1 ** 2, .5 ** 2], np.full(3, .05 ** 2), np.full(3, .002 ** 2)])
    window.initialize(state, covariance, "approximate_initial_pose")
    for row, t in zip(imu, source):
        m = row["message"]
        a = np.array([m[k] for k in ["xacc", "yacc", "zacc"]]) * [1, -1, -1] / 1000
        g = np.array([m[k] for k in ["xgyro", "ygyro", "zgyro"]]) * [1, -1, -1] / 1000
        window.add_imu(ImuReading(int(t), a, g))
    outputs = []
    for t, r in uwb[1:]:
        measurement = RangeMeasurement(t, anchors, r, np.diag(config["uwb"]["range_variance_m2"]),
                                       config["uwb"]["tag_body_m"])
        update = window.step(t, [measurement])
        s = update.state
        outputs.append({"requested_stamp_ns": t, "state_stamp_ns": s.stamp_ns,
                        "valid": update.valid, "reason": update.reason,
                        "prediction_only": update.prediction_only,
                        "position_m": s.position_m.tolist(), "velocity_mps": s.velocity_mps.tolist(),
                        "rotation_map_body": s.rotation_map_body.tolist(),
                        "diagnostics": update.diagnostics, "runtime_ms": update.runtime_ms})
    valid = [r for r in outputs if r["valid"]]
    result = {"phase": args.phase, "calibration_status": config["calibration_status"],
              "approximations": config["approximations"], "updates": len(outputs), "valid_updates": len(valid),
              "failure_reasons": dict(Counter(r["reason"] for r in outputs if not r["valid"])),
              "initial_range_fit_residual_m": fit.fun.tolist(), "initial_position_m": state.position_m.tolist(),
              "clock_method": "IMU boot clock; UWB receive time mapped by fitted clock; UWB sensor delay unknown",
              "outputs": outputs}
    if valid:
        positions = np.array([r["position_m"] for r in valid])
        result.update(position_span_m=np.ptp(positions, axis=0).tolist(),
                      max_adjacent_position_step_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).max()) if len(valid) > 1 else None,
                      mean_height_m=float(positions[:, 2].mean()))
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "outputs"}))


if __name__ == "__main__":
    main()
