#!/usr/bin/env python3
"""Recover a metric map frame for monocular ORB-SLAM2 from UWB positions.

The Sim(3) fit consumes only ORB's Tcw records and independently computed UWB
multilateration outputs. Ground truth is never used to construct the estimate;
it is read only for the synthetic-experiment metrics.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def umeyama(source: np.ndarray, target: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    source_mean, target_mean = source.mean(0), target.mean(0)
    centered_source, centered_target = source - source_mean, target - target_mean
    u, singular_values, vt = np.linalg.svd(centered_target.T @ centered_source / len(source))
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(u @ vt)
    rotation = u @ correction @ vt
    scale = np.trace(np.diag(singular_values) @ correction) / np.mean(np.sum(centered_source * centered_source, axis=1))
    translation = target_mean - scale * rotation @ source_mean
    return float(scale), rotation, translation


def _load_orb_poses(path: Path) -> tuple[list[int], np.ndarray, np.ndarray]:
    sequences, positions, rotations = [], [], []
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            sequence = int(row["sequence"])
            rotation = np.array([[float(row[f"Tcw{r}{c}"]) for c in range(3)] for r in range(3)])
            translation = np.array([float(row[f"Tcw{r}3"]) for r in range(3)])
            sequences.append(sequence)
            positions.append(-rotation.T @ translation)
            rotations.append(rotation)
    return sequences, np.asarray(positions), np.asarray(rotations)


def _load_uwb(path: Path) -> dict[int, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as stream:
        return {
            int(row["sequence"]): np.array([row["estimate_x_m"], row["estimate_y_m"], row["estimate_z_m"]], dtype=float)
            for row in csv.DictReader(stream)
        }


def _write_metric_pose(path: Path, sequences: list[int], positions: np.ndarray, rotations_camera_map: np.ndarray, timestamps: dict[int, int]) -> None:
    fields = ["sequence", "sample_timestamp_ns", "camera_x_m", "camera_y_m", "camera_z_m"]
    fields += [f"R_camera_map_{r}{c}" for r in range(3) for c in range(3)]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sequence, position, rotation in zip(sequences, positions, rotations_camera_map):
            writer.writerow({
                "sequence": sequence,
                "sample_timestamp_ns": timestamps[sequence],
                "camera_x_m": position[0], "camera_y_m": position[1], "camera_z_m": position[2],
                **{f"R_camera_map_{r}{c}": rotation[r, c] for r in range(3) for c in range(3)},
            })


def _position_stats(errors: np.ndarray) -> dict[str, float | list[float]]:
    distances = np.linalg.norm(errors, axis=1)
    return {
        "position_rmse_3d_m": float(np.sqrt(np.mean(distances**2))),
        "position_mae_3d_m": float(np.mean(distances)),
        "position_p50_3d_m": float(np.percentile(distances, 50)),
        "position_p95_3d_m": float(np.percentile(distances, 95)),
        "position_max_3d_m": float(np.max(distances)),
        "axis_bias_m": [float(value) for value in errors.mean(0)],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--orb", required=True, help="ORB FrameTrajectoryTcw.csv")
    parser.add_argument("--uwb", required=True, help="UWB multilateration *_estimates.csv")
    parser.add_argument("--out", required=True, help="synthetic evaluation JSON")
    parser.add_argument("--pose-out", required=True, help="algorithm-only metric map->camera pose CSV")
    args = parser.parse_args()

    dataset = Path(args.dataset)
    sequences, visual_positions, visual_rotations = _load_orb_poses(Path(args.orb))
    if not sequences:
        raise ValueError("ORB trajectory contains no valid frames")
    uwb = _load_uwb(Path(args.uwb))
    if any(sequence not in uwb for sequence in sequences):
        raise ValueError("ORB and UWB outputs are not synchronized by sequence")
    reference_positions = np.asarray([uwb[sequence] for sequence in sequences])
    scale, global_rotation, global_translation = umeyama(visual_positions, reference_positions)
    metric_positions = (scale * (global_rotation @ visual_positions.T)).T + global_translation
    # World coordinates change from W_orb to map as p_map = A p_orb + t;
    # map->camera orientation is therefore R_camera_orb A^T.
    metric_rotations = np.asarray([rotation @ global_rotation.T for rotation in visual_rotations])

    with (dataset / "ground_truth/platform_pose.csv").open(encoding="utf-8", newline="") as stream:
        platform_truth = {int(row["sequence"]): np.array([row["base_x_m"], row["base_y_m"], row["base_z_m"]], dtype=float) for row in csv.DictReader(stream)}
    with (dataset / "ground_truth/camera_pose_map.csv").open(encoding="utf-8", newline="") as stream:
        camera_truth = {
            int(row["sequence"]): {
                "timestamp": int(row["sample_timestamp_ns"]),
                "rotation": np.array([[float(row[f"R_map_camera_{r}{c}"]) for c in range(3)] for r in range(3)]),
            }
            for row in csv.DictReader(stream)
        }
    timestamps = {sequence: item["timestamp"] for sequence, item in camera_truth.items()}
    _write_metric_pose(Path(args.pose_out), sequences, metric_positions, metric_rotations, timestamps)

    errors = metric_positions - np.asarray([platform_truth[sequence] for sequence in sequences])
    rotation_errors = []
    for estimate, sequence in zip(metric_rotations, sequences):
        relative = camera_truth[sequence]["rotation"].T @ estimate
        rotation_errors.append(np.degrees(np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))))
    rotation_errors = np.asarray(rotation_errors)
    valid_timestamps = [timestamps[sequence] for sequence in sequences]
    update_rate = ((len(set(valid_timestamps)) - 1) * 1e9 / (max(valid_timestamps) - min(valid_timestamps))) if len(set(valid_timestamps)) > 1 else None
    result = {
        "algorithm": "orb_slam2.monocular_plus_uwb_sim3",
        "frames_expected": len(platform_truth), "frames_valid": len(sequences),
        "availability": len(sequences) / len(platform_truth),
        "failure_rate": 1.0 - len(sequences) / len(platform_truth),
        "update_rate_hz": update_rate, "recovery_time_ms": None,
        "scale_recovery": {"method": "Umeyama fit to independent UWB multilateration", "scale": scale},
        **_position_stats(errors),
        "rotation_rmse_deg": float(np.sqrt(np.mean(rotation_errors**2))),
        "rotation_p50_deg": float(np.percentile(rotation_errors, 50)),
        "rotation_p95_deg": float(np.percentile(rotation_errors, 95)),
        "latency_mean_ms": None, "latency_p95_ms": None,
        "pose_output": str(Path(args.pose_out)),
        "target_localization": "not produced by ORB-SLAM2 trajectory alone",
    }
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
