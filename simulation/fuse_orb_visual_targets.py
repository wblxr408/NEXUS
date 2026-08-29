#!/usr/bin/env python3
"""Compose ORB/UWB map->camera estimates with visual camera->target poses.

This is the target-localization stage for a synchronized sequence. It consumes
only algorithm outputs: metric ORB/UWB camera poses and the visual pose model's
predictions. It deliberately has no ground-truth input or fallback.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _rotation(row: dict[str, str]) -> np.ndarray:
    return np.array([[float(row[f"R_camera_map_{r}{c}"]) for c in range(3)] for r in range(3)])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-poses", required=True, help="metric map->camera poses from fuse_orb_slam2_uwb.py")
    parser.add_argument("--camera-predictions", required=True, help="visual camera->target prediction JSON")
    parser.add_argument("--output", required=True, help="map->target prediction JSON")
    args = parser.parse_args()

    with Path(args.camera_poses).open(encoding="utf-8", newline="") as stream:
        camera_poses = {
            row["sequence"]: {
                "position": np.array([row["camera_x_m"], row["camera_y_m"], row["camera_z_m"]], dtype=float),
                "rotation_camera_map": _rotation(row),
            }
            for row in csv.DictReader(stream)
        }
    camera_predictions = json.loads(Path(args.camera_predictions).read_text(encoding="utf-8"))
    output: dict[str, list[dict]] = {}
    for sequence, items in camera_predictions.items():
        pose = camera_poses.get(str(int(sequence)))
        if pose is None:
            continue
        rotation_map_camera = pose["rotation_camera_map"].T
        output[sequence] = []
        for item in items:
            rotation_camera_target = np.asarray(item["R"], dtype=float).reshape(3, 3)
            translation_camera_target = np.asarray(item["t_m"], dtype=float).reshape(3)
            output[sequence].append({
                "obj_id": int(item["obj_id"]),
                "R": (rotation_map_camera @ rotation_camera_target).tolist(),
                "t_m": (pose["position"] + rotation_map_camera @ translation_camera_target).tolist(),
                "runtime_ms": float(item.get("runtime_ms", 0.0)),
                "platform_pose_source": "orb_slam2_monocular_plus_uwb_sim3",
            })
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {sum(len(items) for items in output.values())} map-target predictions to {destination}")


if __name__ == "__main__":
    main()
