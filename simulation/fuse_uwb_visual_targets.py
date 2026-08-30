#!/usr/bin/env python3
"""Compose UWB platform position and visual camera poses into map target poses.

Inputs are algorithm outputs only: UWB multilateration estimates, model
camera->target predictions, an explicitly observable onboard attitude stream,
and the calibrated base->camera transform.  It never reads ground_truth/.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml


def _rotation_from_row(row: dict[str, str], prefix: str) -> np.ndarray:
    return np.array([row[f"{prefix}{r}{c}"] for r in range(3) for c in range(3)], dtype=np.float64).reshape(3, 3)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--camera-predictions", required=True)
    parser.add_argument("--uwb-estimates", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    root = Path(args.dataset)
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    camera_predictions = json.loads(Path(args.camera_predictions).read_text(encoding="utf-8"))
    with Path(args.uwb_estimates).open(encoding="utf-8", newline="") as stream:
        uwb = {
            row["sequence"]: {
                "position_m": np.array([row["estimate_x_m"], row["estimate_y_m"], row["estimate_z_m"]], dtype=np.float64),
                "runtime_ms": float(row.get("runtime_ms", 0.0)),
            }
            for row in csv.DictReader(stream)
        }
    with (root / "platform_attitude.csv").open(encoding="utf-8", newline="") as stream:
        attitudes = {row["sequence"]: _rotation_from_row(row, "base_R_map_") for row in csv.DictReader(stream)}
    rotation_camera_base = np.asarray(config["camera"]["base_to_camera_rotation_matrix"], dtype=np.float64)
    translation_camera_base = np.asarray(config["camera"]["base_to_camera_translation_m"], dtype=np.float64)
    rotation_base_camera = rotation_camera_base.T
    map_predictions: dict[str, list[dict]] = {}
    for sequence, items in camera_predictions.items():
        if sequence not in uwb or sequence not in attitudes:
            continue
        rotation_map_base = attitudes[sequence].T
        base_position_map = uwb[sequence]["position_m"]
        fused = []
        for item in items:
            rotation_camera_target = np.asarray(item["R"], dtype=np.float64).reshape(3, 3)
            translation_camera_target = np.asarray(item["t_m"], dtype=np.float64).reshape(3)
            rotation_base_target = rotation_base_camera @ rotation_camera_target
            translation_base_target = rotation_base_camera @ (translation_camera_target - translation_camera_base)
            fused.append({
                "obj_id": int(item["obj_id"]),
                "R": (rotation_map_base @ rotation_base_target).tolist(),
                "t_m": (base_position_map + rotation_map_base @ translation_base_target).tolist(),
                "runtime_ms": float(item.get("runtime_ms", 0.0)) + uwb[sequence]["runtime_ms"],
            })
        map_predictions[sequence] = fused
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(map_predictions, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {sum(len(items) for items in map_predictions.values())} map-target predictions to {output}")


if __name__ == "__main__":
    main()
