#!/usr/bin/env python3
"""Run the deterministic ORB/UWB/GDRN composition and both pose evaluations.

ORB and GDR-Net are external binaries/models, so this entry point starts after
their algorithm-only outputs exist. It makes the final map-target result with
one command and never substitutes ground truth for a missing estimate.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def _run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--orb", required=True)
    parser.add_argument("--uwb", required=True)
    parser.add_argument("--camera-predictions", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    root = Path(args.output_dir)
    root.mkdir(parents=True, exist_ok=True)
    python = sys.executable
    fusion_metrics = root / "orb_uwb_metrics.json"
    camera_poses = root / "orb_uwb_camera_poses.csv"
    _run([python, "simulation/fuse_orb_slam2_uwb.py", "--dataset", args.dataset, "--orb", args.orb,
          "--uwb", args.uwb, "--out", str(fusion_metrics), "--pose-out", str(camera_poses)])
    map_predictions = root / "predictions_map_to_target.json"
    _run([python, "simulation/fuse_orb_visual_targets.py", "--camera-poses", str(camera_poses),
          "--camera-predictions", args.camera_predictions, "--output", str(map_predictions)])
    map_metrics = root / "metrics_map_to_target.json"
    camera_metrics = root / "metrics_camera_to_target.json"
    env = {**__import__("os").environ, "PYTHONPATH": str(Path("code/analysis/evaluation").resolve())}
    subprocess.run([python, "code/analysis/evaluation/pose_evaluate_cli.py", "--dataset", args.dataset,
                    "--predictions", args.camera_predictions, "--output", str(camera_metrics)], check=True, env=env)
    subprocess.run([python, "code/analysis/evaluation/map_pose_evaluate_cli.py", "--dataset", args.dataset,
                    "--predictions", str(map_predictions), "--output", str(map_metrics)], check=True, env=env)
    summary = {
        "platform": json.loads(fusion_metrics.read_text(encoding="utf-8")),
        "camera_to_target": json.loads(camera_metrics.read_text(encoding="utf-8")),
        "map_to_target": json.loads(map_metrics.read_text(encoding="utf-8")),
    }
    (root / "pipeline_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output_dir": str(root), "map_predictions": str(map_predictions)}, indent=2))


if __name__ == "__main__":
    main()
