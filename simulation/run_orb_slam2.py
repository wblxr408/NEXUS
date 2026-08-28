#!/usr/bin/env python3
"""Run the upstream ORB-SLAM2 monocular TUM example and record run metadata."""
from __future__ import annotations
import argparse, json, os, re, subprocess, time
from pathlib import Path

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--binary", required=True)
    ap.add_argument("--vocabulary", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--timeout", type=int, default=600)
    args = ap.parse_args()
    dataset, output = Path(args.dataset).resolve(), Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # Linux builds commonly place Pangolin/ROS shared objects outside ldconfig.
    extra = ["/opt/ros/humble/lib", str(Path(args.binary).resolve().parent.parent.parent / "lib")]
    env["LD_LIBRARY_PATH"] = ":".join(extra + [env.get("LD_LIBRARY_PATH", "")])
    command = [str(Path(args.binary).resolve()), str(Path(args.vocabulary).resolve()), str((dataset / "camera.yaml").resolve()), str(dataset)]
    started = time.perf_counter()
    proc = subprocess.run(command, cwd=output, env=env, text=True, capture_output=True, timeout=args.timeout)
    elapsed = time.perf_counter() - started
    (output / "stdout.log").write_text(proc.stdout, encoding="utf-8")
    (output / "stderr.log").write_text(proc.stderr, encoding="utf-8")
    trajectory = output / "KeyFrameTrajectory.txt"
    frame_trajectory = output / "FrameTrajectoryTcw.csv"
    keyframes = sum(1 for line in trajectory.read_text(encoding="utf-8").splitlines() if line.strip()) if trajectory.exists() else 0
    frame_count = len((dataset / "rgb.txt").read_text(encoding="utf-8").splitlines()) - 3
    valid_frames = 0
    if frame_trajectory.exists():
        with frame_trajectory.open(encoding="utf-8", newline="") as stream:
            valid_frames = sum(1 for _ in stream) - 1
        valid_frames = max(0, valid_frames)
    # The C++ example prints tracking timing to stdout; preserve it as a
    # measured runtime indicator when available instead of leaving it blank.
    log_text = proc.stdout + "\n" + proc.stderr
    mean_match = re.search(r"mean tracking time:\s*([0-9.eE+-]+)", log_text)
    median_match = re.search(r"median tracking time:\s*([0-9.eE+-]+)", log_text)
    mean_ms = float(mean_match.group(1)) * 1000.0 if mean_match else None
    median_ms = float(median_match.group(1)) * 1000.0 if median_match else None
    metrics = {"algorithm": "orb_slam2.monocular", "return_code": proc.returncode, "frame_count": frame_count,
               "keyframe_count": keyframes, "valid_frame_count": valid_frames,
               "availability": valid_frames / frame_count if frame_count else 0.0,
               "failure_rate": 1.0 - valid_frames / frame_count if frame_count else 1.0,
               "update_rate_hz": None, "latency_mean_ms": None, "latency_p95_ms": None,
               "runtime_mean_ms": mean_ms, "runtime_median_ms": median_ms,
               "runtime_total_s": elapsed, "scale_alignment": "none", "note": "KeyFrameTrajectory is keyframe-only; no output means monocular initialization/tracking failed."}
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    raise SystemExit(proc.returncode)

if __name__ == "__main__":
    main()
