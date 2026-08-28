#!/usr/bin/env python3
"""Evaluate ORB-SLAM2's exported per-frame Tcw trajectory against held-out GT."""
from __future__ import annotations
import argparse, csv, json
from pathlib import Path
import numpy as np


def rotation_error_deg(estimated: np.ndarray, truth: np.ndarray) -> float:
    """Geodesic SO(3) error for map->camera rotations."""
    relative = truth.T @ estimated
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))

def umeyama(source: np.ndarray, target: np.ndarray):
    sx, sy = source.mean(0), target.mean(0)
    x, y = source - sx, target - sy
    u, d, vt = np.linalg.svd((y.T @ x) / len(source))
    sfix = np.eye(3); sfix[-1, -1] = np.linalg.det(u @ vt)
    rot = u @ sfix @ vt
    scale = np.trace(np.diag(d) @ sfix) / np.mean(np.sum(x * x, axis=1))
    trans = sy - scale * rot @ sx
    return scale, rot, trans

def stats(errors: np.ndarray):
    d = np.linalg.norm(errors, axis=1)
    return {"position_rmse_3d_m": float(np.sqrt(np.mean(d*d))), "position_mae_3d_m": float(np.mean(d)),
            "position_p50_3d_m": float(np.percentile(d, 50)), "position_p95_3d_m": float(np.percentile(d, 95)),
            "position_max_3d_m": float(d.max()), "axis_bias_m": [float(v) for v in errors.mean(0)]}

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dataset", required=True); ap.add_argument("--trajectory", required=True); ap.add_argument("--out", required=True)
    a = ap.parse_args(); dataset = Path(a.dataset); trajectory = Path(a.trajectory)
    gt = {}
    with (dataset / "ground_truth/camera_pose_map.csv").open() as f:
        for row in csv.DictReader(f): gt[int(row["sequence"])] = np.array([float(row["camera_x_m"]), float(row["camera_y_m"]), float(row["camera_z_m"])])
    rows = list(csv.DictReader(trajectory.open()))
    ids, est, rotations = [], [], []
    for row in rows:
        R = np.array([[float(row[f"Tcw{r}{c}"]) for c in range(3)] for r in range(3)])
        t = np.array([float(row[f"Tcw{r}3"]) for r in range(3)])
        ids.append(int(row["sequence"])); est.append(-R.T @ t); rotations.append(R)
    est = np.asarray(est); rotations = np.asarray(rotations)
    truth = np.asarray([gt[i] for i in ids])
    gt_timestamps = {}
    with (dataset / "ground_truth/camera_pose_map.csv").open() as f:
        for row in csv.DictReader(f):
            gt_timestamps[int(row["sequence"])] = int(row["sample_timestamp_ns"])
    output_timestamps = [gt_timestamps[i] for i in ids]
    update_rate_hz = ((len(set(output_timestamps)) - 1) * 1e9 /
                      (max(output_timestamps) - min(output_timestamps))) if len(set(output_timestamps)) > 1 else None
    if not ids:
        result = {"algorithm": "orb_slam2.monocular_pnp_compat", "frames_expected": len(gt),
                  "frames_valid": 0, "availability": 0.0, "failure_rate": 1.0,
                  "monocular_scale": {"method": "unavailable: no valid trajectory", "scale": None},
                  "raw_unaligned": None, "sim3_aligned": None, "ate": None,
                  "update_rate_hz": None, "recovery_time_ms": None,
                  "latency_mean_ms": None, "latency_p95_ms": None,
                  "target_localization": "not produced by ORB-SLAM2 trajectory alone"}
        Path(a.out).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return
    raw = stats(est - truth)
    variation = float(np.max(np.linalg.norm(est - est.mean(0), axis=1)))
    truth_variation = float(np.max(np.linalg.norm(truth - truth.mean(0), axis=1)))
    if variation < 1e-5 or truth_variation < 1e-5:
        scale_info, aligned_report = {"method": "unavailable: estimated trajectory is stationary", "scale": None}, None
    else:
        scale, rotation, translation = umeyama(est, truth)
        aligned = (scale * (rotation @ est.T)).T + translation
        aligned_stats = stats(aligned - truth)
        rpe = np.linalg.norm(np.diff(aligned, axis=0) - np.diff(truth, axis=0), axis=1) if len(aligned) > 1 else np.array([])
        # A Sim(3) position alignment also fixes the global map rotation.  The
        # corresponding map->camera estimate is R_est A^T.
        aligned_rotations = np.asarray([r @ rotation.T for r in rotations])
        gt_rotations = []
        with (dataset / "ground_truth/camera_pose_map.csv").open() as f:
            gt_rotation_by_id = {}
            for row in csv.DictReader(f):
                gt_rotation_by_id[int(row["sequence"])] = np.array(
                    [[float(row[f"R_map_camera_{rr}{cc}"]) for cc in range(3)] for rr in range(3)])
        gt_rotations = np.asarray([gt_rotation_by_id[i] for i in ids])
        rotation_errors = np.asarray([rotation_error_deg(e, g) for e, g in zip(aligned_rotations, gt_rotations)])
        aligned_report = {**aligned_stats, "rpe_translation_rmse_m": float(np.sqrt(np.mean(rpe*rpe))) if len(rpe) else None,
                          "rotation_rmse_deg": float(np.sqrt(np.mean(rotation_errors**2))),
                          "rotation_p50_deg": float(np.percentile(rotation_errors, 50)),
                          "rotation_p95_deg": float(np.percentile(rotation_errors, 95)),
                          "rotation_max_deg": float(np.max(rotation_errors))}
        scale_info = {"method": "Sim3 Umeyama fit to GT for diagnostic only", "scale": float(scale)}
    # Raw rotation is meaningful only if the exported map frame has already
    # been externally anchored; report it separately from the diagnostic fit.
    gt_rotation_by_id = {}
    with (dataset / "ground_truth/camera_pose_map.csv").open() as f:
        for row in csv.DictReader(f):
            gt_rotation_by_id[int(row["sequence"])] = np.array(
                [[float(row[f"R_map_camera_{rr}{cc}"]) for cc in range(3)] for rr in range(3)])
    raw_rotation_errors = np.asarray([rotation_error_deg(e, gt_rotation_by_id[i]) for e, i in zip(rotations, ids)])
    result = {"algorithm": "orb_slam2.monocular_pnp_compat", "frames_expected": len(gt), "frames_valid": len(ids),
              "availability": len(ids)/len(gt), "failure_rate": 1-len(ids)/len(gt), "monocular_scale": scale_info,
              "raw_unaligned": {**raw, "rotation_rmse_deg": float(np.sqrt(np.mean(raw_rotation_errors**2))),
                                "rotation_p50_deg": float(np.percentile(raw_rotation_errors, 50)),
                                "rotation_p95_deg": float(np.percentile(raw_rotation_errors, 95)),
                                "rotation_max_deg": float(np.max(raw_rotation_errors))},
              "sim3_aligned": aligned_report,
              "ate": {"raw_position_rmse_3d_m": raw["position_rmse_3d_m"],
                      "sim3_position_rmse_3d_m": aligned_report["position_rmse_3d_m"] if aligned_report else None},
              "update_rate_hz": update_rate_hz, "recovery_time_ms": None,
              "latency_mean_ms": None, "latency_p95_ms": None, "target_localization": "not produced by ORB-SLAM2 trajectory alone"}
    Path(a.out).write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))

if __name__ == "__main__": main()
