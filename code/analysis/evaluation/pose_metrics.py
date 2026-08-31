"""Metric-simulation 6D pose evaluation for GDR-Net/BOP-style records.

Prediction translations use metres (NEXUS convention). BOP ground-truth
translations are converted from millimetres on load. No alignment, filtering,
or truth fallback is performed: a missing prediction counts against
availability.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def _array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    shape_matches = result.ndim == len(shape) and all(expected == -1 or actual == expected for actual, expected in zip(result.shape, shape))
    if not shape_matches or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    return result


def rotation_error_deg(rotation_est: Any, rotation_gt: Any) -> float:
    """Geodesic SO(3) error in degrees."""
    estimated = _array(rotation_est, (3, 3), "rotation_est")
    reference = _array(rotation_gt, (3, 3), "rotation_gt")
    relative = reference.T @ estimated
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def symmetry_rotations(model_info: dict[str, Any], continuous_steps: int = 72) -> list[np.ndarray]:
    """Expand standard BOP symmetry fields into model-frame rotations."""
    rotations = [np.eye(3)]
    for value in model_info.get("symmetries_discrete", []):
        rotations.append(_array(value, (16,), "symmetries_discrete").reshape(4, 4)[:3, :3])
    for item in model_info.get("symmetries_continuous", []):
        axis = _array(item["axis"], (3,), "symmetry axis")
        axis /= np.linalg.norm(axis)
        for angle in np.linspace(0.0, 2.0 * np.pi, continuous_steps, endpoint=False)[1:]:
            rotations.append(cv2.Rodrigues(axis * angle)[0])
    return rotations


def model_is_symmetric(model_info: dict[str, Any]) -> bool:
    """Use BOP keys as authority while retaining frozen legacy datasets."""
    if model_info.get("symmetries_discrete") or model_info.get("symmetries_continuous"):
        return True
    return str(model_info.get("symmetry", "none")) not in {"", "none", "false"}


def rotation_error_symmetric_deg(rotation_est: Any, rotation_gt: Any, rotations: list[np.ndarray]) -> float:
    return min(rotation_error_deg(rotation_est, np.asarray(rotation_gt) @ symmetry) for symmetry in rotations)


def _distances(rotation_est, translation_est, rotation_gt, translation_gt, model_points, symmetric):
    estimated_points = (rotation_est @ model_points.T).T + translation_est
    if not symmetric:
        reference_points = (rotation_gt @ model_points.T).T + translation_gt
        return np.linalg.norm(estimated_points - reference_points, axis=1)
    reference_points = (rotation_gt @ model_points.T).T + translation_gt
    # Models in this project are small (tens of vertices); a direct pairwise
    # implementation is transparent and avoids an untracked SciPy dependency.
    pairwise = np.linalg.norm(estimated_points[:, None, :] - reference_points[None, :, :], axis=2)
    return np.min(pairwise, axis=1)


def _percentiles(values):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return {"mean": None, "rmse": None, "p50": None, "p95": None, "max": None}
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def evaluate_pose_records(records, *, diameter_by_object=None, add_threshold_fraction=0.1):
    """Aggregate pose records into the project metric names.

    Each record contains ``rotation_est``, ``translation_est_m``,
    ``rotation_gt``, ``translation_gt_m``, ``model_points_m``, ``symmetric``
    and optionally ``camera_matrix`` and ``projection_points_m``.
    """
    translation_errors, translation_deltas, rotation_errors, add_errors, projection_errors = [], [], [], [], []
    add_successes, add_s_errors, add_s_successes = [], [], []
    pose_2deg_2cm, rotation_2deg, translation_2cm = [], [], []
    reprojection_errors = []
    runtimes, latencies = [], []
    for record in records:
        rotation_est = _array(record["rotation_est"], (3, 3), "rotation_est")
        rotation_gt = _array(record["rotation_gt"], (3, 3), "rotation_gt")
        translation_est = _array(record["translation_est_m"], (3,), "translation_est_m")
        translation_gt = _array(record["translation_gt_m"], (3,), "translation_gt_m")
        model_points = _array(record["model_points_m"], (-1, 3), "model_points_m")
        if model_points.shape[0] == 0:
            raise ValueError("model_points_m must contain at least one point")
        translation_delta = translation_est - translation_gt
        translation_errors.append(float(np.linalg.norm(translation_delta)))
        translation_deltas.append(translation_delta)
        rotations = record.get("symmetry_rotations", [np.eye(3)])
        rotation_error = rotation_error_symmetric_deg(rotation_est, rotation_gt, rotations)
        rotation_errors.append(rotation_error)
        pose_2deg_2cm.append(bool(rotation_error <= 2.0 and np.linalg.norm(translation_delta) <= 0.02))
        rotation_2deg.append(bool(rotation_error <= 2.0))
        translation_2cm.append(bool(np.linalg.norm(translation_delta) <= 0.02))
        distances = _distances(rotation_est, translation_est, rotation_gt, translation_gt, model_points, bool(record.get("symmetric", False)))
        if record.get("symmetric", False):
            add_s_errors.append(float(np.mean(distances)))
            diameter = float(record.get("diameter_m", 0.0))
            add_s_successes.append(bool(diameter > 0 and np.mean(distances) <= add_threshold_fraction * diameter))
        else:
            add_error = float(np.mean(distances))
            add_errors.append(add_error)
            diameter = float(record.get("diameter_m", 0.0))
            add_successes.append(bool(diameter > 0 and add_error <= add_threshold_fraction * diameter))
        if "camera_matrix" in record:
            camera = _array(record["camera_matrix"], (3, 3), "camera_matrix")
            projected_est, _ = cv2.projectPoints(model_points, cv2.Rodrigues(rotation_est)[0], translation_est, camera, np.zeros(5))
            projected_gt, _ = cv2.projectPoints(model_points, cv2.Rodrigues(rotation_gt)[0], translation_gt, camera, np.zeros(5))
            projection_errors.append(float(np.sqrt(np.mean(np.sum((projected_est.reshape(-1, 2) - projected_gt.reshape(-1, 2)) ** 2, axis=1)))))
        if "reprojection_error_px" in record:
            reprojection_errors.append(float(record["reprojection_error_px"]))
        if "runtime_ms" in record:
            runtimes.append(float(record["runtime_ms"]))
        if "latency_ms" in record:
            latencies.append(float(record["latency_ms"]))

    translation = _percentiles(translation_errors)
    rotation = _percentiles(rotation_errors)
    add = _percentiles(add_errors)
    add_s = _percentiles(add_s_errors)
    projection = _percentiles(projection_errors)
    axis_bias = np.mean(translation_deltas, axis=0) if translation_deltas else np.full(3, np.nan)
    horizontal_errors = np.linalg.norm(np.asarray(translation_deltas)[:, :2], axis=1) if translation_deltas else np.array([])
    result = {
        "sample_count": len(records),
        "valid_sample_count": len(records),
        "translation_rmse_3d_m": translation["rmse"],
        "translation_mae_3d_m": translation["mean"],
        "translation_p50_3d_m": translation["p50"],
        "translation_p95_3d_m": translation["p95"],
        "translation_max_3d_m": translation["max"],
        "translation_bias_x_m": float(axis_bias[0]) if translation_deltas else None,
        "translation_bias_y_m": float(axis_bias[1]) if translation_deltas else None,
        "translation_bias_z_m": float(axis_bias[2]) if translation_deltas else None,
        "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal_errors**2))) if horizontal_errors.size else None,
        "rotation_rmse_deg": rotation["rmse"],
        "rotation_mean_deg": rotation["mean"],
        "rotation_p50_deg": rotation["p50"],
        "rotation_p95_deg": rotation["p95"],
        "add_m": add["mean"],
        "add_p95_m": add["p95"],
        "add_recall_at_10_percent_diameter": float(np.mean(add_successes)) if add_successes else None,
        "add_s_m": add_s["mean"],
        "add_s_p95_m": add_s["p95"],
        "add_s_recall": float(np.mean(add_s_successes)) if add_s_successes else None,
        "pose_2deg_2cm_recall": float(np.mean(pose_2deg_2cm)) if pose_2deg_2cm else None,
        "rotation_lt_2deg_recall": float(np.mean(rotation_2deg)) if rotation_2deg else None,
        "translation_lt_2cm_recall": float(np.mean(translation_2cm)) if translation_2cm else None,
        "projection_error_mean_px": projection["mean"],
        "projection_error_p95_px": projection["p95"],
        "reprojection_error_mean_px": float(np.mean(reprojection_errors)) if reprojection_errors else None,
        "reprojection_error_p95_px": float(np.percentile(reprojection_errors, 95)) if reprojection_errors else None,
        "runtime_mean_ms": float(np.mean(runtimes)) if runtimes else None,
        "runtime_p95_ms": float(np.percentile(runtimes, 95)) if runtimes else None,
        "latency_mean_ms": float(np.mean(latencies)) if latencies else None,
        "latency_p95_ms": float(np.percentile(latencies, 95)) if latencies else None,
        "recovery_time_ms": None,
    }
    return result


def _load_ascii_ply(path: Path) -> np.ndarray:
    with path.open("r", encoding="ascii") as stream:
        lines = stream.readlines()
    try:
        vertex_line = next(index for index, line in enumerate(lines) if line.startswith("element vertex "))
        vertex_count = int(lines[vertex_line].split()[2])
        header_end = lines.index("end_header\n") + 1
    except (StopIteration, ValueError, IndexError) as error:
        raise ValueError(f"unsupported PLY header: {path}") from error
    points = [[float(value) for value in line.split()[:3]] for line in lines[header_end:header_end + vertex_count]]
    return _array(np.asarray(points, dtype=np.float64) * 0.001, (-1, 3), "PLY points")


def _timestamps_by_sequence(root: Path) -> dict[str, int]:
    trajectory = root / "trajectory.csv"
    if not trajectory.is_file():
        return {}
    import csv
    with trajectory.open(encoding="utf-8", newline="") as stream:
        return {row["sequence"]: int(row["sample_timestamp_ns"]) for row in csv.DictReader(stream)}


def _update_rate_hz(timestamps_ns: list[int]) -> float | None:
    values = sorted(set(timestamps_ns))
    if len(values) < 2 or values[-1] <= values[0]:
        return None
    return float((len(values) - 1) * 1e9 / (values[-1] - values[0]))


def _finish_availability(result: dict[str, Any], expected: int, valid: int, timestamps_ns: list[int]) -> None:
    result["expected_sample_count"] = expected
    result["invalid_sample_count"] = expected - valid
    result["availability"] = float(valid / expected) if expected else 0.0
    result["failure_rate"] = float(1.0 - result["availability"])
    result["update_rate_hz"] = _update_rate_hz(timestamps_ns)
    # The compact evaluation inputs contain no sensor-to-host transport clock.
    # A complete successful sequence has no recovery interval; missing samples
    # cannot be assigned a recovery time without an output-event log.
    result["recovery_time_ms"] = 0.0 if expected == valid else None


def evaluate_bop_predictions(dataset_root: str | Path, predictions: dict[str, Any], *, expected_objects_per_frame: int | None = None):
    """Evaluate predictions keyed by BOP image/frame id.

    Prediction JSON format::

        {"0": [{"obj_id": 1, "R": [[...]], "t_m": [...]}]}

    Missing objects are failures and are not replaced with ground truth.
    """
    root = Path(dataset_root)
    # v02 is a BOP-style evaluation dataset and stores its frame set in test.
    # Retain v01 compatibility so its frozen geometry-contract record remains
    # evaluable.
    scene_root = root / "test" / "000001"
    if not scene_root.is_dir():
        scene_root = root / "train_pbr" / "000001"
    gt = json.loads((scene_root / "scene_gt.json").read_text(encoding="utf-8"))
    cameras = json.loads((scene_root / "scene_camera.json").read_text(encoding="utf-8"))
    models_dir = root / "models"
    model_cache = {}
    records, expected = [], 0
    timestamps = _timestamps_by_sequence(root)
    valid_timestamps = []
    for frame_id, gt_objects in gt.items():
        expected += len(gt_objects)
        predicted = {int(item["obj_id"]): item for item in predictions.get(str(frame_id), [])}
        camera = np.asarray(cameras[str(frame_id)]["cam_K"], dtype=np.float64).reshape(3, 3)
        for gt_object in gt_objects:
            object_id = int(gt_object["obj_id"])
            prediction = predicted.get(object_id)
            if prediction is None:
                continue
            model_path = models_dir / f"obj_{object_id:06d}.ply"
            if object_id not in model_cache:
                model_cache[object_id] = _load_ascii_ply(model_path)
            gt_rotation = np.asarray(gt_object["cam_R_m2c"], dtype=np.float64).reshape(3, 3)
            gt_translation = np.asarray(gt_object["cam_t_m2c"], dtype=np.float64).reshape(3) * 0.001
            model_info = json.loads((models_dir / "models_info.json").read_text(encoding="utf-8"))[str(object_id)]
            records.append({
                "object_id": object_id,
                "rotation_est": prediction["R"],
                "translation_est_m": prediction["t_m"] if "t_m" in prediction else np.asarray(prediction["t_mm"], dtype=float) * 0.001,
                "rotation_gt": gt_rotation,
                "translation_gt_m": gt_translation,
                "model_points_m": model_cache[object_id],
                "symmetric": model_is_symmetric(model_info),
                "symmetry_rotations": symmetry_rotations(model_info),
                "diameter_m": float(model_info["diameter"]) * 0.001,
                "camera_matrix": camera,
                **({"runtime_ms": prediction["runtime_ms"]} if "runtime_ms" in prediction else {}),
                **({"latency_ms": prediction["latency_ms"]} if "latency_ms" in prediction else {}),
            })
            if frame_id in timestamps:
                valid_timestamps.append(timestamps[frame_id])
    result = evaluate_pose_records(records)
    _finish_availability(result, expected, len(records), valid_timestamps)
    result["coordinate_frame"] = "camera_0"
    result["unit"] = {"translation": "m", "rotation": "deg", "projection": "px"}
    result["statistic_definition"] = "No alignment or filtering; each predicted object is matched by frame and obj_id"
    return result


def evaluate_map_target_predictions(dataset_root: str | Path, predictions: dict[str, Any]):
    """Evaluate map -> target predictions against evaluator-only map truth.

    The prediction layout matches :func:`evaluate_bop_predictions`, but its
    rotations/translations map object coordinates directly into ``map``.
    """
    import csv

    root = Path(dataset_root)
    models_dir = root / "models"
    truth_path = root / "ground_truth" / "target_pose_map.csv"
    truth: dict[tuple[str, int], dict[str, Any]] = {}
    with truth_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rotation_fields = [f"map_R_target_{row_index}{column}" for row_index in range(3) for column in range(3)]
            truth[(row["sequence"], int(row["object_id"]))] = {
                "translation": np.array([row["target_x_m"], row["target_y_m"], row["target_z_m"]], dtype=np.float64),
                "rotation": np.array([row[field] for field in rotation_fields], dtype=np.float64).reshape(3, 3),
                "timestamp_ns": int(row["sample_timestamp_ns"]),
            }
    model_info = json.loads((models_dir / "models_info.json").read_text(encoding="utf-8"))
    model_cache: dict[int, np.ndarray] = {}
    records, expected, valid_timestamps = [], 0, []
    for key, target_truth in truth.items():
        frame_id, object_id = key
        expected += 1
        prediction = next((item for item in predictions.get(frame_id, []) if int(item["obj_id"]) == object_id), None)
        if prediction is None:
            continue
        if object_id not in model_cache:
            model_cache[object_id] = _load_ascii_ply(models_dir / f"obj_{object_id:06d}.ply")
        details = model_info[str(object_id)]
        records.append({
            "object_id": object_id,
            "rotation_est": prediction["R"],
            "translation_est_m": prediction["t_m"],
            "rotation_gt": target_truth["rotation"],
            "translation_gt_m": target_truth["translation"],
            "model_points_m": model_cache[object_id],
            "symmetric": model_is_symmetric(details),
            "symmetry_rotations": symmetry_rotations(details),
            "diameter_m": float(details["diameter"]) * 0.001,
            **({"runtime_ms": prediction["runtime_ms"]} if "runtime_ms" in prediction else {}),
            **({"latency_ms": prediction["latency_ms"]} if "latency_ms" in prediction else {}),
        })
        valid_timestamps.append(target_truth["timestamp_ns"])
    result = evaluate_pose_records(records)
    _finish_availability(result, expected, len(records), valid_timestamps)
    result["coordinate_frame"] = "map"
    result["unit"] = {"translation": "m", "rotation": "deg"}
    result["statistic_definition"] = "No alignment or filtering; each predicted object is matched by sequence and obj_id"
    return result
