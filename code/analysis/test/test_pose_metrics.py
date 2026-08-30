import json
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.pose_metrics import evaluate_bop_predictions, evaluate_map_target_predictions, evaluate_pose_records, rotation_error_deg
from vision.gdr_net_numpy_compat import apply_numpy2_compat


def test_rotation_error_identity_is_zero():
    assert rotation_error_deg(np.eye(3), np.eye(3)) == 0.0


def test_gdr_net_numpy_compat_exposes_removed_aliases_when_needed():
    apply_numpy2_compat()
    assert np.float is np.float64
    assert np.maximum_sctype(np.float) == np.longdouble


def test_pose_records_report_zero_for_identical_pose():
    result = evaluate_pose_records([{
        "rotation_est": np.eye(3), "translation_est_m": np.zeros(3),
        "rotation_gt": np.eye(3), "translation_gt_m": np.zeros(3),
        "model_points_m": np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
        "symmetric": False,
    }])
    assert result["translation_rmse_3d_m"] == 0.0
    assert result["rotation_rmse_deg"] == 0.0
    assert result["add_m"] == 0.0


def test_generated_dataset_gt_can_be_round_tripped(tmp_path):
    import subprocess
    dataset = tmp_path / "dataset"
    subprocess.run([sys.executable, "simulation/generate_gdr_net_dataset.py", "--out", str(dataset)], check=True)
    gt = json.loads((dataset / "train_pbr/000001/scene_gt.json").read_text())
    predictions = {
        frame: [{"obj_id": item["obj_id"], "R": np.asarray(item["cam_R_m2c"]).reshape(3, 3).tolist(), "t_m": (np.asarray(item["cam_t_m2c"]) * 0.001).tolist()} for item in items]
        for frame, items in gt.items()
    }
    result = evaluate_bop_predictions(dataset, predictions)
    assert result["expected_sample_count"] == 50
    assert result["valid_sample_count"] == 50
    assert result["availability"] == 1.0
    assert result["translation_rmse_3d_m"] < 1e-12
    assert result["rotation_rmse_deg"] < 1e-3


def test_v02_keeps_map_pose_truth_out_of_bop_camera_metadata(tmp_path):
    import subprocess
    dataset = tmp_path / "dataset"
    subprocess.run([
        sys.executable, "simulation/generate_gdr_net_dataset.py",
        "--config", "simulation/gdr_net_dataset_v02.yaml", "--out", str(dataset),
    ], check=True)
    scene = dataset / "test" / "000001"
    cameras = json.loads((scene / "scene_camera.json").read_text())
    assert set(cameras["0"]) == {"cam_K", "depth_scale"}
    with (dataset / "ground_truth" / "platform_pose.csv").open(newline="", encoding="utf-8") as stream:
        assert "base_R_map_00" in next(csv.reader(stream))
    with (dataset / "ground_truth" / "target_pose_map.csv").open(newline="", encoding="utf-8") as stream:
        assert "map_R_target_00" in next(csv.reader(stream))
    gt = json.loads((scene / "scene_gt.json").read_text())
    predictions = {
        frame: [{"obj_id": item["obj_id"], "R": np.asarray(item["cam_R_m2c"]).reshape(3, 3).tolist(), "t_m": (np.asarray(item["cam_t_m2c"]) * 0.001).tolist()} for item in items]
        for frame, items in gt.items()
    }
    assert evaluate_bop_predictions(dataset, predictions)["availability"] == 1.0
    map_truth = {}
    with (dataset / "ground_truth" / "target_pose_map.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rotation = [[float(row[f"map_R_target_{r}{c}"]) for c in range(3)] for r in range(3)]
            map_truth.setdefault(row["sequence"], []).append({
                "obj_id": int(row["object_id"]), "R": rotation,
                "t_m": [float(row["target_x_m"]), float(row["target_y_m"]), float(row["target_z_m"])],
            })
    assert evaluate_map_target_predictions(dataset, map_truth)["translation_rmse_3d_m"] < 1e-12
