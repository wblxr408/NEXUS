"""Tests for the degradation injectors and the pose figure entry point."""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).parents[1]
REPO = Path(__file__).parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(REPO / "simulation"))

from evaluation.plot_pose_results import (CONTRACT_COLUMNS, TRUTH_COLUMNS, decompose_depth_lateral, load_camera_poses,
                                          load_pose_errors, plot_chain_box_comparison, plot_covariance_calibration,
                                          plot_depth_lateral_decomposition, plot_per_target_depth_bias,
                                          plot_runtime_frame, plot_transform_chain)
from inject_degradations import (DEGRADATION_SCENARIOS, background_clutter, exposure, image_motion_direction_deg,
                                 inject_split, motion_blur, occlusion, printed_patch, roi_shift, uwb_nlos)

PLAN_10_4_SCENARIOS = {"no_degradation", "occlusion_30", "occlusion_50", "occlusion_70", "exposure_over",
                       "exposure_under", "hard_shadow", "motion_blur", "background_clutter",
                       "uwb_nlos_positive_bias", "printed_patch", "roi_shift"}


def _image_and_mask(size: int = 96) -> tuple[np.ndarray, np.ndarray]:
    image = np.random.default_rng(11).integers(30, 220, size=(size, size, 3), dtype=np.uint8)
    mask = np.zeros((size, size), dtype=np.uint8)
    mask[30:70, 25:65] = 255
    return image, mask


def _tiny_dataset(tmp_path: Path) -> Path:
    config = yaml.safe_load((REPO / "simulation" / "target_detection_dataset_v01.yaml").read_text(encoding="utf-8"))
    config["camera"]["image_width_px"], config["camera"]["image_height_px"] = 320, 240
    config["camera"]["K"] = [[240.0, 0.0, 160.0], [0.0, 240.0, 120.0], [0.0, 0.0, 1.0]]
    config["camera"]["P"] = [[240.0, 0.0, 160.0, 0.0], [0.0, 240.0, 120.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
    config["splits"] = {"test": {"trajectory_ids": ["test_perimeter_arc"], "frames_per_trajectory": 2}}
    config["render"]["texture_landmark_count"] = 60
    config_path = tmp_path / "tiny_dataset.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    dataset = tmp_path / "dataset"
    subprocess.run([sys.executable, "simulation/generate_target_detection_dataset.py",
                    "--config", str(config_path), "--out", str(dataset)], check=True, cwd=REPO)
    return dataset


def _write_error_csv(path: Path, depth_errors_m: dict[int, float]) -> Path:
    header = list(CONTRACT_COLUMNS) + list(TRUTH_COLUMNS)
    lines = [",".join(header)]
    for frame in ("000000", "000001"):
        for obj_id, depth_error in depth_errors_m.items():
            for chain in ("A", "B", "C"):
                truth = np.array([1.0 + obj_id, 2.0, 0.20])
                estimate = truth + np.array([0.0, 0.0, depth_error])
                lines.append(",".join([frame, str(obj_id), f"class_{obj_id}", chain,
                                       f"{estimate[0]:.6f}", f"{estimate[1]:.6f}", f"{estimate[2]:.6f}",
                                       "0.030", "0.030", "0.060", "3", "0.42", "triangulation", "VALID", "", "7.4",
                                       f"{truth[0]:.6f}", f"{truth[1]:.6f}", f"{truth[2]:.6f}"]))
    lines.append(",".join(["000001", "9", "class_9", "A", "nan", "nan", "nan", "0.030", "0.030", "0.060",
                           "1", "0.00", "uwb_fallback", "INVALID", "transform_unavailable", "6.1",
                           "10.000000", "2.000000", "0.200000"]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_camera_csv(path: Path) -> Path:
    header = ["split", "trajectory_id", "sequence", "sample_timestamp_ns", "camera_x_m", "camera_y_m", "camera_z_m"]
    header += [f"R_map_camera_{r}{c}" for r in range(3) for c in range(3)]
    lines = [",".join(header)]
    for sequence in (0, 1):
        identity = ["1", "0", "0", "0", "1", "0", "0", "0", "1"]
        lines.append(",".join(["test", "test_perimeter_arc", str(sequence), str(1_780_000_000_000_000_000 + sequence),
                               "2.0", "2.35", "3.30"] + identity))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def test_occlusion_rectangle_reports_the_measured_fraction():
    image, mask = _image_and_mask()
    result, metadata = occlusion(image, mask, 0.5, rng=np.random.default_rng(3))
    covered = np.count_nonzero((mask > 0) & np.all(result == np.array([16, 16, 16]), axis=2))
    assert metadata["occluded_roi_px"] == covered
    assert 0.5 <= metadata["actual_fraction"] <= 0.6
    grid_y, grid_x = np.mgrid[0:96, 0:96]
    circle = np.where((grid_y - 48) ** 2 + (grid_x - 48) ** 2 <= 22 ** 2, 255, 0).astype(np.uint8)
    _, round_roi = occlusion(image, circle, 0.5, rng=np.random.default_rng(3))
    assert round_roi["actual_fraction"] != round_roi["target_fraction"]
    assert abs(round_roi["actual_fraction"] - 0.5) < 0.05


def test_occlusion_polygon_reaches_the_requested_fraction():
    image, mask = _image_and_mask()
    _, metadata = occlusion(image, mask, 0.7, kind="polygon", rng=np.random.default_rng(5))
    assert metadata["kind"] == "polygon"
    assert metadata["actual_fraction"] >= 0.7


def test_exposure_modes_move_brightness_in_the_declared_direction():
    image, mask = _image_and_mask()
    brighter, over = exposure(image, mask, mode="over", gamma=0.5, gain=1.7)
    darker, under = exposure(image, mask, mode="under", gamma=2.0, gain=0.32)
    shadow, hard = exposure(image, mask, mode="hard_shadow", shadow_strength=0.8, shadow_direction_deg=0.0)
    assert brighter.mean() > image.mean() > darker.mean()
    assert over["gamma"] == 0.5 and under["gain"] == 0.32
    assert shadow[:, :10].mean() > shadow[:, -10:].mean()
    assert hard["shadow_direction_deg"] == 0.0 and hard["roi_mean_level_after"] < hard["roi_mean_level_before"]


def test_motion_blur_follows_the_camera_motion_direction():
    image, mask = _image_and_mask()
    direction = image_motion_direction_deg([0.0, 0.0, 3.0], [0.4, 0.0, 3.0], np.eye(3))
    assert abs(abs(direction) - 180.0) < 1e-9
    result, metadata = motion_blur(image, mask, kernel_length_px=15, direction_deg=direction)
    assert metadata["kernel_length_px"] == 15 and metadata["kernel_nonzero_px"] > 1
    assert metadata["roi_sharpness_ratio"] < 1.0
    assert not np.array_equal(result, image)


def test_background_clutter_leaves_roi_pixels_untouched():
    image, mask = _image_and_mask()
    result, metadata = background_clutter(image, mask, count=12, size_px=(10, 25), rng=np.random.default_rng(7))
    assert np.array_equal(result[mask > 0], image[mask > 0])
    assert metadata["roi_px_changed"] == 0
    assert metadata["clutter_px_outside_roi"] > 0 and metadata["patch_count"] == 12


def test_uwb_nlos_biases_only_the_selected_anchors_upwards():
    ranges, metadata = uwb_nlos([3.0, 4.0, 5.0, 6.0], [0, 2], 0.35)
    assert np.allclose(ranges, [3.35, 4.0, 5.35, 6.0])
    assert metadata["affected_anchor_count"] == 2 and metadata["mean_bias_m"] == 0.35
    assert all(bias > 0.0 for bias in metadata["applied_bias_m"])
    try:
        uwb_nlos([3.0, 4.0], [0], -0.2)
    except ValueError:
        return
    raise AssertionError("a non-positive NLOS bias must be rejected")


def test_printed_patch_is_deterministic_and_stays_inside_the_roi():
    image, mask = _image_and_mask()
    first, metadata = printed_patch(image, mask, seed=99, blocks=6, coverage=0.4)
    second, _ = printed_patch(image, mask, seed=99, blocks=6, coverage=0.4)
    changed = np.any(first != image, axis=2)
    assert np.array_equal(first, second)
    assert not np.any(changed & (mask == 0))
    assert metadata["actual_roi_coverage"] > 0.0 and metadata["pattern_seed"] == 99
    assert metadata["optimization"].startswith("none")


def test_roi_shift_translates_the_mask_by_the_requested_pixels():
    _, mask = _image_and_mask()
    shifted, metadata = roi_shift(mask, [5, -4])
    before = np.argwhere(mask > 0).mean(axis=0)
    after = np.argwhere(shifted > 0).mean(axis=0)
    assert np.allclose(after - before, [-4.0, 5.0])
    assert metadata["shifted_roi_px"] == metadata["roi_px"]
    assert 0.0 < metadata["iou"] < 1.0 and metadata["retained_fraction"] < 1.0


def test_degradation_scenarios_match_the_plan_10_4_rows():
    assert set(DEGRADATION_SCENARIOS) == PLAN_10_4_SCENARIOS
    assert all(name == name.lower() and name.isascii() for name in DEGRADATION_SCENARIOS)
    injectors = {definition["injector"] for definition in DEGRADATION_SCENARIOS.values()}
    assert injectors == {"none", "occlusion", "exposure", "motion_blur", "background_clutter", "uwb_nlos",
                         "printed_patch", "roi_shift"}
    assert {definition["params"]["fraction"] for name, definition in DEGRADATION_SCENARIOS.items()
            if definition["injector"] == "occlusion"} == {0.30, 0.50, 0.70}


def test_inject_split_mirrors_the_dataset_layout(tmp_path):
    dataset = _tiny_dataset(tmp_path)
    occluded = inject_split(dataset, tmp_path / "occluded", "test", "occlusion_50", 7)
    scene = "test/000001"
    assert occluded["not_ground_truth"] is True and occluded["frame_count"] == 2
    assert sorted(path.name for path in (tmp_path / "occluded" / scene / "rgb").glob("*.png")) == ["000000.png", "000001.png"]
    assert len(list((tmp_path / "occluded" / scene / "mask").glob("*.png"))) == 20
    assert (tmp_path / "occluded" / scene / "scene_gt.json").is_file()
    assert (tmp_path / "occluded" / "ground_truth" / "target_pose_map.csv").is_file()
    assert (tmp_path / "occluded" / "models" / "models_info.json").is_file()
    fractions = [record["actual_fraction"] for record in occluded["frames"][0]["injections"]]
    assert fractions and all(fraction >= 0.5 for fraction in fractions)


def test_roi_shift_scenario_changes_masks_only_and_cli_refuses_to_overwrite(tmp_path):
    dataset = _tiny_dataset(tmp_path)
    output = tmp_path / "cli_out"
    subprocess.run([sys.executable, "simulation/inject_degradations.py", "--dataset", str(dataset),
                    "--scenario", "roi_shift", "--out", str(output), "--split", "test", "--seed", "5"],
                   check=True, cwd=REPO)
    scene = "test/000001"
    source_rgb = (dataset / scene / "rgb" / "000000.png").read_bytes()
    manifest = json.loads((output / "roi_shift" / "degradation_manifest.json").read_text(encoding="utf-8"))
    assert manifest["applies_to"] == "mask" and manifest["seed"] == 5
    assert (output / "roi_shift" / scene / "rgb" / "000000.png").read_bytes() == source_rgb
    assert (output / "roi_shift" / scene / "mask" / "000000_000000.png").read_bytes() != (dataset / scene / "mask" / "000000_000000.png").read_bytes()
    repeated = subprocess.run([sys.executable, "simulation/inject_degradations.py", "--dataset", str(dataset),
                               "--scenario", "roi_shift", "--out", str(output), "--split", "test"], cwd=REPO,
                              capture_output=True, text=True)
    assert repeated.returncode != 0 and "refusing to overwrite" in repeated.stderr


def test_load_pose_errors_reads_the_contract_and_flags_invalid_rows(tmp_path):
    table = load_pose_errors(_write_error_csv(tmp_path / "errors.csv", {1: -0.09, 2: -0.05}))
    assert table.frame_id.size == 13 and int(table.valid.sum()) == 12
    assert set(table.chain.tolist()) == {"A", "B", "C"}
    assert np.allclose(table.valid_rows().error_map_m[:, 2], [-0.09] * 3 + [-0.05] * 3 + [-0.09] * 3 + [-0.05] * 3)
    assert np.allclose(table.valid_rows().sigma_map_m[0], [0.030, 0.030, 0.060])
    assert "transform_unavailable" in table.degraded_reason.tolist()


def test_load_pose_errors_rejects_missing_columns_and_files(tmp_path):
    path = _write_error_csv(tmp_path / "errors.csv", {1: -0.09})
    lines = path.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")
    keep = [index for index, name in enumerate(header) if name != "sigma_z_m"]
    trimmed = tmp_path / "trimmed.csv"
    trimmed.write_text("\n".join(",".join(line.split(",")[index] for index in keep) for line in lines) + "\n", encoding="utf-8")
    try:
        load_pose_errors(trimmed)
    except ValueError as error:
        assert "sigma_z_m" in str(error)
    else:
        raise AssertionError("a missing contract column must raise")
    try:
        load_pose_errors(tmp_path / "absent.csv")
    except FileNotFoundError:
        return
    raise AssertionError("a missing CSV must raise")


def test_load_pose_errors_rejects_inconsistent_error_columns(tmp_path):
    path = _write_error_csv(tmp_path / "errors.csv", {1: -0.09})
    lines = path.read_text(encoding="utf-8").splitlines()
    lines[0] += ",error_x_m,error_y_m,error_z_m,translation_error_3d_m"
    lines[1:] = [f"{line},0.0,0.0,0.5,0.5" for line in lines[1:]]
    inconsistent = tmp_path / "inconsistent.csv"
    inconsistent.write_text("\n".join(lines) + "\n", encoding="utf-8")
    try:
        load_pose_errors(inconsistent)
    except ValueError as error:
        assert "disagree" in str(error)
        return
    raise AssertionError("stated error columns that contradict the truth columns must raise")


def test_camera_poses_expose_the_optical_axis_and_split_filter(tmp_path):
    poses = load_camera_poses(_write_camera_csv(tmp_path / "camera.csv"), "test")
    assert sorted(poses) == ["000000", "000001"]
    assert np.allclose(poses["000000"]["optical_axis_map"], [0.0, 0.0, 1.0])
    assert poses["000001"]["trajectory_id"] == "test_perimeter_arc"


def test_depth_lateral_decomposition_projects_onto_the_optical_axis(tmp_path):
    table = load_pose_errors(_write_error_csv(tmp_path / "errors.csv", {1: -0.09, 2: -0.05}))
    poses = load_camera_poses(_write_camera_csv(tmp_path / "camera.csv"), "test")
    decomposition = decompose_depth_lateral(table, poses)
    assert decomposition["sample_count"] == 12
    assert np.allclose(decomposition["lateral_m"], 0.0)
    assert decomposition["depth_mean_m"] < 0.0
    assert abs(decomposition["depth_rms_m"] - np.sqrt(np.mean(np.array([0.09, 0.05] * 6) ** 2))) < 1e-9
    assert abs(decomposition["depth_square_share"] - 1.0) < 1e-9


def test_all_six_figures_are_built_from_loaded_data(tmp_path):
    table = load_pose_errors(_write_error_csv(tmp_path / "errors.csv", {1: -0.09, 2: -0.05}))
    poses = load_camera_poses(_write_camera_csv(tmp_path / "camera.csv"), "test")
    decomposition = decompose_depth_lateral(table, poses)
    valid = table.valid_rows()
    figures = [
        plot_transform_chain(),
        plot_runtime_frame(np.zeros((48, 64, 3), dtype=np.uint8), np.array([[10.0, 12.0]]),
                           {"A": np.array([[11.0, 13.0]]), "B": np.array([[9.0, 11.0]]), "C": np.array([[12.0, 10.0]])}),
        plot_depth_lateral_decomposition(decomposition),
        plot_per_target_depth_bias({f"obj{obj:02d}": decomposition["depth_m"][decomposition["obj_id"] == obj] for obj in (1, 2)}),
        plot_chain_box_comparison({chain: valid.select(valid.chain == chain).translation_error_m for chain in ("A", "B", "C")}),
        plot_covariance_calibration(valid.sigma_map_m.reshape(-1), valid.error_map_m.reshape(-1)),
    ]
    assert all(figure.get_axes() for figure in figures)
    assert len(figures[2].get_axes()) == 2


def test_plot_cli_names_figures_and_skips_missing_inputs(tmp_path):
    errors = _write_error_csv(tmp_path / "errors.csv", {1: -0.09, 2: -0.05})
    camera = _write_camera_csv(tmp_path / "camera.csv")
    out_dir = tmp_path / "figures"
    completed = subprocess.run([sys.executable, "code/analysis/evaluation/plot_pose_results.py",
                                "--errors-csv", str(errors), "--camera-poses-csv", str(camera),
                                "--split", "test", "--out-dir", str(out_dir)],
                               check=True, cwd=REPO, capture_output=True, text=True)
    written = sorted(path.name for path in out_dir.glob("*.png"))
    assert written == ["fig_chain_translation_box_test_perimeter_arc_v01.png",
                       "fig_covariance_calibration_test_perimeter_arc_v01.png",
                       "fig_depth_lateral_decomposition_test_perimeter_arc_v01.png",
                       "fig_per_target_depth_bias_test_perimeter_arc_v01.png",
                       "fig_transform_chain_test_perimeter_arc_v01.png"]
    assert "skipped runtime_frame" in completed.stdout
