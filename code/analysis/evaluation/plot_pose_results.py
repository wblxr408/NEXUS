#!/usr/bin/env python3
"""Static evidence figures for the three localization chains (plan 11.3).

Six figures live here: the transform-chain schematic, the runtime frame
overlay, the depth/lateral error decomposition, the per-target depth bias, the
per-chain P50/P95 box comparison and the covariance-vs-error calibration.
Plan 11.3 points at ``plot_e012_pose_results.py`` and
``export_e008_simulation_evidence.py``; neither exists in this repository, so
this module is the single entry point instead.

Every number comes from caller-supplied CSV: the per-sample contract of plan
4.5 plus ``<x|y|z>_map_truth_m`` truth columns, and the per-frame camera poses
written by ``simulation/generate_target_detection_dataset.py``.  Nothing is
synthesized, no example data is embedded, and a missing input skips its figure
instead of drawing an empty one.  Metric names follow
``docs/算法比较定义参数.md``; the depth/lateral split uses the plan 10.2
definition (depth along the camera optical axis, lateral perpendicular to it).
Figure functions return a ``Figure`` and never write files.
"""
from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  backend must be selected first

CONTRACT_COLUMNS = ("frame_id", "obj_id", "class_name", "chain", "x_map_m", "y_map_m", "z_map_m",
                    "sigma_x_m", "sigma_y_m", "sigma_z_m", "n_views", "baseline_m", "depth_source",
                    "validity", "degraded_reason", "runtime_ms")
TRUTH_COLUMNS = ("x_map_truth_m", "y_map_truth_m", "z_map_truth_m")
ERROR_COLUMNS = ("error_x_m", "error_y_m", "error_z_m", "translation_error_3d_m")
VALID_TOKENS = {"1", "valid", "validity_valid", "true"}
CHAIN_COLORS = {"A": "#d62728", "B": "#1f77b4", "C": "#e08214"}


def _float(value: str) -> float:
    text = str(value).strip().lower()
    return float("nan") if text in {"", "nan", "none", "null"} else float(text)


def _chain_key(value: str) -> str:
    text = str(value).strip()
    return text[-1].upper() if text and text[-1].upper() in CHAIN_COLORS else text


@dataclass(frozen=True)
class PoseErrorTable:
    """Per-sample chain output plus map truth, one row per solved target."""

    frame_id: np.ndarray
    obj_id: np.ndarray
    class_name: np.ndarray
    chain: np.ndarray
    estimate_map_m: np.ndarray
    truth_map_m: np.ndarray
    sigma_map_m: np.ndarray
    n_views: np.ndarray
    baseline_m: np.ndarray
    depth_source: np.ndarray
    validity: np.ndarray
    degraded_reason: np.ndarray
    runtime_ms: np.ndarray
    valid: np.ndarray

    @property
    def error_map_m(self) -> np.ndarray:
        return self.estimate_map_m - self.truth_map_m

    @property
    def translation_error_m(self) -> np.ndarray:
        return np.linalg.norm(self.error_map_m, axis=1)

    def select(self, selector: np.ndarray) -> PoseErrorTable:
        return replace(self, **{field.name: getattr(self, field.name)[selector] for field in fields(self)})

    def valid_rows(self) -> PoseErrorTable:
        return self.select(self.valid)


def load_pose_errors(path: str | Path) -> PoseErrorTable:
    """Read the plan 4.5 per-sample CSV; missing columns are a hard error."""
    csv_path = Path(path)
    if not csv_path.is_file():
        raise FileNotFoundError(f"pose error CSV not found: {csv_path}")
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"pose error CSV has no data rows: {csv_path}")
    missing = [name for name in CONTRACT_COLUMNS + TRUTH_COLUMNS if name not in rows[0]]
    if missing:
        raise ValueError(f"pose error CSV {csv_path} is missing required columns: {missing}")
    estimate = np.array([[_float(row["x_map_m"]), _float(row["y_map_m"]), _float(row["z_map_m"])] for row in rows])
    truth = np.array([[_float(row[name]) for name in TRUTH_COLUMNS] for row in rows])
    validity = np.array([str(row["validity"]).strip() for row in rows])
    if all(name in rows[0] for name in ERROR_COLUMNS):
        stated = np.array([[_float(row["error_x_m"]), _float(row["error_y_m"]), _float(row["error_z_m"])] for row in rows])
        deviation = np.abs(np.nan_to_num(stated - (estimate - truth)))
        if float(deviation.max(initial=0.0)) > 1e-6:
            raise ValueError(f"error_<axis>_m columns disagree with estimate minus truth in {csv_path}")
    table = PoseErrorTable(
        frame_id=np.array([str(row["frame_id"]).strip() for row in rows]),
        obj_id=np.array([int(_float(row["obj_id"])) for row in rows]),
        class_name=np.array([str(row["class_name"]).strip() for row in rows]),
        chain=np.array([_chain_key(row["chain"]) for row in rows]),
        estimate_map_m=estimate, truth_map_m=truth,
        sigma_map_m=np.array([[_float(row["sigma_x_m"]), _float(row["sigma_y_m"]), _float(row["sigma_z_m"])] for row in rows]),
        n_views=np.array([_float(row["n_views"]) for row in rows]),
        baseline_m=np.array([_float(row["baseline_m"]) for row in rows]),
        depth_source=np.array([str(row["depth_source"]).strip() for row in rows]),
        validity=validity,
        degraded_reason=np.array([str(row["degraded_reason"]).strip() for row in rows]),
        runtime_ms=np.array([_float(row["runtime_ms"]) for row in rows]),
        valid=np.array([token.lower() in VALID_TOKENS for token in validity]) & np.all(np.isfinite(estimate), axis=1),
    )
    if not table.valid.any():
        raise ValueError(f"pose error CSV {csv_path} contains no valid rows to plot")
    return table


def load_camera_poses(path: str | Path, split: str | None = None) -> dict[str, dict[str, Any]]:
    """Read ``ground_truth/camera_pose_map.csv`` keyed by zero-padded sequence.

    The stored ``R_map_camera_<r><c>`` matrix maps ``map`` vectors into the
    camera frame (the header follows the dataset, the content is
    ``R_camera_map``), so its third row is the optical axis expressed in ``map``.
    """
    csv_path = Path(path)
    if not csv_path.is_file():
        raise FileNotFoundError(f"camera pose CSV not found: {csv_path}")
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = ["split", "trajectory_id", "sequence", "camera_x_m", "camera_y_m", "camera_z_m"]
    required += [f"R_map_camera_{r}{c}" for r in range(3) for c in range(3)]
    missing = [name for name in required if not rows or name not in rows[0]]
    if missing:
        raise ValueError(f"camera pose CSV {csv_path} is missing required columns: {missing}")
    poses: dict[str, dict[str, Any]] = {}
    for row in rows:
        if split is not None and row["split"] != split:
            continue
        rotation = np.array([_float(row[f"R_map_camera_{r}{c}"]) for r in range(3) for c in range(3)]).reshape(3, 3)
        poses[f"{int(row['sequence']):06d}"] = {
            "position_map_m": np.array([_float(row["camera_x_m"]), _float(row["camera_y_m"]), _float(row["camera_z_m"])]),
            "rotation_camera_from_map": rotation, "optical_axis_map": rotation[2] / np.linalg.norm(rotation[2]),
            "trajectory_id": row["trajectory_id"], "split": row["split"]}
    if not poses:
        raise ValueError(f"camera pose CSV {csv_path} has no rows for split={split}")
    return poses


def _pose_for_frame(poses: dict[str, dict[str, Any]], frame_id: str) -> dict[str, Any]:
    key = f"{int(float(frame_id)):06d}" if str(frame_id).strip().lstrip("-").replace(".", "", 1).isdigit() else str(frame_id).strip()
    if key not in poses:
        raise KeyError(f"frame {frame_id} has no camera pose row")
    return poses[key]


def decompose_depth_lateral(table: PoseErrorTable, poses: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """Split each translation error into optical-axis depth and lateral parts.

    ``depth_m`` is signed along the camera optical axis, so a positive value
    means the estimate sits further from the camera than the truth.
    """
    valid = table.valid_rows()
    axes = np.array([_pose_for_frame(poses, frame)["optical_axis_map"] for frame in valid.frame_id])
    error = valid.error_map_m
    depth = np.sum(error * axes, axis=1)
    lateral = np.linalg.norm(error - depth[:, None] * axes, axis=1)
    return {"depth_m": depth, "lateral_m": lateral, "obj_id": valid.obj_id, "class_name": valid.class_name,
            "chain": valid.chain, "depth_rms_m": float(np.sqrt(np.mean(depth ** 2))),
            "lateral_rms_m": float(np.sqrt(np.mean(lateral ** 2))), "depth_mean_m": float(np.mean(depth)),
            "depth_square_share": float(np.sum(depth ** 2) / np.sum(error ** 2)), "sample_count": int(depth.size)}


def project_map_points(points_map_m: np.ndarray, pose: dict[str, Any], camera_matrix: np.ndarray) -> np.ndarray:
    """Project ``map`` points into pixels for one camera pose (pinhole, no distortion)."""
    points_camera = (np.asarray(pose["rotation_camera_from_map"], dtype=float) @ (np.asarray(points_map_m, dtype=float) - pose["position_map_m"]).T).T
    homogeneous = points_camera @ np.asarray(camera_matrix, dtype=float).T
    return homogeneous[:, :2] / homogeneous[:, 2:3]


def plot_transform_chain() -> plt.Figure:
    """Schematic of ``map -> camera -> target`` and the composed transform."""
    figure, axes = plt.subplots(figsize=(9.0, 3.4))
    axes.set_axis_off()
    boxes = (("map", 0.08), ("camera", 0.45), ("target_link", 0.82))
    for label, x in boxes:
        axes.add_patch(plt.Rectangle((x - 0.07, 0.55), 0.14, 0.16, facecolor="#eef2f7", edgecolor="#40566e", linewidth=1.4))
        axes.text(x, 0.63, label, ha="center", va="center", fontsize=11)
    for (start, end, label) in ((0.15, 0.38, "T_map_camera"), (0.52, 0.75, "T_camera_target")):
        axes.annotate("", xy=(end, 0.63), xytext=(start, 0.63), arrowprops={"arrowstyle": "-|>", "color": "#40566e", "linewidth": 1.6})
        axes.text((start + end) / 2.0, 0.68, label, ha="center", va="bottom", fontsize=10)
    axes.annotate("", xy=(0.75, 0.34), xytext=(0.15, 0.34), arrowprops={"arrowstyle": "-|>", "color": "#b3541e", "linewidth": 1.8, "linestyle": "--"})
    axes.text(0.45, 0.24, "T_map_target = T_map_camera x T_camera_target", ha="center", fontsize=12, color="#b3541e")
    axes.text(0.45, 0.10, "translation in metres, rotation SO(3); map is the sandbox frame of simulation/target_catalog_v01.yaml",
              ha="center", fontsize=8.5, color="#4d4d4d")
    axes.set_xlim(0.0, 1.0)
    axes.set_ylim(0.0, 0.85)
    axes.set_title("Transform chain used by every localization chain")
    figure.tight_layout()
    return figure


def plot_runtime_frame(image: np.ndarray, truth_centers_px: np.ndarray, chain_estimates_px: dict[str, np.ndarray]) -> plt.Figure:
    """RGB frame with green truth centres and one cross colour per chain."""
    figure, axes = plt.subplots(figsize=(9.0, 6.5))
    axes.imshow(np.asarray(image))
    truth = np.asarray(truth_centers_px, dtype=float).reshape(-1, 2)
    axes.scatter(truth[:, 0], truth[:, 1], s=170, facecolors="none", edgecolors="#20c020", linewidths=1.8, label="truth centre")
    for chain, centers in chain_estimates_px.items():
        pixels = np.asarray(centers, dtype=float).reshape(-1, 2)
        axes.scatter(pixels[:, 0], pixels[:, 1], marker="x", s=90, linewidths=1.8,
                     color=CHAIN_COLORS.get(chain, "#7f7f7f"), label=f"chain {chain} estimate")
    axes.set_xlim(0, np.asarray(image).shape[1])
    axes.set_ylim(np.asarray(image).shape[0], 0)
    axes.set_xlabel("u [px]")
    axes.set_ylabel("v [px]")
    axes.set_title("Runtime frame overlay: truth vs three-chain estimates")
    axes.legend(loc="upper right", framealpha=0.85)
    figure.tight_layout()
    return figure


def plot_depth_lateral_decomposition(errors: dict[str, Any]) -> plt.Figure:
    """Depth (optical axis) and lateral error histograms with their RMS (plan 10.2)."""
    depth_mm = np.asarray(errors["depth_m"], dtype=float) * 1000.0
    lateral_mm = np.asarray(errors["lateral_m"], dtype=float) * 1000.0
    figure, axes = plt.subplots(2, 1, figsize=(8.0, 6.4))
    axes[0].hist(depth_mm, bins=40, color="#1f77b4", alpha=0.85)
    axes[0].axvline(0.0, color="#333333", linewidth=1.0)
    axes[0].axvline(depth_mm.mean(), color="#b3541e", linestyle="--", linewidth=1.4, label=f"mean {depth_mm.mean():.1f} mm")
    axes[0].set_title(f"Depth error along the optical axis: RMS {np.sqrt(np.mean(depth_mm ** 2)):.1f} mm (n={depth_mm.size})")
    axes[0].set_xlabel("signed depth error [mm], positive = estimate further along the optical axis")
    axes[0].legend()
    axes[1].hist(lateral_mm, bins=40, color="#2ca02c", alpha=0.85)
    axes[1].axvline(np.sqrt(np.mean(lateral_mm ** 2)), color="#b3541e", linestyle="--", linewidth=1.4,
                    label=f"RMS {np.sqrt(np.mean(lateral_mm ** 2)):.1f} mm")
    axes[1].set_title(f"Lateral error perpendicular to the optical axis; depth share of squared error "
                      f"{100.0 * np.sum(depth_mm ** 2) / np.sum(depth_mm ** 2 + lateral_mm ** 2):.1f}%")
    axes[1].set_xlabel("lateral error magnitude [mm]")
    for axis in axes:
        axis.set_ylabel("samples")
    figure.tight_layout()
    return figure


def plot_per_target_depth_bias(per_target: dict[str, np.ndarray]) -> plt.Figure:
    """Signed mean depth bias per target, with the zero line drawn."""
    labels = list(per_target)
    means = np.array([float(np.mean(np.asarray(per_target[label], dtype=float))) for label in labels]) * 1000.0
    spreads = np.array([float(np.std(np.asarray(per_target[label], dtype=float))) for label in labels]) * 1000.0
    figure, axes = plt.subplots(figsize=(max(7.0, 0.85 * len(labels) + 3.0), 4.6))
    colors = ["#1f77b4" if value < 0.0 else "#d62728" for value in means]
    axes.bar(np.arange(len(labels)), means, yerr=spreads, capsize=3, color=colors, alpha=0.9)
    axes.axhline(0.0, color="#333333", linewidth=1.2)
    axes.set_xticks(np.arange(len(labels)))
    axes.set_xticklabels(labels, rotation=45, ha="right", fontsize=8.5)
    axes.set_ylabel("mean depth bias [mm]")
    negative = int(np.sum(means < 0.0))
    axes.set_title(f"Per-target depth bias: {negative}/{len(labels)} targets biased negative (bars: mean, whiskers: 1 std)")
    figure.tight_layout()
    return figure


def plot_chain_box_comparison(per_chain_errors: dict[str, np.ndarray]) -> plt.Figure:
    """Box comparison of translation error per chain with P50/P95 annotated."""
    chains = list(per_chain_errors)
    values = [np.asarray(per_chain_errors[chain], dtype=float) for chain in chains]
    figure, axes = plt.subplots(figsize=(max(6.0, 2.2 * len(chains) + 2.0), 5.0))
    boxes = axes.boxplot(values, labels=[f"chain {chain}" for chain in chains], whis=(5, 95), showfliers=False, patch_artist=True)
    for patch, chain in zip(boxes["boxes"], chains):
        patch.set_facecolor(CHAIN_COLORS.get(chain, "#7f7f7f"))
        patch.set_alpha(0.45)
    for index, sample in enumerate(values, start=1):
        p50, p95 = float(np.percentile(sample, 50)), float(np.percentile(sample, 95))
        axes.plot([index], [p50], marker="o", color="#333333", markersize=6)
        axes.plot([index], [p95], marker="^", color="#b3541e", markersize=7)
        axes.annotate(f"P50 {p50:.4f} m", xy=(index, p50), xytext=(8, -12), textcoords="offset points", fontsize=8.5)
        axes.annotate(f"P95 {p95:.4f} m", xy=(index, p95), xytext=(8, 6), textcoords="offset points", fontsize=8.5, color="#b3541e")
    axes.set_ylabel("translation_error_3d_m")
    axes.set_title("Three-chain translation error: box (5-95% whiskers), circle P50, triangle P95")
    figure.tight_layout()
    return figure


def plot_covariance_calibration(sigma: np.ndarray, error: np.ndarray) -> plt.Figure:
    """Predicted one-sigma vs measured error with chi-square consistency."""
    predicted = np.asarray(sigma, dtype=float).reshape(-1)
    measured = np.abs(np.asarray(error, dtype=float).reshape(-1))
    if predicted.size != measured.size:
        raise ValueError("sigma and error must have the same length")
    finite = np.isfinite(predicted) & np.isfinite(measured) & (predicted > 0.0)
    predicted, measured = predicted[finite], measured[finite]
    if predicted.size == 0:
        raise ValueError("covariance calibration needs positive finite sigma values")
    normalized = measured ** 2 / predicted ** 2
    figure, axes = plt.subplots(figsize=(6.6, 6.0))
    axes.scatter(predicted, measured, s=14, alpha=0.55, color="#1f77b4", label="samples")
    limit = float(max(predicted.max(), measured.max())) * 1.05
    grid = np.array([0.0, limit])
    for factor, style, color in ((1.0, "-", "#333333"), (2.0, "--", "#b3541e"), (3.0, ":", "#7f3f9f")):
        axes.plot(grid, factor * grid, style, color=color, linewidth=1.4, label=f"error = {factor:.0f} sigma")
    axes.set_xlim(0.0, limit)
    axes.set_ylim(0.0, limit)
    axes.set_xlabel("predicted 1-sigma [m]")
    axes.set_ylabel("measured absolute error [m]")
    inside = [float(np.mean(measured <= factor * predicted)) for factor in (1.0, 2.0, 3.0)]
    axes.set_title(f"Covariance calibration: mean(e^2/sigma^2) = {np.mean(normalized):.2f} (ideal 1.0)\n"
                   f"within 1/2/3 sigma: {inside[0]:.1%} / {inside[1]:.1%} / {inside[2]:.1%}, n={predicted.size}")
    axes.legend(loc="upper left")
    figure.tight_layout()
    return figure


def _slug(text: str) -> str:
    keep = [character if character.isalnum() else "_" for character in str(text).strip().lower()]
    return "".join(keep).strip("_").replace("__", "_") or "unspecified"


def _condition(explicit: str | None, table: PoseErrorTable, poses: dict[str, dict[str, Any]] | None, errors_csv: Path) -> str:
    if explicit:
        return _slug(explicit)
    if poses:
        trajectories = sorted({poses[key]["trajectory_id"] for key in poses})
        if trajectories:
            return _slug("_".join(trajectories))
    return _slug(errors_csv.stem)


def _camera_matrix(path: str | Path) -> np.ndarray:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    matrix = payload["K"] if isinstance(payload, dict) else payload
    return np.asarray(matrix, dtype=float).reshape(3, 3)


def _runtime_frame_figure(table: PoseErrorTable, poses: dict[str, dict[str, Any]], image_path: Path,
                          camera_matrix: np.ndarray, frame_id: str) -> plt.Figure:
    valid = table.valid_rows()
    selector = valid.frame_id == frame_id
    if not selector.any():
        raise KeyError(f"no valid rows for frame {frame_id}")
    frame = valid.select(selector)
    pose = _pose_for_frame(poses, frame_id)
    estimates = {chain: project_map_points(frame.select(frame.chain == chain).estimate_map_m, pose, camera_matrix)
                 for chain in sorted(set(frame.chain.tolist()))}
    return plot_runtime_frame(plt.imread(str(image_path)), project_map_points(frame.truth_map_m, pose, camera_matrix), estimates)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--errors-csv", required=True, help="per-sample CSV following plan 4.5 plus <axis>_map_truth_m columns")
    parser.add_argument("--camera-poses-csv", help="dataset ground_truth/camera_pose_map.csv, required for the depth/lateral figures")
    parser.add_argument("--frame-image", help="one RGB frame for the runtime overlay")
    parser.add_argument("--frame-id", help="frame_id of --frame-image; defaults to its file stem")
    parser.add_argument("--camera-info-json", help="calibration/camera_info.json (or a bare 3x3 K) for the runtime overlay")
    parser.add_argument("--split", help="restrict camera poses to one dataset split")
    parser.add_argument("--out-dir", default="outputs/figures")
    parser.add_argument("--condition", help="condition token for the figure file names")
    parser.add_argument("--version", default="01")
    args = parser.parse_args()

    errors_csv = Path(args.errors_csv)
    table = load_pose_errors(errors_csv)
    poses = load_camera_poses(args.camera_poses_csv, args.split) if args.camera_poses_csv else None
    condition = _condition(args.condition, table, poses, errors_csv)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    valid = table.valid_rows()

    figures: list[tuple[str, plt.Figure]] = [("transform_chain", plot_transform_chain())]
    skipped: list[str] = []
    if args.frame_image and args.camera_info_json and poses:
        frame_id = args.frame_id or Path(args.frame_image).stem
        try:
            figures.append(("runtime_frame", _runtime_frame_figure(table, poses, Path(args.frame_image), _camera_matrix(args.camera_info_json), frame_id)))
        except KeyError as error:
            skipped.append(f"runtime_frame: {error}")
    else:
        skipped.append("runtime_frame: needs --frame-image, --camera-info-json and --camera-poses-csv")
    if poses:
        decomposition = decompose_depth_lateral(table, poses)
        figures.append(("depth_lateral_decomposition", plot_depth_lateral_decomposition(decomposition)))
        pairs = sorted({(int(obj_id), str(name)) for obj_id, name in zip(decomposition["obj_id"], decomposition["class_name"])})
        per_target = {f"obj{obj_id:02d}_{name}": decomposition["depth_m"][decomposition["obj_id"] == obj_id] for obj_id, name in pairs}
        figures.append(("per_target_depth_bias", plot_per_target_depth_bias(per_target)))
    else:
        skipped.append("depth_lateral_decomposition and per_target_depth_bias: need --camera-poses-csv for the optical axis")
    per_chain = {chain: valid.select(valid.chain == chain).translation_error_m for chain in sorted(set(valid.chain.tolist()))}
    figures.append(("chain_translation_box", plot_chain_box_comparison(per_chain)))
    sigma, error = valid.sigma_map_m.reshape(-1), valid.error_map_m.reshape(-1)
    if np.any(np.isfinite(sigma) & (sigma > 0.0)):
        figures.append(("covariance_calibration", plot_covariance_calibration(sigma, error)))
    else:
        skipped.append("covariance_calibration: no positive finite sigma_<axis>_m values in the CSV")

    for topic, figure in figures:
        path = out_dir / f"fig_{topic}_{condition}_v{args.version}.png"
        figure.savefig(path, dpi=150)
        plt.close(figure)
        print(f"wrote {path}")
    for reason in skipped:
        print(f"skipped {reason}")


if __name__ == "__main__":
    main()

