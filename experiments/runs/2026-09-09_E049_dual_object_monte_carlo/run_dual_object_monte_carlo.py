#!/usr/bin/env python3
"""Generate a reproducible, measurement-parameterized dual-object MC study.

This script is deliberately a simulation artifact, not a replacement for a
surveyed ground-truth trial.  It keeps the E042/E043/E048 values used below in
the accompanying figure specification and writes all simulated observations.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from scipy.optimize import least_squares


ROOT = Path(__file__).resolve().parent
SEED = 20260909
N_REPLICATES = 500
COLORS = {"blue": "#1463D9", "orange": "#F2541B", "green": "#14806B", "gray": "#4B5563"}

# E042 platform_experimental.yaml.  The E042 initial pose is a replay seed,
# explicitly not surveyed ground truth; it is therefore the nominal state for
# noise propagation only.
ANCHORS_M = np.array([
    [0.9, -0.9, 2.56],
    [0.0, 5.0, 2.71],
    [4.0, 5.0, 2.71],
    [4.0, 0.0, 2.71],
])
UAV_NOMINAL_M = np.array([0.9848675742439982, 3.7889502056955906, 0.13])

# E043 recorded_data_replay.json, post-MAD-filter standard deviations in m.
UWB_SIGMA_M = np.array([
    0.01706652969956575,
    0.022334966378763515,
    0.013109694714235577,
    0.022607441667132116,
])

# E048 candidate J06-NE-HIGH coordinate.  Its alignment to E047_live_map is
# unverified; it is used only as a candidate reference for sensitivity plots.
TARGET_CANDIDATE_M = np.array([0.9062, 4.855914893617021, 0.22])
TARGET_OBSERVED_M = np.array([
    [0.7712899131644295, 4.68444468716065, 0.22],
    [0.7590772272743538, 4.755000288734263, 0.22],
    [0.8413074700113858, 4.725441551617223, 0.22],
])


def horizontal_error(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.linalg.norm(points[:, :2] - reference[:2], axis=1)


def ecdf(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ordered = np.sort(values)
    return ordered, np.arange(1, len(ordered) + 1) / len(ordered)


def p95(values: np.ndarray) -> float:
    return float(np.quantile(values, 0.95))


def weighted_multilateration(ranges_m: np.ndarray) -> np.ndarray:
    weights = 1.0 / UWB_SIGMA_M
    initial = ANCHORS_M[:, :2].mean(axis=0)
    solved = least_squares(
        lambda xy: weights * (np.linalg.norm(ANCHORS_M - np.r_[xy, UAV_NOMINAL_M[2]], axis=1) - ranges_m),
        initial,
        method="trf",
    )
    return np.r_[solved.x, UAV_NOMINAL_M[2]]


def covariance_ellipse(ax, samples_xy: np.ndarray, color: str, label: str) -> None:
    mean = samples_xy.mean(axis=0)
    cov = np.cov(samples_xy, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = eigenvalues.argsort()[::-1]
    eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    # sqrt(chi2_2,0.95) = 2.4477; full width/height require the factor two.
    diameter = 2 * 2.4477 * np.sqrt(np.maximum(eigenvalues, 0))
    ellipse = Ellipse(
        mean, diameter[0], diameter[1],
        angle=angle, facecolor="none", edgecolor=color, linewidth=2, label=label,
    )
    ax.add_patch(ellipse)


def save_ecdf(path: Path, series: list[tuple[np.ndarray, str, str]], title: str, note: str) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 4.7), constrained_layout=True)
    for values, label, color in series:
        x, y = ecdf(values)
        ax.step(x, y, where="post", color=color, linewidth=2.2, label=f"{label} (P95={p95(values):.3f} m)")
    ax.set(xlabel="Horizontal error (m)", ylabel="Empirical cumulative probability", title=title, ylim=(0, 1.02))
    ax.grid(alpha=.24)
    ax.legend(loc="lower right", frameon=True)
    ax.text(.02, .04, note, transform=ax.transAxes, fontsize=8.7, color=COLORS["gray"], va="bottom")
    fig.savefig(path, dpi=320, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    rng = np.random.default_rng(SEED)
    nominal_ranges = np.linalg.norm(ANCHORS_M - UAV_NOMINAL_M, axis=1)
    simulated_ranges = nominal_ranges + rng.normal(0, UWB_SIGMA_M, size=(N_REPLICATES, len(ANCHORS_M)))
    uav_estimates = np.array([weighted_multilateration(ranges) for ranges in simulated_ranges])
    uav_error_m = horizontal_error(uav_estimates, UAV_NOMINAL_M)

    observed_residuals = TARGET_OBSERVED_M[:, :2] - TARGET_CANDIDATE_M[:2]
    target_bias_xy = observed_residuals.mean(axis=0)
    target_cov_xy = np.cov(observed_residuals, rowvar=False, ddof=1)
    vision_residual_xy = rng.multivariate_normal(target_bias_xy, target_cov_xy, size=N_REPLICATES)
    propagated_uav_xy = uav_estimates[:, :2] - UAV_NOMINAL_M[:2]
    target_raw = np.column_stack((TARGET_CANDIDATE_M[:2] + vision_residual_xy + propagated_uav_xy,
                                  np.full(N_REPLICATES, TARGET_CANDIDATE_M[2])))
    target_bias_corrected = target_raw.copy()
    target_bias_corrected[:, :2] -= target_bias_xy
    target_raw_error_m = horizontal_error(target_raw, TARGET_CANDIDATE_M)
    target_corrected_error_m = horizontal_error(target_bias_corrected, TARGET_CANDIDATE_M)

    with (ROOT / "tbl_dual_object_monte_carlo_v01.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            "replicate", "uav_estimate_x_m", "uav_estimate_y_m", "uav_error_xy_m",
            "target_raw_x_m", "target_raw_y_m", "target_raw_error_xy_m",
            "target_bias_corrected_x_m", "target_bias_corrected_y_m", "target_bias_corrected_error_xy_m",
        ])
        writer.writeheader()
        for index in range(N_REPLICATES):
            writer.writerow({
                "replicate": index + 1,
                "uav_estimate_x_m": uav_estimates[index, 0], "uav_estimate_y_m": uav_estimates[index, 1],
                "uav_error_xy_m": uav_error_m[index],
                "target_raw_x_m": target_raw[index, 0], "target_raw_y_m": target_raw[index, 1],
                "target_raw_error_xy_m": target_raw_error_m[index],
                "target_bias_corrected_x_m": target_bias_corrected[index, 0],
                "target_bias_corrected_y_m": target_bias_corrected[index, 1],
                "target_bias_corrected_error_xy_m": target_corrected_error_m[index],
            })

    save_ecdf(
        ROOT / "fig_uav_self_error_ecdf_v01.png",
        [(uav_error_m, "UAV weighted multilateration", COLORS["blue"])],
        "UAV self-localization: 500 measurement-parameterized simulations",
        "E042 replay seed only.\nNot surveyed ground truth.",
    )
    save_ecdf(
        ROOT / "fig_target_error_ecdf_v01.png",
        [(target_raw_error_m, "Raw target coordinate", COLORS["orange"]),
         (target_corrected_error_m, "Candidate-reference bias corrected", COLORS["green"])],
        "Sandbox target localization: 500 measurement-parameterized simulations",
        "Candidate reference frame unverified.\nSensitivity study only.",
    )

    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.6), constrained_layout=True)
    axes[0].scatter(uav_estimates[:, 0], uav_estimates[:, 1], s=9, alpha=.28, color=COLORS["blue"], label="500 simulated estimates")
    axes[0].scatter(*UAV_NOMINAL_M[:2], marker="*", s=130, color="black", label="E042 nominal seed")
    covariance_ellipse(axes[0], uav_estimates[:, :2], COLORS["blue"], "95% covariance ellipse")
    axes[0].set(title="UAV self-localization", xlabel="Map X (m)", ylabel="Map Y (m)")
    axes[0].axis("equal"); axes[0].grid(alpha=.24); axes[0].legend(fontsize=8, loc="best")
    axes[1].scatter(target_bias_corrected[:, 0], target_bias_corrected[:, 1], s=9, alpha=.25, color=COLORS["green"], label="500 bias-corrected simulations")
    axes[1].scatter(TARGET_OBSERVED_M[:, 0], TARGET_OBSERVED_M[:, 1], marker="x", s=54, linewidths=1.8, color=COLORS["orange"], label="3 E048 observed estimates")
    axes[1].scatter(*TARGET_CANDIDATE_M[:2], marker="*", s=130, color="black", label="candidate reference")
    covariance_ellipse(axes[1], target_bias_corrected[:, :2], COLORS["green"], "95% covariance ellipse")
    axes[1].set(title="Sandbox target localization", xlabel="Map X (m)", ylabel="Map Y (m)")
    axes[1].axis("equal"); axes[1].grid(alpha=.24); axes[1].legend(fontsize=8, loc="best")
    fig.suptitle("Dual-object XY uncertainty footprint", fontsize=13)
    fig.savefig(ROOT / "fig_dual_object_xy_uncertainty_v01.png", dpi=320, bbox_inches="tight")
    plt.close(fig)

    multipliers = np.array([0.5, 0.75, 1.0, 1.5, 2.0, 3.0])
    # Reuse the same standardized random draws at all levels, isolating the
    # parameter-scale effect rather than adding Monte Carlo seed variation.
    range_z = (simulated_ranges - nominal_ranges) / UWB_SIGMA_M
    centered_vision = vision_residual_xy - target_bias_xy
    uav_p95, target_p95 = [], []
    for scale in multipliers:
        estimates = np.array([weighted_multilateration(nominal_ranges + scale * UWB_SIGMA_M * row) for row in range_z])
        current_uav_error = horizontal_error(estimates, UAV_NOMINAL_M)
        corrected_xy = TARGET_CANDIDATE_M[:2] + scale * centered_vision + (estimates[:, :2] - UAV_NOMINAL_M[:2])
        uav_p95.append(p95(current_uav_error))
        target_p95.append(p95(np.linalg.norm(corrected_xy - TARGET_CANDIDATE_M[:2], axis=1)))
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.25), constrained_layout=True, sharex=True)
    axes[0].plot(multipliers, uav_p95, marker="o", linewidth=2.2, color=COLORS["blue"])
    axes[0].set(title="UAV: filtered-UWB scale", xlabel="Recorded UWB noise multiplier", ylabel="P95 horizontal error (m)")
    axes[1].plot(multipliers, target_p95, marker="o", linewidth=2.2, color=COLORS["green"])
    axes[1].set(title="Target: vision-residual scale", xlabel="E048 residual multiplier", ylabel="P95 horizontal error (m)")
    for axis in axes:
        axis.axvline(1, linestyle="--", color=COLORS["gray"], linewidth=1)
        axis.grid(alpha=.24)
    fig.suptitle("Sensitivity using the same 500 seeded draws at every multiplier", fontsize=13)
    fig.savefig(ROOT / "fig_noise_sensitivity_v01.png", dpi=320, bbox_inches="tight")
    plt.close(fig)

    metrics = {
        "status": "COMPUTED_APPROXIMATE_NOT_LIVE_VALIDATED",
        "replicates": N_REPLICATES,
        "seed": SEED,
        "uav_self_horizontal_error_m": {"median": float(np.median(uav_error_m)), "p95": p95(uav_error_m), "rmse": float(np.sqrt(np.mean(uav_error_m ** 2)) )},
        "target_raw_horizontal_error_m": {"median": float(np.median(target_raw_error_m)), "p95": p95(target_raw_error_m), "rmse": float(np.sqrt(np.mean(target_raw_error_m ** 2)) )},
        "target_bias_corrected_horizontal_error_m": {"median": float(np.median(target_corrected_error_m)), "p95": p95(target_corrected_error_m), "rmse": float(np.sqrt(np.mean(target_corrected_error_m ** 2)) )},
        "target_candidate_reference_offset_observed_m": {"mean_xy": target_bias_xy.tolist(), "mean_norm": float(np.linalg.norm(target_bias_xy)), "frames": 3},
        "limitations": [
            "UAV nominal pose is an E042 replay seed, not surveyed truth; its error distribution is conditional simulation output.",
            "The four UWB standard deviations are post-MAD-filter estimates from one static 180-s recording.",
            "Target residual covariance is derived from only three E048 frames.",
            "E048 states that E030 physical-sandbox coordinates are not proven aligned with E047_live_map; target error figures are sensitivity studies, not validated accuracy claims.",
            "No simulated result is presented as a physical repeat measurement."
        ]
    }
    (ROOT / "metrics_v01.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (ROOT / "render_report.json").write_text(json.dumps({
        "data_rows": N_REPLICATES,
        "figures": [path.name for path in sorted(ROOT.glob("fig_*_v01.png"))],
        "dpi": 320,
        "checks_pending": ["image-level visual inspection", "visual_qa.py"],
        "status": "generated_pending_quality_checks"
    }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
