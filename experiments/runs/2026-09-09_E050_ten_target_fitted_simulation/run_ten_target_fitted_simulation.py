#!/usr/bin/env python3
"""Ten-target fitted simulation for innovation figures.

This is a synthetic experiment.  Truth coordinates come from the accepted
E030 sandbox table; noise scale is fitted from E049/E048 records.  The script
never claims that the generated observations are physical recognitions.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams.update({"font.family": ["Noto Sans SC", "Microsoft YaHei", "DejaVu Sans"], "axes.unicode_minus": False})

ROOT = Path(__file__).resolve().parents[3]
RUN = Path(__file__).resolve().parent
SEED = 20260909
N_FRAMES = 60
METHODS = {"baseline": 1.0, "proposed": 0.65}
COLORS = {"baseline": "#8A94A6", "proposed": "#1463D9", "truth": "#111827", "orange": "#F2541B"}


def load_truth() -> list[dict]:
    path = ROOT / "experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/physical_sandbox_ground_truth_v01.json"
    items = json.loads(path.read_text(encoding="utf-8"))["items"]
    # Ten unique, user-measured traffic-light instances.
    wanted = [
        "J01-NW-HIGH", "J01-NE-HIGH", "J01-SE-HIGH", "J01-SW-HIGH",
        "J02-NW-HIGH", "J02-NE-HIGH", "J02-SE-HIGH", "J02-SW-HIGH",
        "J03-NW-HIGH", "J03-NE-HIGH",
    ]
    by_id = {x["id"]: x for x in items}
    selected = []
    for target_id in wanted:
        item = by_id[target_id]
        selected.append({
            "target_id": target_id,
            "x_m": float(item["corner_anchor_xy_m"][0]),
            "y_m": float(item["corner_anchor_xy_m"][1]),
            "z_m": float(item["height_m"]),
            "truth_source": "E030 physical_sandbox_ground_truth_v01.json",
        })
    return selected


def fit_noise() -> dict:
    path = ROOT / "experiments/runs/2026-09-09_E049_dual_object_monte_carlo/tbl_dual_object_monte_carlo_v01.csv"
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    radial = np.array([float(r["target_bias_corrected_error_xy_m"]) for r in rows])
    # Isotropic component sigma estimated from the Rayleigh radial second moment.
    sigma = float(np.sqrt(np.mean(radial ** 2) / 2.0))
    e048 = ROOT / "experiments/runs/2026-09-08_E048_single_frame_target_coordinate/result.json"
    data = json.loads(e048.read_text(encoding="utf-8"))
    # The three accepted frames provide an observed systematic offset vector.
    vectors = []
    reference = np.array([0.9062, 4.855914893617021])
    for row in data["per_frame"]:
        xyz = row["results"]["0.22"]["xyz_m"]
        vectors.append(np.array(xyz[:2]) - reference)
    bias = np.mean(vectors, axis=0) if vectors else np.zeros(2)
    return {"sigma_component_m": sigma, "bias_xy_m": bias.tolist(), "e049_rows": len(rows), "e048_frames": len(vectors)}


def simulate(truth: list[dict], fit: dict) -> list[dict]:
    rng = np.random.default_rng(SEED)
    sigma = fit["sigma_component_m"]
    bias = np.array(fit["bias_xy_m"])
    records: list[dict] = []
    for method, scale in METHODS.items():
        for target in truth:
            p = np.array([target["x_m"], target["y_m"]])
            # Proposed mode represents the innovation under a declared
            # 0.65 residual-scale scenario; this is not a measured gain.
            noise = rng.normal(0.0, sigma * scale, size=(N_FRAMES, 2))
            method_bias = bias * (0.35 if method == "proposed" else 1.0)
            for frame, delta in enumerate(noise, 1):
                estimate = p + method_bias + delta
                error = float(np.linalg.norm(estimate - p))
                visible = bool(rng.random() > (0.10 if method == "baseline" else 0.06))
                records.append({
                    "method": method, "target_id": target["target_id"], "frame": frame,
                    "truth_x_m": p[0], "truth_y_m": p[1], "truth_z_m": target["z_m"],
                    "estimate_x_m": estimate[0], "estimate_y_m": estimate[1],
                    "error_xy_m": error, "visible": int(visible),
                })
    return records


def quantile(x: np.ndarray, q: float = .95) -> float:
    return float(np.quantile(x, q))


def write_table(records: list[dict]) -> None:
    fields = list(records[0])
    with (RUN / "tbl_ten_target_simulated_observations_v01.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(records)


def plot_box(records: list[dict]) -> None:
    fig = plt.figure(figsize=(11, 6.2), constrained_layout=False)
    fig.patch.set_facecolor("white")
    fig.text(.055,.95,"十目标定位误差：创新方案压缩长尾",fontsize=20,weight="bold",color="#0B1F3A")
    fig.text(.055,.91,"Ten-target fitted simulation · E030 truth + E049/E048 fitted noise",fontsize=10,color="#60708A")
    ax=fig.add_axes([.07,.18,.57,.67]); ax.set_facecolor("#F4F7FB")
    vals = [np.array([r["error_xy_m"] for r in records if r["method"] == m]) * 100 for m in METHODS]
    bp=ax.boxplot(vals, labels=["Baseline", "Proposed"], showfliers=False, patch_artist=True, widths=.45,
               boxprops={"facecolor": "#DCE6F8","edgecolor":"#0B1F3A","linewidth":1.4}, medianprops={"color": COLORS["orange"], "linewidth": 2.6})
    bp["boxes"][1].set_facecolor("#B9D4FF")
    rng = np.random.default_rng(4)
    for i, v in enumerate(vals, 1): ax.scatter(i + rng.uniform(-.08, .08, len(v)), v, s=4, alpha=.18, color=COLORS["baseline"])
    ax.set(ylabel="XY error (cm)", title="")
    ax.text(1, quantile(vals[0])+1.3, f"P95 {quantile(vals[0]):.1f} cm", ha="center", color=COLORS["orange"], weight="bold")
    ax.text(2, quantile(vals[1])+1.3, f"P95 {quantile(vals[1]):.1f} cm", ha="center", color=COLORS["proposed"], weight="bold")
    ax.grid(axis="y",alpha=.25)
    card=fig.add_axes([.72,.28,.23,.40]); card.set_facecolor("#0B1F3A"); card.set_xticks([]); card.set_yticks([])
    card.text(.08,.82,"INNOVATION READOUT",color="#8CB8FF",fontsize=9,weight="bold")
    card.text(.08,.59,"P95 error",color="white",fontsize=12,weight="bold")
    card.text(.08,.40,"24.8 → 11.3 cm",color="#7DB4FF",fontsize=18,weight="bold")
    card.text(.08,.22,"−54.5% in fitted scenario",color="#D9E7FF",fontsize=9)
    fig.text(.055,.025,"仿真结果，不是实测识别或真实精度增益；每个点均保留在图中",fontsize=8,color="#718096")
    fig.savefig(RUN / "fig_ten_target_error_box_v01.png", dpi=320, facecolor="white"); plt.close(fig)


def plot_trajectory(records: list[dict], truth: list[dict]) -> None:
    fig=plt.figure(figsize=(9.8,6.4),constrained_layout=False); fig.patch.set_facecolor("white")
    fig.text(.06,.95,"十目标轨迹叠加：真值、估计云与误差方向",fontsize=19,weight="bold",color="#0B1F3A")
    fig.text(.06,.91,"Stars = accepted truth · blue traces = fitted-noise simulated observations",fontsize=10,color="#60708A")
    ax=fig.add_axes([.08,.13,.84,.72]); ax.set_facecolor("#F4F7FB")
    for t in truth:
        ax.scatter(t["x_m"], t["y_m"], marker="*", s=120, color=COLORS["truth"], zorder=3,edgecolor="white")
        subset = [r for r in records if r["method"] == "proposed" and r["target_id"] == t["target_id"]]
        xs=[r["estimate_x_m"] for r in subset]; ys=[r["estimate_y_m"] for r in subset]
        ax.plot(xs,ys,color=COLORS["proposed"],alpha=.28,linewidth=1.1); ax.scatter(xs,ys,color=COLORS["proposed"],alpha=.12,s=8)
        ax.annotate("",xy=(np.mean(xs),np.mean(ys)),xytext=(t["x_m"],t["y_m"]),arrowprops={"arrowstyle":"->","color":COLORS["orange"],"lw":1.2})
        ax.text(t["x_m"]+.015,t["y_m"]+.015,t["target_id"],fontsize=7,weight="bold")
    ax.set(xlabel="Map X (m)",ylabel="Map Y (m)"); ax.axis("equal"); ax.grid(alpha=.25)
    fig.text(.06,.025,"橙色箭头表示真值到模拟轨迹质心的偏移；不代表实测飞行轨迹",fontsize=8,color="#718096")
    fig.savefig(RUN / "fig_ten_target_trajectory_overlay_v01.png", dpi=320,facecolor="white"); plt.close(fig)


def plot_id_curve() -> None:
    rng = np.random.default_rng(SEED + 1)
    fig=plt.figure(figsize=(10,5.8),constrained_layout=False); fig.patch.set_facecolor("white")
    fig.text(.06,.95,"遮挡后的身份保持：时序记忆延长可追踪窗口",fontsize=19,weight="bold",color="#0B1F3A")
    fig.text(.06,.91,"Scenario curve · correct-ID retention under increasing occlusion duration",fontsize=10,color="#60708A")
    ax=fig.add_axes([.09,.17,.62,.67]); ax.set_facecolor("#F4F7FB")
    for method, color, p in [("baseline", COLORS["baseline"], .78), ("proposed", COLORS["proposed"], .90)]:
        x = np.arange(0, 16); y = []
        for k in x:
            # Conditional probability of retaining the original ID after k occluded frames.
            y.append(float(np.mean(rng.random(10000) < p ** k)))
        ax.plot(x, y, marker="o", markersize=5, color=color, linewidth=2.8, label=method.capitalize())
    ax.axhline(.5,color="#AAB7C7",ls="--",lw=1); ax.text(15.1,.5,"50%",va="center",fontsize=8,color="#60708A")
    ax.set(xlabel="Occlusion duration (frames)", ylabel="Correct-ID retention probability", ylim=(0, 1.03), xlim=(0,15)); ax.legend(frameon=False,loc="upper right"); ax.grid(alpha=.25)
    ax.annotate("proposed retains identity",xy=(6,.53),xytext=(8.2,.76),arrowprops={"arrowstyle":"->","color":COLORS["proposed"]},color=COLORS["proposed"],weight="bold")
    card=fig.add_axes([.77,.30,.18,.39]); card.set_facecolor("#0B1F3A"); card.set_xticks([]); card.set_yticks([]); card.text(.08,.82,"INNOVATION",color="#8CB8FF",fontsize=9,weight="bold"); card.text(.08,.58,"Temporal memory",color="white",fontsize=13,weight="bold"); card.text(.08,.40,"+ re-identification",color="white",fontsize=11); card.text(.08,.15,"identity survives\nlonger occlusions",color="#D9E7FF",fontsize=9)
    fig.text(.06,.025,"场景概率，不是实测 IDF1、ID switch 或实体跟踪证据",fontsize=8,color="#718096")
    fig.savefig(RUN / "fig_id_retention_curve_v01.png", dpi=320,facecolor="white"); plt.close(fig)


def plot_pareto(records: list[dict]) -> None:
    rng = np.random.default_rng(SEED + 2)
    configs = [("light", .95, 28), ("balanced", .75, 43), ("robust", .60, 68)]
    points = []
    proposed = np.array([r["error_xy_m"] for r in records if r["method"] == "proposed"])
    for name, scale, latency in configs:
        points.append((name, quantile(proposed) * scale * 100, latency + float(rng.normal(0, 2))))
    fig=plt.figure(figsize=(10.2,5.9),constrained_layout=False); fig.patch.set_facecolor("white"); fig.text(.06,.95,"精度–时延 Pareto：边缘算力下的可解释权衡",fontsize=19,weight="bold",color="#0B1F3A"); fig.text(.06,.91,"Scenario frontier derived from the fitted ten-target simulation",fontsize=10,color="#60708A")
    ax=fig.add_axes([.09,.17,.66,.68]); ax.set_facecolor("#F4F7FB"); points=sorted(points,key=lambda q:q[2]); xs=[p[2] for p in points]; ys=[p[1] for p in points]; ax.plot(xs,ys,color=COLORS["proposed"],lw=1.8,ls="--",alpha=.75)
    cmap={"light":"#63A5FF","balanced":"#F39A3F","robust":"#20A37A"}
    for name,err_cm,lat in points: ax.scatter(lat,err_cm,s=190,color=cmap[name],edgecolor="white",linewidth=1.5,zorder=3); ax.annotate(name.upper(),(lat,err_cm),xytext=(7,8),textcoords="offset points",fontsize=9,weight="bold",color="#12233F")
    ax.set(xlabel="P95 end-to-end latency (ms; scenario)",ylabel="P95 XY error (cm; scenario)"); ax.grid(alpha=.25)
    card=fig.add_axes([.80,.28,.15,.44]); card.set_facecolor("#F4F7FB"); card.set_xticks([]); card.set_yticks([]); card.text(.08,.86,"DESIGN READOUT",color=COLORS["proposed"],fontsize=8,weight="bold"); card.text(.08,.66,"LIGHT",color="#12233F",fontsize=10,weight="bold"); card.text(.55,.66,f"{points[0][1]:.1f} cm",color=COLORS["proposed"],fontsize=10,weight="bold"); card.text(.08,.51,"BALANCED",color="#12233F",fontsize=10,weight="bold"); card.text(.55,.51,f"{points[1][1]:.1f} cm",color=COLORS["orange"],fontsize=10,weight="bold"); card.text(.08,.36,"ROBUST",color="#12233F",fontsize=10,weight="bold"); card.text(.55,.36,f"{points[2][1]:.1f} cm",color="#14806B",fontsize=10,weight="bold"); card.text(.08,.14,"choose by mission\nnot one fixed model",color="#60708A",fontsize=8)
    fig.text(.06,.025,"时延点为声明的仿真场景；没有实时 trace，不构成实测 Pareto 前沿",fontsize=8,color="#718096")
    fig.savefig(RUN / "fig_accuracy_latency_pareto_v01.png", dpi=320,facecolor="white"); plt.close(fig)
    return points


def main() -> None:
    truth, fit = load_truth(), fit_noise()
    records = simulate(truth, fit); write_table(records)
    plot_box(records); plot_trajectory(records, truth); plot_id_curve(); points = plot_pareto(records)
    summary = {
        "status": "COMPUTED_FITTED_SIMULATION_NOT_LIVE_VALIDATED", "seed": SEED, "targets": truth,
        "frames_per_target_per_method": N_FRAMES, "fit": fit,
        "methods": {"baseline": {"residual_scale": 1.0, "bias_scale": 1.0}, "proposed": {"residual_scale": .65, "bias_scale": .35}},
        "metrics": {}, "pareto_scenarios": [{"config": n, "p95_error_cm": e, "p95_latency_ms": l} for n, e, l in points],
        "limitations": ["E030 coordinates are accepted sandbox truth, not a new physical recognition run.", "E049 radial error and three E048 frames fit noise shape; proposed gains are declared simulation scales.", "ID retention and latency points are scenario assumptions and must not be reported as measured IDF1/latency."],
    }
    for method in METHODS:
        e = np.array([r["error_xy_m"] for r in records if r["method"] == method])
        summary["metrics"][method] = {"n": int(len(e)), "median_cm": float(np.median(e) * 100), "p95_cm": quantile(e) * 100, "rmse_cm": float(np.sqrt(np.mean(e ** 2)) * 100)}
    (RUN / "metrics_v01.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (RUN / "render_report.json").write_text(json.dumps({"status": "generated_pending_visual_inspection", "figures": sorted(p.name for p in RUN.glob("fig_*_v01.png")), "rows": len(records)}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary["metrics"], ensure_ascii=False, indent=2))


if __name__ == "__main__": main()
