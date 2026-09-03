"""Algorithm-router adapters for the three independently runnable chains."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .bundle_solver import BundleProblem, load_chain_config, solve_bundle
from .gating import QualityMetrics, select_chain
from .online_solver import OnlineFrame, OnlineLocalizer, online_solution


REPOSITORY = Path(__file__).resolve().parents[3]
CONFIGS = {
    "chain_a_precision": REPOSITORY / "config/chain_a_precision.yaml",
    "chain_b_robust": REPOSITORY / "config/chain_b_robust.yaml",
    "chain_c_adaptive": REPOSITORY / "config/chain_c_adaptive.yaml",
    "chain_d_online_visual": REPOSITORY / "config/chain_d_online_visual.yaml",
}


def _quality(value: QualityMetrics | dict[str, Any] | None) -> QualityMetrics | None:
    if value is None or isinstance(value, QualityMetrics):
        return value
    return QualityMetrics(**value)


def _invalid(algorithm: str, reason: str, decision: str = "unavailable") -> AlgorithmResult:
    from algorithms.contracts import AlgorithmResult
    return AlgorithmResult(
        estimate=np.full((0, 4, 4), np.nan), covariance=np.full((0, 6, 6), np.nan),
        algorithm=algorithm, family="vision", valid=False,
        metadata={"validity": "INVALID", "degraded_reason": reason, "gate_level": decision},
    )


def _run(name: str, problem: BundleProblem, quality: QualityMetrics | dict[str, Any] | None = None, config_path: str | Path | None = None) -> AlgorithmResult:
    from algorithms.contracts import AlgorithmResult
    quality_value = _quality(quality)
    selected_name = name
    gate_level = "precision" if name == "chain_a_precision" else "transition"
    degraded_reason = ""
    if name == "chain_c_adaptive":
        if quality_value is None:
            raise ValueError("chain C requires quality metrics")
        adaptive_path = Path(config_path) if config_path else CONFIGS[name]
        adaptive = yaml.safe_load(adaptive_path.read_text(encoding="utf-8"))
        decision = select_chain(quality_value, adaptive)
        if not decision.valid or decision.chain is None:
            return _invalid("vision.bundle_c", decision.degraded_reason, decision.level)
        selected_name, gate_level, degraded_reason = decision.chain, decision.level, decision.degraded_reason
        config_path = REPOSITORY / adaptive["precision_config" if selected_name == "chain_a_precision" else "robust_config"]
    elif name == "chain_b_robust" and quality_value is not None:
        raw = yaml.safe_load((Path(config_path) if config_path else CONFIGS[name]).read_text(encoding="utf-8"))
        decision = select_chain(quality_value, {"adaptive_gates": raw.get("gates", {})})
        if not decision.valid:
            return _invalid("vision.bundle_b", decision.degraded_reason, decision.level)
        gate_level, degraded_reason = decision.level, decision.degraded_reason
    config = load_chain_config(config_path or CONFIGS[selected_name])
    solution = solve_bundle(problem, config)
    target_ids = sorted(solution.target_poses)
    estimates = np.stack([solution.target_poses[target_id].matrix() for target_id in target_ids])
    covariances = np.stack([solution.target_covariances.get(target_id, np.full((6, 6), np.nan)) for target_id in target_ids])
    algorithm = {"chain_a_precision": "vision.bundle_a", "chain_b_robust": "vision.bundle_b", "chain_c_adaptive": "vision.bundle_c"}[name]
    reason = solution.degraded_reason or degraded_reason
    return AlgorithmResult(
        estimate=estimates, covariance=covariances, algorithm=algorithm, family="vision",
        valid=solution.valid,
        metadata={
            "target_ids": target_ids, "chain": name, "selected_chain": selected_name,
            "gate_level": gate_level, "validity": "VALID" if solution.valid else "INVALID",
            "degraded_reason": reason, "runtime_ms": solution.runtime_ms,
            "depth_source": solution.depth_source, "residual_count": solution.residual_count,
            "rejected_residual_count": solution.rejected_residual_count,
            "view_count": solution.view_count, "baseline_m": solution.baseline_m,
            "stage_costs": list(solution.stage_costs),
            "segment_scale": solution.segment_scale, "segment_bias_m": solution.segment_bias_m.tolist(),
            "network_depth_used": False,
            "unobservable_target_dofs": solution.unobservable_target_dofs,
        },
    )


def run_chain_a(**inputs: Any) -> AlgorithmResult:
    return _run("chain_a_precision", **inputs)


def run_chain_b(**inputs: Any) -> AlgorithmResult:
    return _run("chain_b_robust", **inputs)


def run_chain_c(**inputs: Any) -> AlgorithmResult:
    return _run("chain_c_adaptive", **inputs)


def run_chain_d(*, localizer: OnlineLocalizer, frame: OnlineFrame, **_: Any) -> AlgorithmResult:
    """Run one bounded online update and expose the common algorithm contract."""
    from algorithms.contracts import AlgorithmResult
    if not isinstance(localizer, OnlineLocalizer) or not isinstance(frame, OnlineFrame):
        raise TypeError("chain D requires an OnlineLocalizer and OnlineFrame")
    update = localizer.update(frame)
    solution = online_solution(update)
    target_ids = sorted(solution.target_poses)
    if target_ids:
        estimates = np.stack([solution.target_poses[target_id].matrix() for target_id in target_ids])
        covariances = np.stack([solution.target_covariances.get(target_id, np.full((6, 6), np.nan)) for target_id in target_ids])
    else:
        estimates = np.empty((0, 4, 4), dtype=float)
        covariances = np.empty((0, 6, 6), dtype=float)
    return AlgorithmResult(
        estimate=estimates, covariance=covariances, algorithm="vision.bundle_d", family="vision",
        valid=solution.valid,
        metadata={
            "target_ids": target_ids, "chain": "chain_d_online_visual", "selected_chain": "chain_d_online_visual",
            "gate_level": update.gate_level, "validity": "VALID" if solution.valid else "INVALID",
            "degraded_reason": solution.degraded_reason, "runtime_ms": solution.runtime_ms,
            "depth_source": solution.depth_source, "view_count": solution.view_count,
            "baseline_m": solution.baseline_m, "within_budget": update.within_budget,
            "window_camera_ids": list(update.window_camera_ids), "segment_scale": solution.segment_scale,
            "segment_bias_m": solution.segment_bias_m.tolist(), "network_depth_used": False,
        },
    )
