"""Quality gating shared by robust chain B and adaptive chain C."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class QualityMetrics:
    n_views_valid: int
    baseline_m: float
    bbox_area_px: float
    confidence: float
    pose_sigma_m: float
    channel_disagreement_m: float
    uwb_residual_m: float
    platform_cov_trace_m2: float
    blur_metric: float
    exposure_metric: float
    transform_available: bool = True


@dataclass(frozen=True)
class GateDecision:
    level: str
    chain: str | None
    valid: bool
    degraded_reason: str


def _thresholds(config: str | Path | dict) -> dict:
    if isinstance(config, dict):
        return config.get("adaptive_gates", config.get("gates", config))
    return yaml.safe_load(Path(config).read_text(encoding="utf-8")).get("adaptive_gates", {})


def select_chain(metrics: QualityMetrics, config: str | Path | dict) -> GateDecision:
    """Select precision/transition/fallback without inventing a pose."""
    gate = _thresholds(config)
    if not metrics.transform_available:
        return GateDecision("unavailable", None, False, "transform_unavailable")
    hard_disagreement = float(gate.get("hard_channel_disagreement_m", 0.30))
    hard_uwb = float(gate.get("hard_uwb_residual_m", 0.50))
    if metrics.channel_disagreement_m > hard_disagreement and metrics.uwb_residual_m > hard_uwb:
        return GateDecision("unavailable", None, False, "visual_uwb_channels_inconsistent")
    precision = (
        metrics.n_views_valid >= int(gate.get("precision_minimum_views", 8))
        and metrics.baseline_m >= float(gate.get("precision_minimum_baseline_m", 0.30))
        and metrics.confidence >= float(gate.get("precision_minimum_confidence", 0.70))
        and metrics.pose_sigma_m <= float(gate.get("precision_maximum_pose_sigma_m", 0.04))
        and metrics.channel_disagreement_m <= float(gate.get("precision_maximum_channel_disagreement_m", 0.08))
        and metrics.blur_metric >= float(gate.get("precision_minimum_blur_metric", 80.0))
        and float(gate.get("exposure_minimum", 0.15)) <= metrics.exposure_metric <= float(gate.get("exposure_maximum", 0.90))
    )
    if precision:
        return GateDecision("precision", "chain_a_precision", True, "")
    visual_usable = (
        metrics.n_views_valid >= int(gate.get("transition_minimum_views", 2))
        and metrics.confidence >= float(gate.get("transition_minimum_confidence", 0.35))
        and metrics.channel_disagreement_m <= hard_disagreement
    )
    if visual_usable:
        return GateDecision("transition", "chain_b_robust", True, "visual_degraded")
    if metrics.uwb_residual_m <= hard_uwb:
        return GateDecision("fallback", "chain_b_robust", True, "uwb_fallback")
    return GateDecision("unavailable", None, False, "all_channels_unavailable")
