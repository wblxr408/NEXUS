"""Low-compute markerless target localization bundle solver."""

from .bundle_solver import (
    BundleConfig,
    BundleProblem,
    BundleSolution,
    PoseState,
    load_chain_config,
    solve_bundle,
)
from .bundle_io import factor_from_dict, load_problem, problem_from_dict
from .channel_consistency import ChannelPose, ConsistencyReport, channel_consistency
from .gating import GateDecision, QualityMetrics, select_chain
from .records import COLUMNS, SolveRecord, result_records, solve_records, write_records

__all__ = [
    "BundleConfig",
    "BundleProblem",
    "BundleSolution",
    "COLUMNS",
    "ChannelPose",
    "ConsistencyReport",
    "GateDecision",
    "PoseState",
    "QualityMetrics",
    "SolveRecord",
    "channel_consistency",
    "factor_from_dict",
    "load_chain_config",
    "load_problem",
    "problem_from_dict",
    "result_records",
    "select_chain",
    "solve_bundle",
    "solve_records",
    "write_records",
]
