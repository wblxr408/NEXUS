"""Online sliding-window localization for live drone capture (design C6).

The batch solver in :mod:`bundle_solver` re-solves a whole segment; this module
keeps the same factors and the same config files but bounds the work per frame
so the platform can localize while it flies: a fixed window of recent camera
frames, a warm start from the previous solution, and a bounded iteration count.

Only outgoing measurements and the preceding marginal factor are compressed
into a square-root prior. Retained raw measurements are not counted in that
prior. Fixed support/catalog priors remain once in the graph. Pose, camera and
segment cross-information is preserved without filling unobservable directions.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, fields, replace
from time import perf_counter

import numpy as np

from .bundle_marginalization import MarginalFactor, marginalize_factors
from .bundle_solver import BundleConfig, BundleProblem, BundleSolution, PoseState, solve_bundle
from .factors import CatalogPriorFactor, Factor, SupportPlaneFactor
from .gating import GateDecision, QualityMetrics, select_chain


# A 10 Hz update budget (design C6) leaves this much wall clock per frame.
UPDATE_BUDGET_MS = 100.0


@dataclass(frozen=True)
class OnlineFrame:
    """One live capture: the platform pose estimate plus this frame's factors."""

    camera_id: int
    camera_pose: PoseState
    factors: tuple[Factor, ...]
    quality: QualityMetrics | None = None
    frame_id: str = ""


@dataclass(frozen=True)
class OnlineUpdate:
    frame_id: str
    camera_id: int
    target_poses: dict[int, PoseState]
    target_covariances: dict[int, np.ndarray]
    valid: bool
    degraded_reason: str
    depth_source: str
    gate_level: str
    window_camera_ids: tuple[int, ...]
    view_count: int
    baseline_m: float
    segment_scale: float
    segment_bias_m: np.ndarray
    runtime_ms: float
    within_budget: bool


@dataclass
class _WindowEntry:
    frame: OnlineFrame


class _SolutionState:
    def __init__(self, solution):
        self.solution = solution

    def target_pose(self, identifier):
        pose = self.solution.target_poses[identifier]
        return pose.rotation, pose.translation_m

    def camera_pose(self, identifier):
        pose = self.solution.camera_poses[identifier]
        return pose.rotation, pose.translation_m

    def segment_alignment(self):
        return self.solution.segment_scale, self.solution.segment_bias_m


class OnlineLocalizer:
    """Fixed-window incremental solver over the batch factor set.

    ``window_size`` is the number of retained camera frames. With marginalization
    enabled, solve at most N+1 frames, then eliminate the oldest camera. State
    dimension is bounded by ``6 * targets + 6 * (N+1) + 4``. This static-target
    solver does not model moving targets; their frontend has a separate model.
    """

    def __init__(self, config: BundleConfig, initial_target_poses: dict[int, PoseState],
                 window_size: int = 8, max_nfev: int = 30,
                 carry_prior: bool = True, prior_inflation: float = 1.0,
                 gate_config: dict | str | None = None):
        if window_size < 2:
            raise ValueError("an online window needs at least two frames to intersect bearings")
        if not np.isfinite(prior_inflation) or prior_inflation < 1.:
            raise ValueError("prior_inflation must be finite and at least one")
        # One iteration-capped solve per update. The robust loss is applied to
        # raw factors, never a second time to already whitened marginal rows.
        self.config = replace(config, max_nfev=int(max_nfev), second_pass=False, stages=())
        self.window: deque[_WindowEntry] = deque(maxlen=int(window_size))
        self.target_poses = deepcopy({int(key): value for key, value in initial_target_poses.items()})
        self.target_covariances: dict[int, np.ndarray] = {}
        self.carry_prior = bool(carry_prior)
        # Optional standard-deviation forgetting, once per eviction; one means
        # no artificial loss of historical information (the default).
        self.prior_inflation = float(prior_inflation)
        self._marginal_prior: MarginalFactor | None = None
        self._persistent_priors: dict[tuple, Factor] = {}
        self._camera_poses: dict[int, PoseState] = {}
        self._last_camera_id: int | None = None
        self.gate_config = gate_config or {
            "adaptive_gates": {
                "precision_minimum_views": int(config.minimum_views),
                "transition_minimum_views": int(config.minimum_views),
            }
        }
        self.segment_scale = 1.0
        self.segment_bias_m = np.zeros(3)
        self.updates = 0

    def _prior_factors(self) -> list[Factor]:
        return [self._marginal_prior] if self.carry_prior and self._marginal_prior is not None else []

    def _prepare(self, frame):
        """Own each raw factor once; keep fixed physical priors outside frames."""
        persistent = dict(self._persistent_priors)
        active = []
        camera_ids = {entry.frame.camera_id for entry in self.window} | {frame.camera_id}
        for factor in frame.factors:
            if isinstance(factor, MarginalFactor):
                raise ValueError("marginal factors are owned by the localizer")
            if factor.kind not in self.config.enabled_factors or self.config.factor_weights.get(factor.kind, 1.) <= 0:
                continue
            dependencies = factor.dependencies()
            for kind, identifier in dependencies:
                if kind == "target" and identifier not in self.target_poses:
                    raise ValueError("unknown target in frame factor")
                if kind == "camera" and identifier not in camera_ids:
                    raise ValueError("camera dependency outside retained window")
            observed_cameras = {identifier for kind, identifier in dependencies if kind == "camera"}
            if observed_cameras and (frame.camera_id not in observed_cameras
                                     or (factor.kind != "F5" and observed_cameras != {frame.camera_id})):
                raise ValueError("factor does not belong to its capture frame")
            if isinstance(factor, (SupportPlaneFactor, CatalogPriorFactor)):
                key = (factor.kind, factor.target_id)
                old = persistent.get(key)
                if old is not None and any(not np.array_equal(getattr(old, item.name), getattr(factor, item.name))
                                           for item in fields(factor)):
                    raise ValueError("fixed prior changed; start a new localizer")
                persistent[key] = deepcopy(factor)
            else:
                active.append(deepcopy(factor))
        return _WindowEntry(replace(frame, camera_pose=deepcopy(frame.camera_pose), factors=tuple(active))), persistent

    def _trim_raw(self, entries):
        """No-history ablation and incomplete startup keep only evaluable rows."""
        entries = entries[-self.window.maxlen:]
        cameras = {entry.frame.camera_id for entry in entries}
        return [_WindowEntry(replace(entry.frame, factors=tuple(
            factor for factor in entry.frame.factors
            if all(kind != "camera" or identifier in cameras for kind, identifier in factor.dependencies())
        ))) for entry in entries]

    def _evict(self, entries, solution):
        if len(entries) <= self.window.maxlen:
            return entries, self._marginal_prior
        removed = ("camera", entries[0].frame.camera_id)
        outgoing = list(entries[0].frame.factors) + self._prior_factors()
        retained = []
        for entry in entries[1:]:
            keep = []
            for factor in entry.frame.factors:
                if removed in factor.dependencies():
                    outgoing.append(factor)
                else:
                    keep.append(factor)
            retained.append(_WindowEntry(replace(entry.frame, factors=tuple(keep))))
        prior = marginalize_factors(outgoing, _SolutionState(solution), self.config, {removed},
                                    prior_inflation=self.prior_inflation)
        return retained, prior

    def update(self, frame: OnlineFrame) -> OnlineUpdate:
        started = perf_counter()
        gate_level = "precision"
        if frame.quality is not None:
            decision: GateDecision = select_chain(frame.quality, self.gate_config)
            gate_level = decision.level
            if not decision.valid:
                return self._reject(frame, decision.degraded_reason, gate_level, started)
        if not frame.factors:
            return self._reject(frame, "no_factors", gate_level, started)
        if any(entry.frame.camera_id == frame.camera_id for entry in self.window):
            return self._reject(frame, "duplicate_camera_id", gate_level, started)
        if not isinstance(frame.camera_id, (int, np.integer)) or isinstance(frame.camera_id, bool) or frame.camera_id < 0:
            return self._reject(frame, "invalid_camera_id", gate_level, started)
        if self._last_camera_id is not None and frame.camera_id <= self._last_camera_id:
            return self._reject(frame, "out_of_order_camera_id", gate_level, started)
        try:
            entry, persistent = self._prepare(frame)
            if not entry.frame.factors:
                return self._reject(frame, "no_new_factors", gate_level, started)
            entries = [*self.window, entry]
            if not self.carry_prior:
                entries = self._trim_raw(entries)
            camera_poses = {item.frame.camera_id: self._camera_poses.get(item.frame.camera_id, item.frame.camera_pose)
                            for item in entries}
            factors = [factor for item in entries for factor in item.frame.factors]
            factors += list(persistent.values()) + self._prior_factors()
            problem = BundleProblem(target_poses=dict(self.target_poses), camera_poses=camera_poses, factors=factors,
                                    segment_scale=self.segment_scale, segment_bias_m=self.segment_bias_m.copy())
            solution = solve_bundle(problem, self.config)
            if solution.valid:
                entries, prior = self._evict(entries, solution) if self.carry_prior else (entries, None)
            elif self._marginal_prior is None and solution.degraded_reason.startswith((
                    "unconstrained_targets:", "unobservable_target_positions:",
                    "insufficient_geometric_depth_constraint")):
                # Before initialization, keep raw geometric evidence without
                # freezing an unobservable/poorly initialized linearization.
                entries, prior = self._trim_raw(entries), None
            else:
                return self._reject(frame, solution.degraded_reason, gate_level, started)
        except (ValueError, np.linalg.LinAlgError) as error:
            return self._reject(frame, f"invalid_window:{error}", gate_level, started)
        # Commit only after the solve and marginalization both succeed.
        self.window.clear()
        self.window.extend(entries)
        self._marginal_prior, self._persistent_priors = prior, persistent
        self._last_camera_id = frame.camera_id
        if solution.valid:
            self.target_poses = dict(solution.target_poses)
            self.target_covariances = dict(solution.target_covariances)
            self.segment_scale = solution.segment_scale
            self.segment_bias_m = np.asarray(solution.segment_bias_m).copy()
        self._camera_poses = {entry.frame.camera_id: solution.camera_poses.get(entry.frame.camera_id, entry.frame.camera_pose)
                              if solution.valid else entry.frame.camera_pose for entry in entries}
        self.updates += 1
        runtime_ms = (perf_counter() - started) * 1000.0
        return OnlineUpdate(
            frame_id=frame.frame_id or str(frame.camera_id), camera_id=frame.camera_id,
            target_poses=dict(solution.target_poses), target_covariances=dict(solution.target_covariances),
            valid=solution.valid, degraded_reason=solution.degraded_reason, depth_source=solution.depth_source,
            gate_level=gate_level, window_camera_ids=tuple(entry.frame.camera_id for entry in self.window),
            view_count=solution.view_count, baseline_m=solution.baseline_m,
            segment_scale=solution.segment_scale, segment_bias_m=np.asarray(solution.segment_bias_m).copy(),
            runtime_ms=runtime_ms, within_budget=runtime_ms <= UPDATE_BUDGET_MS,
        )

    def _reject(self, frame: OnlineFrame, reason: str, gate_level: str, started: float) -> OnlineUpdate:
        runtime_ms = (perf_counter() - started) * 1000.0
        return OnlineUpdate(
            frame_id=frame.frame_id or str(frame.camera_id), camera_id=frame.camera_id,
            target_poses={}, target_covariances={}, valid=False, degraded_reason=reason,
            depth_source="", gate_level=gate_level, window_camera_ids=tuple(entry.frame.camera_id for entry in self.window),
            view_count=0, baseline_m=0.0, runtime_ms=runtime_ms, within_budget=runtime_ms <= UPDATE_BUDGET_MS,
            segment_scale=self.segment_scale, segment_bias_m=self.segment_bias_m.copy(),
        )


def online_solution(update: OnlineUpdate) -> BundleSolution:
    """Adapt an update to the batch solution shape so records.py can write it."""
    return BundleSolution(
        target_poses=update.target_poses, camera_poses={}, target_covariances=update.target_covariances,
        valid=update.valid, degraded_reason=update.degraded_reason, cost=0.0, optimality=0.0,
        residual_count=0, rejected_residual_count=0, runtime_ms=update.runtime_ms,
        depth_source=update.depth_source, segment_scale=update.segment_scale,
        segment_bias_m=np.asarray(update.segment_bias_m).copy(),
        view_count=update.view_count, baseline_m=update.baseline_m,
    )
