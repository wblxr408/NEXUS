"""Sparse F1--F8 least-squares solver shared by localization chains A/B/C."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from time import perf_counter
from typing import Iterable

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from .factors import Factor


@dataclass(frozen=True)
class PoseState:
    rotation: np.ndarray
    translation_m: np.ndarray

    @classmethod
    def identity(cls, translation_m=(0.0, 0.0, 0.0)) -> "PoseState":
        return cls(np.eye(3), np.asarray(translation_m, dtype=float))

    def matrix(self) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = self.rotation
        transform[:3, 3] = self.translation_m
        return transform


@dataclass
class BundleProblem:
    target_poses: dict[int, PoseState]
    camera_poses: dict[int, PoseState]
    factors: list[Factor]
    segment_scale: float = 1.0
    segment_bias_m: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass(frozen=True)
class BundleConfig:
    chain: str
    enabled_factors: tuple[str, ...]
    factor_weights: dict[str, float]
    optimize_camera_poses: bool = False
    optimize_segment_alignment: bool = False
    loss: str = "soft_l1"
    f_scale: float = 1.0
    max_nfev: int = 200
    outlier_sigma: float = 3.0
    second_pass: bool = True
    minimum_views: int = 2
    minimum_baseline_m: float = 0.10


@dataclass(frozen=True)
class BundleSolution:
    target_poses: dict[int, PoseState]
    camera_poses: dict[int, PoseState]
    target_covariances: dict[int, np.ndarray]
    valid: bool
    degraded_reason: str
    cost: float
    optimality: float
    residual_count: int
    rejected_residual_count: int
    runtime_ms: float
    depth_source: str
    segment_scale: float
    segment_bias_m: np.ndarray


class _Layout:
    def __init__(self, problem: BundleProblem, config: BundleConfig, factors: list[Factor]):
        self.target_slices: dict[int, slice] = {}
        self.camera_slices: dict[int, slice] = {}
        cursor = 0
        for target_id in sorted(problem.target_poses):
            self.target_slices[target_id] = slice(cursor, cursor + 6)
            cursor += 6
        referenced_cameras = {int(identifier) for factor in factors for kind, identifier in factor.dependencies() if kind == "camera"}
        if config.optimize_camera_poses:
            for camera_id in sorted(referenced_cameras):
                self.camera_slices[camera_id] = slice(cursor, cursor + 6)
                cursor += 6
        has_segment_factor = any(kind == "segment" for factor in factors for kind, _ in factor.dependencies())
        self.segment_slice = slice(cursor, cursor + 4) if config.optimize_segment_alignment and has_segment_factor else None
        if self.segment_slice is not None:
            cursor += 4
        self.size = cursor

    @staticmethod
    def _pose_vector(pose: PoseState) -> np.ndarray:
        return np.r_[Rotation.from_matrix(np.asarray(pose.rotation)).as_rotvec(), np.asarray(pose.translation_m)]

    @staticmethod
    def _pose(vector: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return Rotation.from_rotvec(vector[:3]).as_matrix(), vector[3:6]

    def initial(self, problem: BundleProblem) -> np.ndarray:
        state = np.zeros(self.size)
        for key, value in self.target_slices.items():
            state[value] = self._pose_vector(problem.target_poses[key])
        for key, value in self.camera_slices.items():
            state[value] = self._pose_vector(problem.camera_poses[key])
        if self.segment_slice is not None:
            state[self.segment_slice] = np.r_[np.log(problem.segment_scale), problem.segment_bias_m]
        return state


class _State:
    def __init__(self, vector: np.ndarray, layout: _Layout, problem: BundleProblem):
        self.vector, self.layout, self.problem = vector, layout, problem

    def target_pose(self, target_id: int) -> tuple[np.ndarray, np.ndarray]:
        return self.layout._pose(self.vector[self.layout.target_slices[target_id]])

    def camera_pose(self, camera_id: int) -> tuple[np.ndarray, np.ndarray]:
        if camera_id in self.layout.camera_slices:
            return self.layout._pose(self.vector[self.layout.camera_slices[camera_id]])
        pose = self.problem.camera_poses[camera_id]
        return np.asarray(pose.rotation), np.asarray(pose.translation_m)

    def segment_alignment(self) -> tuple[float, np.ndarray]:
        if self.layout.segment_slice is None:
            return self.problem.segment_scale, np.asarray(self.problem.segment_bias_m)
        values = self.vector[self.layout.segment_slice]
        return float(np.exp(values[0])), values[1:4]


def load_chain_config(path: str | Path) -> BundleConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    solver = raw.get("solver", {})
    return BundleConfig(
        chain=str(raw["chain"]),
        enabled_factors=tuple(raw["factors"]["enabled"]),
        factor_weights={str(key): float(value) for key, value in raw["factors"].get("weights", {}).items()},
        optimize_camera_poses=bool(solver.get("optimize_camera_poses", False)),
        optimize_segment_alignment=bool(solver.get("optimize_segment_alignment", False)),
        loss=str(solver.get("loss", "soft_l1")),
        f_scale=float(solver.get("f_scale", 1.0)),
        max_nfev=int(solver.get("max_nfev", 200)),
        outlier_sigma=float(solver.get("outlier_sigma", 3.0)),
        second_pass=bool(solver.get("second_pass", True)),
        minimum_views=int(raw.get("gates", {}).get("minimum_views", 2)),
        minimum_baseline_m=float(raw.get("gates", {}).get("minimum_baseline_m", 0.10)),
    )


def _active_factors(problem: BundleProblem, config: BundleConfig) -> list[Factor]:
    enabled = set(config.enabled_factors)
    factors = [factor for factor in problem.factors if factor.kind in enabled and config.factor_weights.get(factor.kind, 1.0) > 0.0]
    if not factors:
        raise ValueError("bundle has no enabled factors")
    return factors


def _residual_blocks(vector: np.ndarray, layout: _Layout, problem: BundleProblem, factors: Iterable[Factor], config: BundleConfig) -> list[np.ndarray]:
    state = _State(vector, layout, problem)
    return [np.sqrt(config.factor_weights.get(factor.kind, 1.0)) * np.ravel(factor.residual(state)) for factor in factors]


def _residual(vector: np.ndarray, layout: _Layout, problem: BundleProblem, factors: list[Factor], config: BundleConfig) -> np.ndarray:
    return np.concatenate(_residual_blocks(vector, layout, problem, factors, config))


def _dependency_slice(layout: _Layout, dependency: tuple[str, int | None]) -> slice | None:
    kind, identifier = dependency
    if kind == "target":
        return layout.target_slices[int(identifier)]
    if kind == "camera":
        return layout.camera_slices.get(int(identifier))
    if kind == "segment":
        return layout.segment_slice
    raise ValueError(f"unknown dependency kind: {kind}")


def _jacobian_sparsity(x0: np.ndarray, layout: _Layout, problem: BundleProblem, factors: list[Factor], config: BundleConfig):
    blocks = _residual_blocks(x0, layout, problem, factors, config)
    matrix = lil_matrix((sum(len(block) for block in blocks), layout.size), dtype=np.int8)
    row = 0
    for factor, block in zip(factors, blocks):
        for dependency in factor.dependencies():
            columns = _dependency_slice(layout, dependency)
            if columns is not None:
                matrix[row:row + len(block), columns] = 1
        row += len(block)
    return matrix.tocsr()


def _pose_dict(vector: np.ndarray, slices: dict[int, slice], layout: _Layout) -> dict[int, PoseState]:
    result = {}
    for identifier, block in slices.items():
        rotation, translation = layout._pose(vector[block])
        result[identifier] = PoseState(rotation, translation.copy())
    return result


def _covariances(result, layout: _Layout) -> dict[int, np.ndarray]:
    jacobian = result.jac.toarray() if hasattr(result.jac, "toarray") else np.asarray(result.jac)
    dof = max(1, jacobian.shape[0] - jacobian.shape[1])
    variance = float(2.0 * result.cost / dof)
    covariance = np.linalg.pinv(jacobian.T @ jacobian, rcond=1e-10) * variance
    return {identifier: covariance[block, block].copy() for identifier, block in layout.target_slices.items()}


def solve_bundle(problem: BundleProblem, config: BundleConfig) -> BundleSolution:
    """Solve an F1--F8 bundle and return explicit validity/provenance fields."""
    started = perf_counter()
    factors = _active_factors(problem, config)
    layout = _Layout(problem, config, factors)
    x0 = layout.initial(problem)
    sparsity = _jacobian_sparsity(x0, layout, problem, factors, config)
    result = least_squares(
        _residual, x0, args=(layout, problem, factors, config), method="trf",
        loss=config.loss, f_scale=config.f_scale, jac_sparsity=sparsity,
        max_nfev=config.max_nfev, x_scale="jac",
    )
    rejected = 0
    if config.second_pass and result.fun.size:
        blocks = _residual_blocks(result.x, layout, problem, factors, config)
        keep_flags = [np.linalg.norm(block) <= config.outlier_sigma * np.sqrt(len(block)) for block in blocks]
        keep = [factor for factor, flag in zip(factors, keep_flags) if flag]
        rejected = sum(len(block) for block, flag in zip(blocks, keep_flags) if not flag)
        if keep and len(keep) < len(factors):
            result = least_squares(
                _residual, result.x, args=(layout, problem, keep, config), method="trf",
                loss=config.loss, f_scale=config.f_scale,
                jac_sparsity=_jacobian_sparsity(result.x, layout, problem, keep, config),
                max_nfev=config.max_nfev, x_scale="jac",
            )
            factors = keep
    target_poses = _pose_dict(result.x, layout.target_slices, layout)
    camera_poses = _pose_dict(result.x, layout.camera_slices, layout)
    camera_poses = {**problem.camera_poses, **camera_poses}
    scale, bias = _State(result.x, layout, problem).segment_alignment()
    bearing_factors = [factor for factor in factors if factor.kind == "F1"]
    camera_ids = sorted({factor.camera_id for factor in bearing_factors})
    baseline = 0.0
    for first in camera_ids:
        for second in camera_ids:
            baseline = max(baseline, float(np.linalg.norm(camera_poses[first].translation_m - camera_poses[second].translation_m)))
    triangulated = len(camera_ids) >= config.minimum_views and baseline >= config.minimum_baseline_m
    has_support = any(f.kind == "F3" for f in factors)
    source = "triangulation" if triangulated else "support_plane" if has_support else "uwb_fallback"
    finite = np.all(np.isfinite(result.x)) and np.all(np.isfinite(result.fun))
    constrained_targets = {int(identifier) for factor in factors if factor.kind in {"F1", "F2", "F3"} for kind, identifier in factor.dependencies() if kind == "target"}
    missing_targets = sorted(set(problem.target_poses) - constrained_targets)
    observable = (triangulated or has_support) and not missing_targets
    valid = bool(result.success and finite and observable)
    if missing_targets:
        reason = f"unconstrained_targets:{','.join(map(str, missing_targets))}"
    elif not observable:
        reason = "insufficient_geometric_depth_constraint"
    else:
        reason = "" if valid else (result.message if result.message else "solver_failed")
    return BundleSolution(
        target_poses=target_poses,
        camera_poses=camera_poses,
        target_covariances=_covariances(result, layout) if finite else {},
        valid=valid,
        degraded_reason=str(reason),
        cost=float(result.cost),
        optimality=float(result.optimality),
        residual_count=int(result.fun.size),
        rejected_residual_count=int(rejected),
        runtime_ms=(perf_counter() - started) * 1000.0,
        depth_source=source,
        segment_scale=scale,
        segment_bias_m=np.asarray(bias).copy(),
    )
