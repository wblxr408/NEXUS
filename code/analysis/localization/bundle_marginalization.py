"""Square-root marginal factors for target/camera poses and segment alignment.

This module is independent of ROS. Pose increments are [right rotation, map
translation]; segment increments are [log scale, map bias]. Null directions
are retained as zero information, including unobservable target orientation.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import qr
from scipy.spatial.transform import Rotation


def robust_rho(squared, loss):
    """SciPy least_squares loss, first derivative and second derivative."""
    z = np.asarray(squared, dtype=float)
    if loss == "linear":
        return np.vstack((z, np.ones_like(z), np.zeros_like(z)))
    if loss == "soft_l1":
        root = np.sqrt(1 + z)
        return np.vstack((2 * (root - 1), 1 / root, -.5 / root ** 3))
    if loss == "huber":
        outside = z > 1
        root = np.sqrt(np.maximum(z, 1.))
        return np.vstack((np.where(outside, 2 * root - 1, z), np.where(outside, 1 / root, 1.),
                          np.where(outside, -.5 / root ** 3, 0.)))
    if loss == "cauchy":
        return np.vstack((np.log1p(z), 1 / (1 + z), -1 / (1 + z) ** 2))
    if loss == "arctan":
        return np.vstack((np.arctan(z), 1 / (1 + z ** 2), -2 * z / (1 + z ** 2) ** 2))
    raise ValueError(f"unsupported bundle robust loss: {loss}")


@dataclass(frozen=True)
class GeometryEvidence:
    view_count: int = 0
    positions: tuple = ()  # at most six actual baseline witnesses
    baseline_m: float = 0.
    support: bool = False

    def merge(self, other):
        points = np.asarray(self.positions + other.positions, dtype=float).reshape(-1, 3)
        baseline = max(self.baseline_m, other.baseline_m)
        if len(points):
            baseline = max(baseline, float(np.max(np.linalg.norm(points[:, None] - points[None], axis=2))))
            indices = np.unique(np.r_[np.argmin(points, axis=0), np.argmax(points, axis=0)])
            positions = tuple(tuple(point) for point in points[indices])
        else:
            positions = ()
        return GeometryEvidence(self.view_count + other.view_count, positions, baseline, self.support or other.support)


@dataclass(frozen=True)
class StateReference:
    key: tuple
    value: np.ndarray

    @property
    def size(self):
        return 4 if self.key[0] == "segment" else 6

    @classmethod
    def capture(cls, key, state):
        if key[0] == "segment":
            scale, bias = state.segment_alignment()
            if not np.isfinite(scale) or scale <= 0:
                raise ValueError("marginalization requires a positive segment scale")
            value = np.r_[np.log(scale), bias]
        else:
            rotation, translation = state.target_pose(key[1]) if key[0] == "target" else state.camera_pose(key[1])
            value = np.eye(4)
            value[:3, :3], value[:3, 3] = rotation, translation
        if not np.all(np.isfinite(value)):
            raise ValueError("marginalization reference must be finite")
        return cls(key, value.copy())

    def local(self, state):
        current = self.capture(self.key, state).value
        if self.key[0] == "segment":
            return current - self.value
        return np.r_[Rotation.from_matrix(self.value[:3, :3].T @ current[:3, :3]).as_rotvec(), current[:3, 3] - self.value[:3, 3]]

    def retract(self, delta):
        value = self.value.copy()
        if self.key[0] == "segment":
            value += delta
        else:
            value[:3, :3] = value[:3, :3] @ Rotation.from_rotvec(delta[:3]).as_matrix()
            value[:3, 3] += delta[3:]
        return StateReference(self.key, value)


class _LocalState:
    def __init__(self, base, reference):
        self.base, self.reference = base, reference

    def target_pose(self, identifier):
        if self.reference.key == ("target", identifier):
            return self.reference.value[:3, :3], self.reference.value[:3, 3]
        return self.base.target_pose(identifier)

    def camera_pose(self, identifier):
        if self.reference.key == ("camera", identifier):
            return self.reference.value[:3, :3], self.reference.value[:3, 3]
        return self.base.camera_pose(identifier)

    def segment_alignment(self):
        if self.reference.key == ("segment", None):
            return float(np.exp(self.reference.value[0])), self.reference.value[1:]
        return self.base.segment_alignment()


@dataclass(frozen=True)
class MarginalFactor:
    references: tuple
    matrix: np.ndarray
    offset: np.ndarray
    geometry: dict = field(default_factory=dict)  # None=all views; int=per target
    kind: str = field(default="marginal", init=False)

    def __post_init__(self):
        matrix, offset = np.asarray(self.matrix, dtype=float), np.asarray(self.offset, dtype=float)
        keys = [reference.key for reference in self.references]
        if (matrix.ndim != 2 or matrix.shape[1] != sum(reference.size for reference in self.references)
                or offset.shape != (matrix.shape[0],) or len(keys) != len(set(keys))
                or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(offset))):
            raise ValueError("invalid square-root marginal factor")
        object.__setattr__(self, "matrix", matrix.copy())
        object.__setattr__(self, "offset", offset.copy())

    def dependencies(self):
        return tuple(reference.key for reference in self.references)

    def residual(self, state):
        return self.matrix @ np.concatenate([reference.local(state) for reference in self.references]) + self.offset


def _evidence(factors, state):
    output, cameras = {}, {}
    for factor in factors:
        if isinstance(factor, MarginalFactor):
            for target, evidence in factor.geometry.items():
                output[target] = output.get(target, GeometryEvidence()).merge(evidence)
        elif factor.kind == "F1":
            cameras.setdefault(None, set()).add(factor.camera_id)
            cameras.setdefault(factor.target_id, set()).add(factor.camera_id)
        elif factor.kind == "F3":
            for target in (None, factor.target_id):
                output[target] = output.get(target, GeometryEvidence()).merge(GeometryEvidence(support=True))
    for target, ids in cameras.items():
        current = GeometryEvidence()
        for identifier in sorted(ids):
            position = tuple(np.asarray(state.camera_pose(identifier)[1], dtype=float))
            current = current.merge(GeometryEvidence(1, (position,)))
        output[target] = output.get(target, GeometryEvidence()).merge(current)
    return output


def marginalize_factors(factors, state, config, remove_keys, *, prior_inflation=1.):
    """Profile only outgoing factors plus the preceding prior, once each.

    Robust IRLS weights are frozen at the departing linearization point; hard
    outliers are omitted. The resulting Gaussian prior is never re-robustified.
    QR removes outgoing state columns, SVD compresses observable retained rows.
    """
    if not np.isfinite(prior_inflation) or prior_inflation < 1.:
        raise ValueError("prior_inflation must be finite and at least one")
    keys = {dependency for factor in factors for dependency in factor.dependencies()
            if dependency[0] == "target" or (dependency[0] == "camera" and config.optimize_camera_poses)
            or (dependency[0] == "segment" and config.optimize_segment_alignment)}
    keys = sorted(keys, key=lambda key: (key[0], -1 if key[1] is None else key[1]))
    references = [StateReference.capture(key, state) for key in keys]
    columns, cursor = {}, 0
    for reference in references:
        columns[reference.key] = slice(cursor, cursor + reference.size)
        cursor += reference.size
    blocks, jacobians, accepted = [], [], []
    for factor in factors:
        is_prior = isinstance(factor, MarginalFactor)
        scale = 1. if is_prior else np.sqrt(config.factor_weights.get(factor.kind, 1.))
        base = np.ravel(factor.residual(state)) * scale
        if not np.all(np.isfinite(base)):
            raise ValueError("nonfinite outgoing factor residual")
        if not is_prior and np.linalg.norm(base) > config.outlier_sigma * np.sqrt(len(base)):
            continue
        weight = np.ones(len(base)) if is_prior else np.sqrt(robust_rho((base / config.f_scale) ** 2, config.loss)[1])
        jacobian = np.zeros((len(base), cursor))
        for reference in references:
            if reference.key not in factor.dependencies():
                continue
            for axis in range(reference.size):
                delta = np.zeros(reference.size)
                delta[axis] = 1e-6
                plus = factor.residual(_LocalState(state, reference.retract(delta)))
                minus = factor.residual(_LocalState(state, reference.retract(-delta)))
                jacobian[:, columns[reference.key].start + axis] = (np.ravel(plus) - np.ravel(minus)) / 2e-6 * scale * weight
        blocks.append(base * weight)
        jacobians.append(jacobian)
        accepted.append(factor)
    if not blocks or not references:
        return None
    matrix, offset = np.vstack(jacobians), np.concatenate(blocks)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("nonfinite outgoing factor Jacobian")
    # Projection can leave tiny nonzero roundoff even when all rows are fully
    # explained by removed variables. Compare to the original problem scale.
    original_scale = float(np.linalg.norm(matrix))
    removed = [column for key in remove_keys if key in columns for column in range(columns[key].start, columns[key].stop)]
    retained = [reference for reference in references if reference.key not in remove_keys]
    keep = [column for reference in retained for column in range(columns[reference.key].start, columns[reference.key].stop)]
    if not keep:
        return None
    if removed:
        basis, triangular, _ = qr(matrix[:, removed], mode="economic", pivoting=True)
        diagonal = np.abs(np.diag(triangular))
        tolerance = max(1e-12, np.max(diagonal, initial=0.) * 1e-10)
        rank = int(np.count_nonzero(diagonal > tolerance))
        basis = basis[:, :rank]
        matrix = matrix[:, keep] - basis @ (basis.T @ matrix[:, keep])
        offset = offset - basis @ (basis.T @ offset)
    else:
        matrix = matrix[:, keep]
    u, singular, vh = np.linalg.svd(matrix, full_matrices=False)
    observable = singular > max(1e-12, original_scale * 1e-10, np.max(singular, initial=0.) * 1e-10)
    if not np.any(observable):
        return None
    root = singular[observable, None] * vh[observable] / prior_inflation
    residual = u[:, observable].T @ offset / prior_inflation
    return MarginalFactor(tuple(retained), root, residual, _evidence(accepted, state))
