"""Square-root Schur marginalization of only the outgoing graph factors."""

from dataclasses import dataclass

import numpy as np

from .platform_state import inverse_right_jacobian_so3


def schur_square_root(jacobian, residual, remove_columns):
    """Return A,b with ||A dx_keep+b||² equal to the marginalized quadratic.

    A constant independent of retained variables is omitted. Rank-deficient
    directions retain zero information instead of being regularized secretly.
    """
    jacobian = np.asarray(jacobian, dtype=float)
    residual = np.asarray(residual, dtype=float)
    if (jacobian.ndim != 2 or residual.shape != (jacobian.shape[0],)
            or not np.all(np.isfinite(jacobian)) or not np.all(np.isfinite(residual))):
        raise ValueError("marginalization requires finite compatible Jacobian and residual")
    remove = np.asarray(remove_columns, dtype=int)
    if (remove.ndim != 1 or len(np.unique(remove)) != len(remove)
            or np.any(remove < 0) or np.any(remove >= jacobian.shape[1])):
        raise ValueError("invalid marginalization column indices")
    keep = np.setdiff1d(np.arange(jacobian.shape[1]), remove)
    dropped, retained = jacobian[:, remove], jacobian[:, keep]
    # Orthogonal projection is the square-root form of the Schur complement
    # and avoids explicitly subtracting two ill-conditioned Hessians.
    if dropped.size:
        u, singular, _ = np.linalg.svd(dropped, full_matrices=False)
        threshold = (singular[0] if singular.size else 0.) * 1e-10
        basis = u[:, singular > threshold]
        retained = retained - basis @ (basis.T @ retained)
        residual = residual - basis @ (basis.T @ residual)
    u, singular, vh = np.linalg.svd(retained, full_matrices=False)
    threshold = max(1e-12, (singular[0] if singular.size else 0.) * 1e-10)
    observed = singular > threshold
    matrix = singular[observed, None] * vh[observed]
    offset = u[:, observed].T @ residual
    return matrix, offset, keep


@dataclass(frozen=True)
class MarginalPrior:
    references: tuple
    matrix: np.ndarray
    offset: np.ndarray

    @property
    def stamps(self):
        return tuple(state.stamp_ns for state in self.references)

    def residual(self, states):
        delta = np.concatenate([reference.local(states[reference.stamp_ns]) for reference in self.references])
        return self.matrix @ delta + self.offset

    def jacobians(self, states):
        result = {}
        for index, reference in enumerate(self.references):
            matrix = self.matrix[:, index * 15:(index + 1) * 15].copy()
            angle = reference.local(states[reference.stamp_ns])[6:9]
            matrix[:, 6:9] = matrix[:, 6:9] @ inverse_right_jacobian_so3(angle)
            result[reference.stamp_ns] = matrix
        return result


def numerical_jacobian(function, references, stamps=None, epsilon=1e-6):
    """Central local-manifold differences, used for gates and outgoing priors."""
    stamps = tuple(references) if stamps is None else tuple(stamps)
    base = np.asarray(function(references), dtype=float)
    jacobian = np.zeros((len(base), len(stamps) * 15))
    for index, stamp in enumerate(stamps):
        for axis in range(15):
            delta = np.zeros(15)
            delta[axis] = epsilon
            plus, minus = dict(references), dict(references)
            plus[stamp] = references[stamp].retract(delta)
            minus[stamp] = references[stamp].retract(-delta)
            jacobian[:, index * 15 + axis] = (function(plus) - function(minus)) / (2 * epsilon)
    return base, jacobian
