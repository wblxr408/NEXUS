"""Small adapter for EPro-PnP pose samples or any weighted SE(3) sampler."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class PoseDistribution:
    mean_pose: np.ndarray
    covariance: np.ndarray
    effective_sample_size: float
    multimodal: bool


def distribution_from_samples(rotations: np.ndarray, translations_m: np.ndarray,
                              log_weights: np.ndarray) -> PoseDistribution:
    """Moment-match probability-PnP samples into the solver output contract."""
    rotations = np.asarray(rotations, dtype=float)
    translations = np.asarray(translations_m, dtype=float)
    weights = np.exp(np.asarray(log_weights, dtype=float) - np.max(log_weights))
    weights /= np.sum(weights)
    mean_rotation = Rotation.from_matrix(rotations).mean(weights=weights)
    mean_translation = np.sum(weights[:, None] * translations, axis=0)
    rotation_delta = (mean_rotation.inv() * Rotation.from_matrix(rotations)).as_rotvec()
    tangent = np.column_stack((rotation_delta, translations - mean_translation))
    covariance = (tangent * weights[:, None]).T @ tangent
    pose = np.eye(4); pose[:3, :3] = mean_rotation.as_matrix(); pose[:3, 3] = mean_translation
    effective = float(1.0 / np.sum(weights**2))
    eigenvalues = np.linalg.eigvalsh(covariance[:3, :3])
    multimodal = bool(np.max(eigenvalues) > np.deg2rad(20.0) ** 2)
    return PoseDistribution(pose, covariance, effective, multimodal)
