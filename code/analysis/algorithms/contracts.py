from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class AlgorithmResult:
    """Common result contract for simulation and later ROS2 adapters."""

    estimate: np.ndarray
    covariance: np.ndarray
    algorithm: str
    family: str
    valid: bool = True
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_dict(self):
        return {
            "algorithm": self.algorithm,
            "family": self.family,
            "estimate": np.asarray(self.estimate, dtype=float).tolist(),
            "covariance": np.asarray(self.covariance, dtype=float).tolist(),
            "valid": self.valid,
            "metadata": self.metadata,
        }
