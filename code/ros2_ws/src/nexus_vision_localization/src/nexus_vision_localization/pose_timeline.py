"""Image-time platform poses with bounded interpolation and no extrapolation."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .reference_geometry import pose_covariance, rigid_transform


@dataclass(frozen=True)
class TimedPose:
    stamp_ns: int
    transform: np.ndarray
    covariance: np.ndarray  # translation and fixed-axis rotation in map
    interpolated: bool = False

    def __post_init__(self):
        if not isinstance(self.stamp_ns, int) or isinstance(self.stamp_ns, bool) or self.stamp_ns <= 0:
            raise ValueError("platform timestamp must be a positive integer")
        object.__setattr__(self, "transform", rigid_transform(self.transform))
        object.__setattr__(self, "covariance", pose_covariance(self.covariance))


def _interpolate(first, second, fraction):
    result = np.eye(4)
    result[:3, 3] = first[:3, 3] * (1 - fraction) + second[:3, 3] * fraction
    delta = Rotation.from_matrix(first[:3, :3].T @ second[:3, :3]).as_rotvec()
    if np.linalg.norm(delta) > np.pi - .01:
        raise ValueError("platform interpolation crosses the rotation branch cut")
    result[:3, :3] = first[:3, :3] @ Rotation.from_rotvec(delta * fraction).as_matrix()
    return result


class PoseTimeline:
    def __init__(self, maximum_gap_s=.25, retention_s=5., maximum_samples=256,
                 acceleration_sigma_mps2=2., angular_acceleration_sigma_rps2=2.):
        values = [maximum_gap_s, retention_s, maximum_samples, acceleration_sigma_mps2, angular_acceleration_sigma_rps2]
        if not all(np.isfinite(value) and value > 0 for value in values) or maximum_samples < 2 or int(maximum_samples) != maximum_samples:
            raise ValueError("invalid platform pose timeline bounds/noise")
        self.maximum_gap_ns, self.retention_ns = int(maximum_gap_s * 1e9), int(retention_s * 1e9)
        self.maximum_samples = int(maximum_samples)
        self.acceleration_sigmas = np.array([acceleration_sigma_mps2] * 3 + [angular_acceleration_sigma_rps2] * 3)
        self.samples = {}

    def add(self, sample):
        if not isinstance(sample, TimedPose):
            raise TypeError("platform timeline requires TimedPose")
        if self.samples and sample.stamp_ns < max(self.samples) - self.retention_ns:
            raise ValueError("platform sample is outside the retained timeline")
        self.samples[sample.stamp_ns] = sample
        cutoff = max(self.samples) - self.retention_ns
        self.samples = {stamp: self.samples[stamp] for stamp in sorted(self.samples) if stamp >= cutoff}
        while len(self.samples) > self.maximum_samples:
            del self.samples[min(self.samples)]

    def at(self, stamp_ns):
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns <= 0:
            raise ValueError("requested platform timestamp must be a positive integer")
        if stamp_ns in self.samples:
            return self.samples[stamp_ns]
        stamps = sorted(self.samples)
        index = int(np.searchsorted(stamps, stamp_ns))
        if index == 0 or index == len(stamps):
            raise ValueError("platform_pose_not_bracketed")
        first, second = self.samples[stamps[index - 1]], self.samples[stamps[index]]
        gap = second.stamp_ns - first.stamp_ns
        if gap > self.maximum_gap_ns:
            raise ValueError("platform_pose_gap_too_large")
        fraction = (stamp_ns - first.stamp_ns) / gap
        output = _interpolate(first.transform, second.transform, fraction)
        jacobians = []
        epsilon = 1e-5
        for component in range(2):
            jacobian = np.zeros((6, 6))
            jacobian[:3, :3] = np.eye(3) * (1 - fraction if component == 0 else fraction)
            for axis in range(3):
                plus = [first.transform.copy(), second.transform.copy()]
                minus = [first.transform.copy(), second.transform.copy()]
                delta = np.eye(3)[axis] * epsilon
                plus[component][:3, :3] = Rotation.from_rotvec(delta).as_matrix() @ plus[component][:3, :3]
                minus[component][:3, :3] = Rotation.from_rotvec(-delta).as_matrix() @ minus[component][:3, :3]
                a, b = _interpolate(*plus, fraction), _interpolate(*minus, fraction)
                jacobian[3:, axis + 3] = Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).as_rotvec() / (2 * epsilon)
            jacobians.append(jacobian)
        # Unknown endpoint correlation: covariance of a+b <= 2(Pa+Pb).
        covariance = 2 * sum(j @ p @ j.T for j, p in zip(jacobians, (first.covariance, second.covariance)))
        dt_left, dt_right = (stamp_ns - first.stamp_ns) / 1e9, (second.stamp_ns - stamp_ns) / 1e9
        interpolation_sigma = .5 * dt_left * dt_right * self.acceleration_sigmas
        covariance += np.diag(interpolation_sigma ** 2)
        return TimedPose(stamp_ns, output, (covariance + covariance.T) / 2, interpolated=True)
