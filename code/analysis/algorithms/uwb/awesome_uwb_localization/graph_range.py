"""UWB-only 3D range graph adapted from qxiaofan/awesome-uwb-localization.

The upstream ROS1 node uses fixed SE3 anchor vertices, moving tag vertices,
``EdgeSE3Range`` residuals, trajectory edges, a Cauchy kernel, and g2o
Levenberg-Marquardt optimization.  This dependency-free adapter preserves
that UWB-only objective for the NEXUS observation contract.
"""

from __future__ import annotations

import time

import numpy as np

from algorithms.contracts import AlgorithmResult


class RangeGraphOptimizer3D:
    """Sliding-window LM solver for fixed-anchor 3D range edges."""

    def __init__(
        self,
        initial_xyz,
        *,
        trajectory_length=10,
        maximum_iterations=10,
        maximum_velocity_mps=5.0,
        range_sigma_floor_m=0.01,
        cauchy_scale=2.0,
    ):
        initial = np.asarray(initial_xyz, dtype=float).reshape(-1)
        if initial.size != 3 or not np.all(np.isfinite(initial)):
            raise ValueError("initial_xyz must contain three finite coordinates")
        if trajectory_length < 1 or maximum_iterations < 1:
            raise ValueError("trajectory_length and maximum_iterations must be positive")
        self.initial_xyz = initial
        self.trajectory_length = int(trajectory_length)
        self.maximum_iterations = int(maximum_iterations)
        self.maximum_velocity_mps = float(maximum_velocity_mps)
        self.range_sigma_floor_m = float(range_sigma_floor_m)
        self.cauchy_scale = float(cauchy_scale)
        self.frames = []
        self.positions = np.empty((0, 3), dtype=float)
        self.anchor_ids = None

    @staticmethod
    def _parse_frame(observation_frame):
        samples = observation_frame.ranges
        if len(samples) < 4:
            raise ValueError("at least four 3D range edges are required")
        anchor_ids = tuple(sample.anchor_id for sample in samples)
        if len(set(anchor_ids)) != len(anchor_ids):
            raise ValueError("anchor IDs must be unique within a frame")
        anchors = np.asarray([sample.anchor_position_m for sample in samples], dtype=float)
        ranges = np.asarray([sample.range_m for sample in samples], dtype=float)
        sigmas = np.asarray([sample.stddev_m for sample in samples], dtype=float)
        if anchors.shape != (len(samples), 3):
            raise ValueError("3D anchor positions are required")
        if not np.all(np.isfinite(anchors)):
            raise ValueError("anchor positions must be finite")
        if np.any(ranges <= 0.0) or not np.all(np.isfinite(ranges)):
            raise ValueError("ranges must be positive and finite")
        if np.any(sigmas < 0.0) or not np.all(np.isfinite(sigmas)):
            raise ValueError("range standard deviations must be finite and non-negative")
        if np.linalg.matrix_rank(anchors[1:] - anchors[0]) < 3:
            raise ValueError("range graph anchors are coplanar or rank deficient")
        return anchor_ids, anchors, ranges, sigmas

    @staticmethod
    def _range_initialization(anchors, ranges):
        """Bootstrap a tag vertex from its fixed-anchor range edges."""
        matrix = anchors[1:] - anchors[0]
        anchor_norms = np.sum(anchors ** 2, axis=1)
        rhs = (
            ranges[0] ** 2
            - ranges[1:] ** 2
            + anchor_norms[1:]
            - anchor_norms[0]
        ) / 2.0
        estimate, _, rank, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
        if rank < 3 or not np.all(np.isfinite(estimate)):
            raise ValueError("range graph anchors do not constrain a 3D position")
        return estimate

    def _residuals_and_jacobian(self, flat_positions):
        positions = np.asarray(flat_positions, dtype=float).reshape((-1, 3))
        residuals = []
        rows = []

        for frame_index, frame in enumerate(self.frames):
            _, anchors, ranges, sigmas = frame
            delta = positions[frame_index] - anchors
            predicted = np.maximum(np.linalg.norm(delta, axis=1), 1e-9)
            sigma = np.maximum(sigmas, self.range_sigma_floor_m)
            for anchor_index in range(len(ranges)):
                residuals.append((ranges[anchor_index] - predicted[anchor_index]) / sigma[anchor_index])
                row = np.zeros(positions.size, dtype=float)
                start = 3 * frame_index
                row[start:start + 3] = -delta[anchor_index] / (
                    predicted[anchor_index] * sigma[anchor_index]
                )
                rows.append(row)

        # The upstream graph adds a zero-range edge between successive poses
        # for a moving tag.  This equivalent soft displacement residual keeps
        # the 3D trajectory locally smooth while retaining the range objective.
        for frame_index in range(1, len(self.frames)):
            previous_stamp = self.frames[frame_index - 1][0]
            current_stamp = self.frames[frame_index][0]
            dt = max((current_stamp - previous_stamp) / 1e9, 1e-3)
            process_sigma = max(self.maximum_velocity_mps * dt / 3.0, 0.02)
            delta = positions[frame_index] - positions[frame_index - 1]
            distance = max(float(np.linalg.norm(delta)), 1e-9)
            residuals.append(distance / process_sigma)
            row = np.zeros(positions.size, dtype=float)
            direction = delta / (distance * process_sigma)
            row[3 * (frame_index - 1):3 * frame_index] = -direction
            row[3 * frame_index:3 * (frame_index + 1)] = direction
            rows.append(row)

        return np.asarray(residuals), np.asarray(rows)

    def _cost(self, residuals):
        scaled = residuals / self.cauchy_scale
        return float(np.sum(self.cauchy_scale ** 2 * np.log1p(scaled ** 2)))

    def _optimize(self, initial):
        state = np.asarray(initial, dtype=float).reshape(-1)
        damping = 1e-3
        iterations = 0
        for iterations in range(1, self.maximum_iterations + 1):
            residuals, jacobian = self._residuals_and_jacobian(state)
            weights = 1.0 / np.sqrt(1.0 + (residuals / self.cauchy_scale) ** 2)
            weighted_jacobian = jacobian * weights[:, None]
            weighted_residuals = residuals * weights
            normal = weighted_jacobian.T @ weighted_jacobian
            gradient = weighted_jacobian.T @ weighted_residuals
            diagonal = np.maximum(np.diag(normal), 1.0)
            system = normal + damping * np.diag(diagonal)
            try:
                step = np.linalg.solve(system, -gradient)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(system, -gradient, rcond=None)[0]
            candidate = state + step
            candidate_residuals, _ = self._residuals_and_jacobian(candidate)
            if self._cost(candidate_residuals) < self._cost(residuals):
                state = candidate
                damping = max(damping / 3.0, 1e-9)
                if np.linalg.norm(step) < 1e-7:
                    break
            else:
                damping = min(damping * 10.0, 1e9)

        residuals, jacobian = self._residuals_and_jacobian(state)
        weights = 1.0 / (1.0 + (residuals / self.cauchy_scale) ** 2)
        information = jacobian.T @ (jacobian * weights[:, None])
        covariance = np.linalg.pinv(information)[-3:, -3:]
        return state.reshape((-1, 3)), covariance, iterations, self._cost(residuals)

    def add(self, observation_frame):
        anchor_ids, anchors, ranges, sigmas = self._parse_frame(observation_frame)
        if self.anchor_ids is None:
            self.anchor_ids = anchor_ids
        elif anchor_ids != self.anchor_ids:
            raise ValueError("anchor order changed within the graph window")
        if self.frames and observation_frame.timestamp_ns <= self.frames[-1][0]:
            raise ValueError("observation timestamps must be strictly increasing")

        self.frames.append((int(observation_frame.timestamp_ns), anchors, ranges, sigmas))
        range_initial = self._range_initialization(anchors, ranges)
        if self.positions.size == 0:
            initial = range_initial.reshape((1, 3))
        else:
            initial = np.vstack((self.positions, range_initial))
        if len(self.frames) > self.trajectory_length:
            self.frames.pop(0)
            initial = initial[1:]

        self.positions, covariance, iterations, chi2 = self._optimize(initial)
        return self.positions[-1].copy(), covariance, iterations, chi2


def run_graph_range(
    *,
    observation_frame,
    initial_xyz=(0.0, 0.0, 2.0),
    initial_xy=None,
    trajectory_length=10,
    maximum_iterations=10,
    **_,
):
    """Run one UWB epoch through the stateful UWB-only 3D graph adapter."""
    start_ns = time.monotonic_ns()
    try:
        if initial_xy is not None:
            initial_xyz = (float(initial_xy[0]), float(initial_xy[1]), float(initial_xyz[2]))
        state = getattr(run_graph_range, "_state", None)
        if state is None:
            state = RangeGraphOptimizer3D(
                initial_xyz,
                trajectory_length=trajectory_length,
                maximum_iterations=maximum_iterations,
            )
            run_graph_range._state = state
        xyz, covariance, iterations, chi2 = state.add(observation_frame)
        valid = True
        metadata = {
            "dimension": "3D",
            "timestamp_ns": observation_frame.timestamp_ns,
            "window_size": len(state.frames),
            "iterations": iterations,
            "robust_kernel": "cauchy",
            "chi2": chi2,
        }
    except (ValueError, np.linalg.LinAlgError) as exc:
        xyz = np.full(3, np.nan)
        covariance = np.full((3, 3), np.nan)
        valid = False
        metadata = {"error": str(exc), "timestamp_ns": observation_frame.timestamp_ns}
    metadata["runtime_ms"] = (time.monotonic_ns() - start_ns) / 1e6
    return AlgorithmResult(
        estimate=np.asarray(xyz),
        covariance=np.asarray(covariance),
        algorithm="uwb.awesome_uwb",
        family="uwb",
        valid=valid,
        metadata=metadata,
    )
