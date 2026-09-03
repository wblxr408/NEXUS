"""Sample-time target filtering; independent of ROS and platform UWB state.

The six-state model is [position_map_m, velocity_map_mps]. Predictions are
internal only: a rejected or missing observation never becomes a fresh pose.
Association/identity must be established upstream; this filter keys by track ID.
"""

from dataclasses import dataclass, field

import numpy as np

from .fusion import is_stale, validate_observation


def innovation_gate(innovation, covariance, soft_d2, hard_d2):
    """Return decision, squared Mahalanobis distance, covariance inflation.

    Thresholds are for the complete observation vector (3 DOF for positions),
    not for each axis. Solve with the full covariance, never its diagonal.
    """
    delta = np.asarray(innovation, dtype=float)
    matrix = np.asarray(covariance, dtype=float)
    if not 0 < soft_d2 < hard_d2:
        raise ValueError("innovation gates require 0 < soft_d2 < hard_d2")
    if (delta.ndim != 1 or matrix.shape != (delta.size, delta.size)
            or not np.all(np.isfinite(delta)) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix, matrix.T, atol=1e-9)):
        raise ValueError("innovation and covariance must be finite and compatible")
    whitened = np.linalg.solve(np.linalg.cholesky(matrix), delta)
    d2 = float(whitened @ whitened)
    if d2 > hard_d2:
        return "rejected", d2, float("inf")
    if d2 > soft_d2:
        return "suspect", d2, d2 / soft_d2
    return "accepted", d2, 1.0


@dataclass(frozen=True)
class RobustFilterConfig:
    expected_frame: str = "map"
    max_age_ns: int = 250_000_000
    prediction_horizon_ns: int = 500_000_000
    acceleration_sigma_mps2: float = 2.0
    initial_velocity_sigma_mps: float = 1.0
    soft_d2: float = 7.815
    hard_d2: float = 16.266
    confirmation_hits: int = 3
    maximum_covariance_scale: float = 100.0
    maximum_speed_mps: float = 10.0

    def __post_init__(self):
        values = (self.acceleration_sigma_mps2, self.initial_velocity_sigma_mps,
                  self.soft_d2, self.hard_d2, self.maximum_covariance_scale,
                  self.maximum_speed_mps)
        if (not self.expected_frame or self.max_age_ns <= 0
                or self.prediction_horizon_ns < self.max_age_ns
                or not np.all(np.isfinite(values)) or min(values) <= 0
                or self.soft_d2 >= self.hard_d2 or self.confirmation_hits < 1
                or self.maximum_covariance_scale < 1):
            raise ValueError("invalid target filter configuration")


@dataclass
class _Track:
    x: np.ndarray
    covariance: np.ndarray
    stamp_ns: int
    hits: int = 1
    status: str = "tentative"
    sources: dict = field(default_factory=dict)


class RobustTargetFilter:
    """Innovation-gated constant-velocity estimator with explicit recovery.

    After the prediction horizon, a new location requires a mutually consistent
    sequence before the old state is replaced. No unbounded extrapolation or
    automatic acceptance of the first observation after a long outage.
    """

    def __init__(self, config=None):
        self.config = config or RobustFilterConfig()
        self.tracks = {}
        self.last_source_stamps = {}
        self.recovery = {}
        self.last_diagnostic = {}
        self.counts = {"accepted": 0, "suspect": 0, "rejected": 0}

    def _reject(self, observation, reason, d2=None):
        self.counts["rejected"] += 1
        self.last_diagnostic = {"target_id": observation.target_id,
                                "source_mode": observation.source_mode,
                                "decision": "rejected", "reason": reason, "d2": d2}
        return None

    def _predict(self, track, stamp_ns):
        dt = (stamp_ns - track.stamp_ns) / 1e9
        transition = np.eye(6)
        transition[:3, 3:] = np.eye(3) * dt
        noise_map = np.vstack((np.eye(3) * dt ** 2 / 2, np.eye(3) * dt))
        process = noise_map @ noise_map.T * self.config.acceleration_sigma_mps2 ** 2
        return transition @ track.x, transition @ track.covariance @ transition.T + process

    def status(self, target_id, now_ns):
        track = self.tracks.get(target_id)
        if track is None:
            return "uninitialized"
        age = now_ns - track.stamp_ns
        if age < 0:
            return "invalid_clock"
        if age > self.config.max_age_ns:
            return "lost"
        return track.status

    def predict(self, target_id, stamp_ns):
        """Bounded internal prediction; always marked as unobserved."""
        track = self.tracks.get(target_id)
        if track is None or not 0 <= stamp_ns - track.stamp_ns <= self.config.prediction_horizon_ns:
            return None
        state, covariance = self._predict(track, stamp_ns)
        return {"state": state, "covariance": covariance, "predicted": True,
                "last_observed_stamp_ns": track.stamp_ns, "valid_observation": False}

    def update(self, observation, now_ns, *, covariance_scale=1.0):
        cfg = self.config
        if not validate_observation(observation, cfg.expected_frame):
            return self._reject(observation, "invalid_observation")
        if is_stale(observation.stamp_ns, now_ns, cfg.max_age_ns):
            return self._reject(observation, "stale_or_future")
        if observation.receive_timestamp_ns > now_ns:
            return self._reject(observation, "future_receive_timestamp")
        if not np.isfinite(covariance_scale) or not 1 <= covariance_scale <= cfg.maximum_covariance_scale:
            return self._reject(observation, "invalid_covariance_scale")
        key = (observation.target_id, observation.source_mode)
        if observation.stamp_ns <= self.last_source_stamps.get(key, 0):
            return self._reject(observation, "nonmonotonic_source_timestamp")
        track = self.tracks.get(observation.target_id)
        if track is not None and observation.stamp_ns < track.stamp_ns:
            return self._reject(observation, "out_of_sequence_observation")
        self.last_source_stamps[key] = observation.stamp_ns
        base_covariance = np.asarray(observation.covariance).reshape(3, 3)
        decision, d2, inflation = "accepted", 0.0, 1.0
        recovering = track is not None and observation.stamp_ns - track.stamp_ns > cfg.prediction_horizon_ns
        if recovering:
            previous = self.recovery.get(observation.target_id)
            hits = 1
            if previous is not None:
                old, old_hits = previous
                dt = (observation.stamp_ns - old.stamp_ns) / 1e9
                if 0 < dt <= cfg.prediction_horizon_ns / 1e9:
                    uncertainty = base_covariance + np.asarray(old.covariance).reshape(3, 3)
                    uncertainty += np.eye(3) * (cfg.initial_velocity_sigma_mps * dt) ** 2
                    gate, _, _ = innovation_gate(observation.position - old.position, uncertainty,
                                                 cfg.soft_d2, cfg.hard_d2)
                    if gate == "accepted":
                        hits = old_hits + 1
            self.recovery[observation.target_id] = (observation, hits)
            if hits < cfg.confirmation_hits:
                return self._reject(observation, "recovery_pending")
        if track is None or recovering:
            state = np.r_[observation.position, np.zeros(3)]
            covariance = np.zeros((6, 6))
            covariance[:3, :3] = base_covariance * covariance_scale
            covariance[3:, 3:] = np.eye(3) * cfg.initial_velocity_sigma_mps ** 2
            track = _Track(state, covariance, observation.stamp_ns)
            if recovering:
                track.hits, track.status = cfg.confirmation_hits, "re-associated"
                del self.recovery[observation.target_id]
            elif cfg.confirmation_hits == 1:
                track.status = "confirmed"
        else:
            predicted, uncertainty = self._predict(track, observation.stamp_ns)
            innovation = observation.position - predicted[:3]
            # Gate before learned inflation, so the model cannot rescue an
            # observation which violates the independent hard innovation gate.
            decision, d2, inflation = innovation_gate(
                innovation, uncertainty[:3, :3] + base_covariance, cfg.soft_d2, cfg.hard_d2)
            if decision == "rejected":
                return self._reject(observation, "innovation_outlier", d2)
            effective = base_covariance * min(cfg.maximum_covariance_scale, covariance_scale * inflation)
            gain = np.linalg.solve(uncertainty[:3, :3] + effective, uncertainty[:3, :]).T
            state = predicted + gain @ innovation
            if np.linalg.norm(state[3:]) > cfg.maximum_speed_mps:
                return self._reject(observation, "speed_limit", d2)
            correction = np.eye(6)
            correction[:, :3] -= gain
            covariance = correction @ uncertainty @ correction.T + gain @ effective @ gain.T
            track.x = state
            track.covariance = (covariance + covariance.T) / 2
            track.stamp_ns = observation.stamp_ns
            track.hits += 1
            track.status = ("degraded" if decision == "suspect" or covariance_scale > 1
                            else "confirmed" if track.hits >= cfg.confirmation_hits else "tentative")
        track.sources[observation.source_mode] = observation.stamp_ns
        track.sources = {source: stamp for source, stamp in track.sources.items()
                         if observation.stamp_ns - stamp <= cfg.max_age_ns}
        self.tracks[observation.target_id] = track
        self.counts[decision] += 1
        self.last_diagnostic = {"target_id": observation.target_id,
                                "source_mode": observation.source_mode,
                                "decision": decision, "reason": "", "d2": d2,
                                "covariance_scale": min(cfg.maximum_covariance_scale, covariance_scale * inflation),
                                "track_status": track.status}
        return {"target_id": observation.target_id, "frame_id": cfg.expected_frame,
                "stamp_ns": observation.stamp_ns, "position": track.x[:3].copy(),
                "velocity": track.x[3:].copy(), "covariance": track.covariance[:3, :3].copy(),
                "confidence": float(observation.confidence) / np.sqrt(covariance_scale * inflation),
                "source_mode": 4 if len(track.sources) > 1 else observation.source_mode,
                "orientation_xyzw": observation.orientation_xyzw / np.linalg.norm(observation.orientation_xyzw),
                "degraded_reason": "suspect_observation" if decision == "suspect" else "",
                "track_status": track.status, "predicted": False}
