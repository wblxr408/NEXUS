"""Correlation-aware target p/v/SO(3) estimates, independent of ROS.

Neighboring multiview solutions share image, platform and calibration errors.
Covariance intersection (CI) avoids treating those solutions as independent
measurements. Unknown velocity/orientation remains absent; no fake quaternion
or zero-variance component is inserted to make an older pose interface fit.
"""

from dataclasses import dataclass, field, replace

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.stats import chi2

from .platform_state import (
    covariance_matrix, exp_so3, inverse_right_jacobian_so3, log_so3,
    right_jacobian_so3, rotation_matrix, vector,
)
from .robust_filter import innovation_gate


@dataclass(frozen=True)
class KinematicEstimate:
    target_id: str
    stamp_ns: int
    receive_ns: int
    source: str
    position_reference: str
    position: np.ndarray
    covariance: np.ndarray  # 9x9 [p_map, v_map, angle_map]; unknown rows NaN
    velocity: np.ndarray | None = None
    rotation: np.ndarray | None = None
    confidence: float = 1.
    frame_id: str = "map"
    unit: str = "m"
    state: str = "tentative"

    def __post_init__(self):
        for name in ("position", "covariance", "velocity", "rotation"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, np.array(value, dtype=float, copy=True))

    @property
    def indices(self):
        return np.r_[np.arange(3), np.arange(3, 6) if self.velocity is not None else np.array([], dtype=int),
                     np.arange(6, 9) if self.rotation is not None else np.array([], dtype=int)].astype(int)

    def validate(self, expected_frame):
        if (not isinstance(self.target_id, str) or not self.target_id
                or not isinstance(self.source, str) or not self.source
                or not isinstance(self.position_reference, str) or not self.position_reference
                or self.frame_id != expected_frame or self.unit != "m"
                or self.state not in {"tentative", "confirmed", "degraded", "re-associated"}
                or not np.isfinite(self.confidence) or not 0 < self.confidence <= 1):
            raise ValueError("invalid_kinematic_identity_frame_or_confidence")
        for stamp in (self.stamp_ns, self.receive_ns):
            if not isinstance(stamp, (int, np.integer)) or isinstance(stamp, bool) or stamp <= 0:
                raise ValueError("invalid_kinematic_timestamp")
        if self.receive_ns < self.stamp_ns:
            raise ValueError("receive_precedes_sample")
        vector(self.position, 3, "target position")
        if self.velocity is not None:
            vector(self.velocity, 3, "target velocity")
        if self.rotation is not None:
            rotation_matrix(self.rotation)
        covariance = np.asarray(self.covariance, dtype=float)
        if covariance.shape != (9, 9):
            raise ValueError("kinematic_covariance_requires_9x9")
        known = self.indices
        covariance_matrix(covariance[np.ix_(known, known)], len(known))
        unknown = np.setdiff1d(np.arange(9), known)
        if unknown.size and (not np.all(np.isnan(covariance[unknown, :]))
                             or not np.all(np.isnan(covariance[:, unknown]))):
            raise ValueError("unobserved_covariance_rows_must_be_nan")
        return self


@dataclass(frozen=True)
class KinematicFilterConfig:
    frame_id: str = "map"
    max_age_ns: int = 250_000_000
    prediction_horizon_ns: int = 500_000_000
    retention_ns: int = 5_000_000_000
    acceleration_sigma_mps2: float = 2.
    unknown_velocity_sigma_mps: float = 1.
    angular_rate_sigma_rps: float = .5
    maximum_speed_mps: float = 10.
    soft_probability: float = .95
    hard_probability: float = .999
    maximum_covariance_scale: float = 100.
    confirmation_hits: int = 3
    maximum_tracks: int = 64
    maximum_sources: int = 8

    def __post_init__(self):
        values = (self.acceleration_sigma_mps2, self.unknown_velocity_sigma_mps,
                  self.angular_rate_sigma_rps, self.maximum_speed_mps, self.maximum_covariance_scale)
        if (not self.frame_id or not 0 < self.max_age_ns <= self.prediction_horizon_ns <= self.retention_ns
                or not np.all(np.isfinite(values)) or min(values) <= 0 or self.maximum_covariance_scale < 1
                or not 0 < self.soft_probability < self.hard_probability < 1
                or self.confirmation_hits < 1 or self.maximum_tracks < 1 or self.maximum_sources < 1):
            raise ValueError("invalid_kinematic_filter_configuration")


def _coordinates(estimate, origin):
    """Map-fixed angular covariance -> the common local Log coordinates."""
    mean = np.zeros(9)
    mean[:3] = estimate.position - origin.position
    if estimate.velocity is not None:
        mean[3:6] = estimate.velocity - (origin.velocity if origin.velocity is not None else 0.)
    transform = np.eye(9)
    if estimate.rotation is not None:
        reference = origin.rotation if origin.rotation is not None else estimate.rotation
        mean[6:] = log_so3(estimate.rotation @ reference.T)
        if np.linalg.norm(mean[6:]) >= np.pi - 1e-3:
            raise ValueError("target_rotation_log_ambiguous")
        transform[6:, 6:] = inverse_right_jacobian_so3(-mean[6:])
    indices = estimate.indices
    jacobian = transform[np.ix_(indices, indices)]
    covariance = jacobian @ estimate.covariance[np.ix_(indices, indices)] @ jacobian.T
    return mean, covariance


def intersect_estimates(predicted, observation, covariance_scale=1.):
    """Local Gaussian CI with complete cross blocks and partial observations.

    Minimize log(det(P)) over one scalar weight, including admissible endpoints.
    A source missing a union component supplies zero information in that
    component, rather than a fabricated finite prior. SO(3) is a local model.
    """
    origin = predicted if predicted.rotation is not None else replace(predicted, rotation=observation.rotation)
    left_mean, left_covariance = _coordinates(predicted, origin)
    right_mean, right_covariance = _coordinates(observation, origin)
    union = np.union1d(predicted.indices, observation.indices)
    information, vectors = [], []
    inputs = ((predicted, left_mean, left_covariance, 1.),
              (observation, right_mean, right_covariance, covariance_scale))
    for estimate, mean, covariance, scale in inputs:
        indices = np.searchsorted(union, estimate.indices)
        inverse = np.linalg.solve(covariance * scale, np.eye(len(indices)))
        embedded = np.zeros((len(union), len(union)))
        embedded[np.ix_(indices, indices)] = inverse
        information.append(embedded)
        vectors.append(embedded @ mean[union])

    def objective(weight):
        matrix = weight * information[0] + (1 - weight) * information[1]
        try:
            root = np.linalg.cholesky(matrix)
        except np.linalg.LinAlgError:
            return float("inf")
        return float(-2 * np.log(np.diag(root)).sum())

    optimum = minimize_scalar(objective, bounds=(0., 1.), method="bounded", options={"xatol": 1e-5, "maxiter": 40})
    weights = [.5, float(optimum.x), 0., 1.]
    scores = [objective(weight) for weight in weights]
    best = min(scores)
    if not np.isfinite(best):
        raise ValueError("kinematic_intersection_unobservable")
    # Equal information must not gain certainty merely from a new timestamp.
    weight = next(weight for weight, score in zip(weights, scores) if score <= best + 1e-9)
    matrix = weight * information[0] + (1 - weight) * information[1]
    local_covariance = np.linalg.solve(matrix, np.eye(len(union)))
    local = np.zeros(9)
    local[union] = np.linalg.solve(matrix, weight * vectors[0] + (1 - weight) * vectors[1])
    rotation = exp_so3(local[6:]) @ origin.rotation if 6 in union else None
    velocity = local[3:6] + (origin.velocity if origin.velocity is not None else 0.) if 3 in union else None
    reset = np.eye(9)
    reset[6:, 6:] = right_jacobian_so3(-local[6:])
    jacobian = reset[np.ix_(union, union)]
    covariance = np.full((9, 9), np.nan)
    block = jacobian @ local_covariance @ jacobian.T
    covariance[np.ix_(union, union)] = (block + block.T) / 2
    result = replace(observation, position=origin.position + local[:3], velocity=velocity,
                     rotation=rotation, covariance=covariance)
    return result, weight


@dataclass
class _Track:
    estimate: KinematicEstimate
    hits: int = 1
    component_stamps: dict = field(default_factory=dict)
    invalid_stamp_ns: int = 0
    invalid_reason: str = ""
    identity_uncertain: bool = False


class KinematicTargetFilter:
    def __init__(self, config=None):
        self.config = config or KinematicFilterConfig()
        self.tracks = {}
        self.recovery = {}
        self.source_stamps = {}
        self.last_diagnostic = {}
        self.counts = {"accepted": 0, "suspect": 0, "rejected": 0}

    def expire(self, now_ns):
        for identifier, track in list(self.tracks.items()):
            if now_ns - track.estimate.stamp_ns > self.config.retention_ns:
                del self.tracks[identifier]
                self.recovery.pop(identifier, None)
                self.source_stamps.pop(identifier, None)

    def _reject(self, observation, reason, d2=None):
        self.counts["rejected"] += 1
        self.last_diagnostic = {"target_id": observation.target_id, "source": observation.source,
                                "decision": "rejected", "reason": str(reason), "d2": d2}
        return None

    def _predict(self, estimate, stamp_ns):
        dt = (stamp_ns - estimate.stamp_ns) / 1e9
        known = estimate.indices
        transition = np.eye(9)
        noise = np.zeros((9, 9))
        position = estimate.position.copy()
        if estimate.velocity is not None:
            position += dt * estimate.velocity
            transition[:3, 3:6] = np.eye(3) * dt
            mapping = np.zeros((9, 3))
            mapping[:3], mapping[3:6] = np.eye(3) * dt ** 2 / 2, np.eye(3) * dt
            noise += mapping @ mapping.T * self.config.acceleration_sigma_mps2 ** 2
        else:
            variance = (self.config.unknown_velocity_sigma_mps * dt) ** 2 + (self.config.acceleration_sigma_mps2 * dt ** 2 / 2) ** 2
            noise[:3, :3] = np.eye(3) * variance
        if estimate.rotation is not None:
            noise[6:, 6:] = np.eye(3) * (self.config.angular_rate_sigma_rps * dt) ** 2
        jacobian = transition[np.ix_(known, known)]
        covariance = np.full((9, 9), np.nan)
        covariance[np.ix_(known, known)] = (
            jacobian @ estimate.covariance[np.ix_(known, known)] @ jacobian.T + noise[np.ix_(known, known)])
        return replace(estimate, position=position, covariance=covariance, stamp_ns=stamp_ns,
                       receive_ns=max(estimate.receive_ns, stamp_ns))

    def _available(self, track, stamp_ns):
        estimate = track.estimate
        velocity = estimate.velocity if stamp_ns - track.component_stamps.get("velocity", 0) <= self.config.prediction_horizon_ns else None
        rotation = estimate.rotation if stamp_ns - track.component_stamps.get("rotation", 0) <= self.config.prediction_horizon_ns else None
        current = replace(estimate, velocity=velocity, rotation=rotation)
        covariance = np.full((9, 9), np.nan)
        covariance[np.ix_(current.indices, current.indices)] = estimate.covariance[np.ix_(current.indices, current.indices)]
        return replace(current, covariance=covariance)

    def prediction_block_reason(self, identifier):
        track = self.tracks.get(identifier)
        if track is not None and track.identity_uncertain:
            return "target_identity_ambiguous"
        pending = self.recovery.get(identifier)
        if track is not None and pending is not None and pending[0].position_reference != track.estimate.position_reference:
            return "reference_change_pending"
        return ""

    def predict(self, identifier, now_ns):
        track = self.tracks.get(identifier)
        if (track is None or self.prediction_block_reason(identifier)
                or not 0 <= now_ns - track.estimate.stamp_ns <= self.config.prediction_horizon_ns):
            return None
        estimate = self._predict(self._available(track, now_ns), now_ns)
        return {"estimate": estimate, "valid": False, "predicted": True,
                "last_observed_stamp_ns": track.estimate.stamp_ns, "reason": "prediction_only"}

    def invalidate(self, identifier, stamp_ns, reason, now_ns):
        track = self.tracks.get(identifier)
        if (track is None or not 0 < stamp_ns <= now_ns
                or stamp_ns < max(track.estimate.stamp_ns, track.invalid_stamp_ns)):
            return False
        track.invalid_stamp_ns, track.invalid_reason = stamp_ns, str(reason)
        if "identity_ambiguous" in reason:
            track.identity_uncertain = True
            self.recovery.pop(identifier, None)
        return True

    def _gate(self, predicted, observation):
        origin = predicted if predicted.rotation is not None else replace(predicted, rotation=observation.rotation)
        left, p = _coordinates(predicted, origin)
        right, r = _coordinates(observation, origin)
        common = np.intersect1d(predicted.indices, observation.indices)
        first, second = np.searchsorted(predicted.indices, common), np.searchsorted(observation.indices, common)
        # Unknown cross-correlation: 2(P+R) bounds the difference covariance.
        covariance = 2 * (p[np.ix_(first, first)] + r[np.ix_(second, second)])
        return innovation_gate(right[common] - left[common], covariance,
                               chi2.ppf(self.config.soft_probability, len(common)),
                               chi2.ppf(self.config.hard_probability, len(common)))

    def update(self, observation, now_ns, *, covariance_scale=1.):
        cfg = self.config
        try:
            observation.validate(cfg.frame_id)
            if not 0 <= now_ns - observation.stamp_ns <= cfg.max_age_ns or observation.receive_ns > now_ns:
                raise ValueError("stale_or_future_kinematic_observation")
            if not np.isfinite(covariance_scale) or not 1 <= covariance_scale <= cfg.maximum_covariance_scale:
                raise ValueError("invalid_covariance_scale")
            if observation.velocity is not None and np.linalg.norm(observation.velocity) > cfg.maximum_speed_mps:
                raise ValueError("target_speed_limit")
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            return self._reject(observation, error)
        self.expire(now_ns)
        track = self.tracks.get(observation.target_id)
        stamps = self.source_stamps.get(observation.target_id, {})
        if observation.stamp_ns <= stamps.get(observation.source, 0):
            return self._reject(observation, "nonmonotonic_source_timestamp")
        if track is not None and observation.stamp_ns < max(track.estimate.stamp_ns, track.invalid_stamp_ns):
            return self._reject(observation, "out_of_sequence_observation")
        if (track is None and len(self.tracks) >= cfg.maximum_tracks
                or observation.source not in stamps and len(stamps) >= cfg.maximum_sources):
            return self._reject(observation, "target_or_source_capacity")
        self.source_stamps.setdefault(observation.target_id, {})[observation.source] = observation.stamp_ns
        recovering = track is not None and (
            observation.stamp_ns - track.estimate.stamp_ns > cfg.prediction_horizon_ns
            or track.identity_uncertain or observation.position_reference != track.estimate.position_reference)
        if recovering:
            previous, hits = self.recovery.get(observation.target_id, (None, 0))
            same = previous is not None and previous.position_reference == observation.position_reference and previous.source == observation.source
            dt = observation.stamp_ns - previous.stamp_ns if same else 0
            if same and 0 < dt <= cfg.prediction_horizon_ns:
                try:
                    decision, _, _ = self._gate(self._predict(previous, observation.stamp_ns), observation)
                except (ValueError, np.linalg.LinAlgError) as error:
                    return self._reject(observation, error)
                hits = hits + 1 if decision == "accepted" else 1
            else:
                hits = 1
            self.recovery[observation.target_id] = (observation, hits)
            if hits < cfg.confirmation_hits:
                return self._reject(observation, "reference_change_pending" if observation.position_reference != track.estimate.position_reference else "recovery_pending")
        decision, d2, inflation, weight = "accepted", 0., 1., None
        if track is None or recovering:
            estimate = replace(observation, position=observation.position.copy(),
                               velocity=None if observation.velocity is None else observation.velocity.copy(),
                               rotation=None if observation.rotation is None else observation.rotation.copy(),
                               covariance=observation.covariance.copy() * covariance_scale)
            hits = cfg.confirmation_hits if recovering else 1
            components = {}
        else:
            predicted = self._predict(self._available(track, observation.stamp_ns), observation.stamp_ns)
            try:
                # Hard gate uses the uninflated current measurement. Learning
                # cannot rescue an observation already rejected by geometry.
                decision, d2, inflation = self._gate(predicted, observation)
                if decision == "rejected":
                    return self._reject(observation, "kinematic_innovation_outlier", d2)
                effective = min(cfg.maximum_covariance_scale, covariance_scale * inflation)
                estimate, weight = intersect_estimates(predicted, observation, effective)
                estimate.validate(cfg.frame_id)
                if estimate.velocity is not None and np.linalg.norm(estimate.velocity) > cfg.maximum_speed_mps:
                    raise ValueError("target_speed_limit")
            except (ValueError, np.linalg.LinAlgError) as error:
                return self._reject(observation, error, d2)
            hits = track.hits + int(observation.stamp_ns > track.estimate.stamp_ns)
            components = dict(track.component_stamps)
        for name in ("velocity", "rotation"):
            if getattr(observation, name) is not None:
                components[name] = observation.stamp_ns
        state = ("tentative" if observation.state == "tentative"
                 else "degraded" if decision == "suspect" or covariance_scale > 1 or observation.state == "degraded"
                 else "re-associated" if recovering or observation.state == "re-associated"
                 else "confirmed" if hits >= cfg.confirmation_hits else "tentative")
        scale = min(cfg.maximum_covariance_scale, covariance_scale * inflation)
        estimate = replace(estimate, state=state, confidence=float(observation.confidence / np.sqrt(scale)))
        try:
            estimate.validate(cfg.frame_id)
        except (ValueError, np.linalg.LinAlgError) as error:
            return self._reject(observation, error, d2)
        self.tracks[observation.target_id] = _Track(estimate, hits, components)
        self.recovery.pop(observation.target_id, None)
        self.counts[decision] += 1
        self.last_diagnostic = {"target_id": observation.target_id, "source": observation.source,
                                "position_reference": observation.position_reference, "decision": decision,
                                "reason": "", "d2": d2, "covariance_scale": scale, "ci_weight": weight,
                                "track_state": state}
        return estimate
