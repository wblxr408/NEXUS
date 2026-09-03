from dataclasses import dataclass

import numpy as np


VALID_SOURCE_MODES = frozenset({1, 2, 3, 4})


@dataclass(frozen=True)
class Observation:
    target_id: str
    frame_id: str
    stamp_ns: int
    position: np.ndarray
    covariance: np.ndarray
    confidence: float
    source_mode: int
    orientation_xyzw: np.ndarray
    receive_timestamp_ns: int = 0
    validity: int = 1
    invalid_reason: str = ""
    unit: str = "m"


def validate_observation(observation, expected_frame):
    if observation.validity != 1 or observation.invalid_reason:
        return False
    if observation.unit != "m":
        return False
    if not observation.target_id:
        return False
    if not expected_frame or observation.frame_id != expected_frame:
        return False
    if observation.stamp_ns <= 0:
        return False
    if observation.receive_timestamp_ns < observation.stamp_ns:
        return False
    if observation.source_mode not in VALID_SOURCE_MODES:
        return False
    if not np.isfinite(observation.confidence) or not 0.0 <= float(observation.confidence) <= 1.0:
        return False
    if observation.position.shape != (3,) or not np.all(np.isfinite(observation.position)):
        return False
    if np.asarray(observation.covariance).size != 9:
        return False
    covariance = np.asarray(observation.covariance, dtype=float).reshape(3, 3)
    if (not np.all(np.isfinite(covariance)) or not np.allclose(covariance, covariance.T, atol=1e-9)
            or np.any(np.linalg.eigvalsh(covariance) <= 0.0)):
        return False
    if observation.orientation_xyzw.shape != (4,) or not np.all(np.isfinite(observation.orientation_xyzw)):
        return False
    orientation_norm = float(np.linalg.norm(observation.orientation_xyzw))
    if orientation_norm <= 1e-9:
        return False
    return True


def is_stale(stamp_ns, now_ns, max_age_ns):
    if stamp_ns <= 0 or now_ns < stamp_ns:
        return True
    return now_ns - stamp_ns > max_age_ns


def _effective_covariance(observation):
    covariance = np.asarray(observation.covariance, dtype=float).reshape(3, 3)
    if np.all(np.isfinite(covariance)) and np.allclose(covariance, covariance.T, atol=1e-9):
        eigenvalues = np.linalg.eigvalsh(covariance)
        if np.all(eigenvalues > 0):
            return covariance
    confidence = max(float(observation.confidence), 1e-6)
    return np.eye(3) / confidence


def fuse_observations(observations, expected_frame, now_ns, max_age_ns,
                      max_pair_delta_ns=None):
    valid = [
        item for item in observations
        if validate_observation(item, expected_frame)
        and not is_stale(item.stamp_ns, now_ns, max_age_ns)
    ]
    if not valid:
        return None
    target_id = valid[0].target_id
    valid = [item for item in valid if item.target_id == target_id]
    degraded_reason = ""
    if (len(valid) > 1 and max_pair_delta_ns is not None
            and max(item.stamp_ns for item in valid) - min(item.stamp_ns for item in valid)
            > max_pair_delta_ns):
        valid = [max(valid, key=lambda item: item.stamp_ns)]
        degraded_reason = "unsynchronized_sources"
    if len(valid) == 1:
        item = valid[0]
        return {
            "target_id": item.target_id,
            "frame_id": item.frame_id,
            "stamp_ns": item.stamp_ns,
            "position": item.position.copy(),
            "covariance": _effective_covariance(item),
            "confidence": float(item.confidence),
            "source_mode": item.source_mode,
            "orientation_xyzw": item.orientation_xyzw / np.linalg.norm(item.orientation_xyzw),
            "degraded_reason": degraded_reason,
        }
    information = np.zeros((3, 3))
    information_vector = np.zeros(3)
    for item in valid:
        inverse = np.linalg.pinv(_effective_covariance(item))
        information += inverse
        information_vector += inverse @ item.position
    covariance = np.linalg.pinv(information)
    position = covariance @ information_vector
    best = max(valid, key=lambda item: item.confidence)
    return {
        "target_id": target_id,
        "frame_id": expected_frame,
        "stamp_ns": max(item.stamp_ns for item in valid),
        "position": position,
        "covariance": covariance,
        "confidence": float(max(item.confidence for item in valid)),
        "source_mode": 4,
        "orientation_xyzw": best.orientation_xyzw / np.linalg.norm(best.orientation_xyzw),
        "degraded_reason": "",
    }
