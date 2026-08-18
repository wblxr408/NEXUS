import numpy as np


def is_observation_stale(sample_timestamp_ns, now_timestamp_ns, max_age_ns):
    """Return true when a sample is missing, from the future, or too old."""
    if sample_timestamp_ns <= 0 or now_timestamp_ns < sample_timestamp_ns:
        return True
    return now_timestamp_ns - sample_timestamp_ns > max_age_ns


def source_mode_counts(source_modes):
    """Count source labels without assigning an accuracy meaning to them."""
    counts = {}
    for mode in source_modes:
        key = str(mode)
        counts[key] = counts.get(key, 0) + 1
    return counts


def nearest_time_pairs(left_timestamps_ns, right_timestamps_ns, max_delta_ns):
    left = np.asarray(left_timestamps_ns, dtype=np.int64)
    right = np.asarray(right_timestamps_ns, dtype=np.int64)
    if left.ndim != 1 or right.ndim != 1 or max_delta_ns < 0:
        raise ValueError("timestamps must be one-dimensional and max_delta_ns non-negative")
    pairs = []
    for index, stamp in enumerate(left):
        candidate = int(np.argmin(np.abs(right - stamp))) if right.size else None
        if candidate is not None and abs(int(right[candidate]) - int(stamp)) <= max_delta_ns:
            pairs.append((index, candidate, int(right[candidate]) - int(stamp)))
    return pairs


def weighted_position(positions, covariances, confidences=None):
    points = np.asarray(positions, dtype=float)
    covariances = np.asarray(covariances, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or covariances.shape != (points.shape[0], 3, 3):
        raise ValueError("positions must be N x 3 and covariances N x 3 x 3")
    if points.shape[0] == 0:
        raise ValueError("at least one position is required")
    information = np.zeros((3, 3))
    vector = np.zeros(3)
    for point, covariance in zip(points, covariances):
        if not np.all(np.isfinite(point)) or not np.all(np.isfinite(covariance)):
            raise ValueError("positions and covariances must be finite")
        inverse = np.linalg.pinv(covariance)
        information += inverse
        vector += inverse @ point
    result_covariance = np.linalg.pinv(information)
    return result_covariance @ vector, result_covariance
