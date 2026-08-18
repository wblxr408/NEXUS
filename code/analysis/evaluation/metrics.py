import numpy as np


def _errors(estimates, references):
    estimated = np.asarray(estimates, dtype=float)
    reference = np.asarray(references, dtype=float)
    if estimated.shape != reference.shape or estimated.ndim != 2 or estimated.shape[1] != 3:
        raise ValueError("estimates and references must have matching N x 3 shapes")
    if estimated.shape[0] == 0 or not np.all(np.isfinite(estimated)) or not np.all(np.isfinite(reference)):
        raise ValueError("at least one finite estimate/reference pair is required")
    return estimated - reference


def position_metrics(estimates, references, outlier_threshold_m=None):
    errors = _errors(estimates, references)
    distances = np.linalg.norm(errors, axis=1)
    outlier_count = 0
    if outlier_threshold_m is not None:
        threshold = float(outlier_threshold_m)
        if not np.isfinite(threshold) or threshold < 0:
            raise ValueError("outlier threshold must be a finite non-negative metre value")
        outlier_count = int(np.count_nonzero(distances > threshold))
    return {
        "samples": int(distances.size),
        "unit": "m",
        "rmse_m": float(np.sqrt(np.mean(distances ** 2))),
        "cep50_m": float(np.percentile(distances, 50)),
        "cep95_m": float(np.percentile(distances, 95)),
        "max_error_m": float(np.max(distances)),
        "outlier_count": outlier_count,
    }


def update_frequency_hz(timestamps_ns):
    stamps = np.asarray(timestamps_ns, dtype=np.int64)
    if stamps.size < 2:
        return None
    deltas = np.diff(stamps) / 1e9
    if np.any(deltas <= 0):
        raise ValueError("timestamps must be strictly increasing")
    return float(1.0 / np.mean(deltas))


def latency_metrics(sample_timestamps_ns, receive_timestamps_ns):
    samples = np.asarray(sample_timestamps_ns, dtype=np.int64)
    receives = np.asarray(receive_timestamps_ns, dtype=np.int64)
    if samples.shape != receives.shape or samples.size == 0:
        raise ValueError("sample and receive timestamps must have equal non-empty shapes")
    latency_ms = (receives - samples) / 1e6
    if np.any(latency_ms < 0):
        raise ValueError("receive time cannot precede sample time")
    return {
        "samples": int(latency_ms.size),
        "unit": "ms",
        "mean_latency_ms": float(np.mean(latency_ms)),
        "p95_latency_ms": float(np.percentile(latency_ms, 95)),
        "max_latency_ms": float(np.max(latency_ms)),
    }
