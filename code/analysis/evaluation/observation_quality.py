from collections import Counter

import numpy as np


def _is_valid(value):
    return str(value).strip().upper() in {"1", "TRUE", "VALID", "OK"}


def observation_quality(records, expected_rate_hz=None):
    if not records:
        raise ValueError("at least one observation record is required")
    valid = [row for row in records if _is_valid(row.get("validity", "VALID"))]
    invalid = [row for row in records if row not in valid]
    result = {
        "total_observations": len(records),
        "valid_observations": len(valid),
        "invalid_observations": len(invalid),
        "availability": len(valid) / len(records),
        "source_mode_counts": dict(Counter(str(row.get("source_mode", "UNKNOWN")) for row in records)),
        "invalid_reason_counts": dict(Counter(
            str(row.get("invalid_reason") or "unspecified") for row in invalid)),
        "target_counts": dict(Counter(str(row.get("target_id", "")) for row in records)),
    }
    confidence = np.asarray([
        float(row["confidence"]) for row in valid
        if str(row.get("confidence", "")).strip() not in {"", "nan", "NaN"}
    ], dtype=float)
    if confidence.size:
        finite = confidence[np.isfinite(confidence)]
        if finite.size:
            result["confidence"] = {
                "samples": int(finite.size), "mean": float(np.mean(finite)),
                "p05": float(np.percentile(finite, 5)),
                "p95": float(np.percentile(finite, 95)),
            }
    latency_ms = np.asarray([
        (int(row["receive_timestamp_ns"]) - int(row["sample_timestamp_ns"])) / 1e6
        for row in valid
    ], dtype=float)
    if latency_ms.size:
        if np.any(latency_ms < 0):
            raise ValueError("receive timestamp cannot precede sample timestamp")
        result["latency_ms"] = {
            "samples": int(latency_ms.size), "mean": float(np.mean(latency_ms)),
            "p95": float(np.percentile(latency_ms, 95)),
            "max": float(np.max(latency_ms)),
        }
    reprojection = np.asarray([
        float(row["reprojection_error_px"]) for row in valid
        if str(row.get("reprojection_error_px", "")).strip() not in {"", "nan", "NaN"}
    ], dtype=float)
    reprojection = reprojection[np.isfinite(reprojection)]
    if reprojection.size:
        result["reprojection_error_px"] = {
            "samples": int(reprojection.size), "mean": float(np.mean(reprojection)),
            "p95": float(np.percentile(reprojection, 95)),
            "max": float(np.max(reprojection)),
        }
    stamps = np.sort(np.asarray(
        [int(row["sample_timestamp_ns"]) for row in valid], dtype=np.int64))
    if stamps.size >= 2:
        deltas = np.diff(stamps) / 1e9
        positive = deltas[deltas > 0]
        if positive.size:
            result["update_frequency_hz"] = float(1.0 / np.mean(positive))
            result["max_gap_ms"] = float(np.max(positive) * 1000.0)
            if expected_rate_hz is not None:
                rate = float(expected_rate_hz)
                if rate <= 0:
                    raise ValueError("expected_rate_hz must be positive")
                period = 1.0 / rate
                result["estimated_dropped_samples"] = int(sum(
                    max(round(delta / period) - 1, 0) for delta in positive))
    return result

