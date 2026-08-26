import sys
from pathlib import Path

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "evaluation"))

from observation_quality import observation_quality


def test_quality_report_counts_failures_latency_and_gaps():
    records = [
        {"sample_timestamp_ns": "1000000000", "receive_timestamp_ns": "1010000000",
         "target_id": "t", "source_mode": "VISION", "validity": "VALID",
         "confidence": "0.8", "reprojection_error_px": "1.0"},
        {"sample_timestamp_ns": "1100000000", "receive_timestamp_ns": "1115000000",
         "target_id": "t", "source_mode": "VISION", "validity": "INVALID",
         "invalid_reason": "target_not_detected", "confidence": "nan"},
        {"sample_timestamp_ns": "1300000000", "receive_timestamp_ns": "1320000000",
         "target_id": "t", "source_mode": "UWB", "validity": "VALID",
         "confidence": "0.6", "reprojection_error_px": "nan"},
    ]
    report = observation_quality(records, expected_rate_hz=10.0)
    assert report["availability"] == 2 / 3
    assert report["invalid_reason_counts"] == {"target_not_detected": 1}
    assert report["source_mode_counts"] == {"VISION": 2, "UWB": 1}
    assert report["latency_ms"]["max"] == 20.0
    assert report["estimated_dropped_samples"] == 2
