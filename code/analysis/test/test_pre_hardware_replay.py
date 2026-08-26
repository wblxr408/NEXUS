import json
import sys
from pathlib import Path

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))

from pre_hardware_replay import replay_document


def test_declared_pre_hardware_cases_pass():
    sample_path = ROOT / "test_samples" / "pre_hardware_replay_cases.json"
    with sample_path.open(encoding="utf-8") as stream:
        report = replay_document(json.load(stream))
    assert report["data_type"] == "test_sample"
    assert report["is_measured_result"] is False
    assert report["passed"] is True
    assert len(report["cases"]) == 8
    assert report["metrics"]["data_type"] == "test_sample"
    assert report["metrics"]["position"]["samples"] == 4
    assert report["metrics"]["latency"]["samples"] == 4
    fused = next(case for case in report["cases"] if case["case_id"] == "dual_source_fused")
    assert fused["position_m"] == [1.08, 2.0, 0.5]
    transformed = next(case for case in report["cases"] if case["case_id"] == "camera_frame_single_source_transform")
    assert transformed["position_m"] == [1.2, 2.0, 0.5]
