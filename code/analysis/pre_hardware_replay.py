"""Functional replay for declared pre-hardware test samples.

This module validates message semantics and deterministic fusion/degradation
decisions. It deliberately does not model a sensor or report measured accuracy.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import numpy as np

# The replay intentionally calls the project coordinate module instead of
# duplicating its transform math in analysis code.
_COORDINATE_SOURCE = Path(__file__).parents[1] / "ros2_ws" / "src" / "nexus_coord_transform" / "src"
if str(_COORDINATE_SOURCE) not in sys.path:
    sys.path.insert(0, str(_COORDINATE_SOURCE))

from nexus_coord_transform.geometry import direct_target_measurement
from evaluation.metrics import latency_metrics, position_metrics
from fusion.time_alignment import is_observation_stale, weighted_position


@dataclass(frozen=True)
class ReplayDecision:
    case_id: str
    status: str
    source_mode: str | None
    position_m: list[float] | None
    rejected: list[str]


def _transform_to_map(observation: dict[str, Any], expected_frame: str,
                      transforms: dict[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
    """Use nexus_coord_transform for a declared sensor-frame observation."""
    converted = dict(observation)
    if observation["frame_id"] == expected_frame:
        return converted, None
    transform = transforms.get(observation["frame_id"])
    if transform is None:
        return None, "invalid_frame"
    try:
        converted["position_m"] = direct_target_measurement(
            observation["position_m"], transform["rotation"], transform["translation_m"],
        ).tolist()
    except (KeyError, TypeError, ValueError):
        return None, "invalid_transform"
    converted["frame_id"] = expected_frame
    return converted, None


def _valid_observation(observation: dict[str, Any], expected_frame: str,
                       now_timestamp_ns: int, max_age_ns: int) -> str | None:
    if observation.get("target_id") != "test_target":
        return "missing_or_wrong_target"
    if observation.get("unit") != "m":
        return "invalid_unit"
    if is_observation_stale(int(observation.get("sample_timestamp_ns", 0)),
                            now_timestamp_ns, max_age_ns):
        return "stale"
    position = np.asarray(observation.get("position_m"), dtype=float)
    covariance = np.asarray(observation.get("covariance_m2"), dtype=float)
    confidence = observation.get("confidence")
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        return "invalid_position"
    if covariance.shape != (3, 3) or not np.all(np.isfinite(covariance)):
        return "invalid_covariance"
    if not isinstance(confidence, (int, float)) or not 0.0 <= confidence <= 1.0:
        return "invalid_confidence"
    return None


def replay_case(case: dict[str, Any]) -> ReplayDecision:
    """Replay one declared test-only case with fixed fusion and degradation rules."""
    if case.get("data_type", "test_sample") != "test_sample":
        raise ValueError("pre-hardware replay only accepts data_type=test_sample")
    expected_frame = case["expected_frame"]
    now_timestamp_ns = int(case["now_timestamp_ns"])
    max_age_ns = int(case["max_age_ns"])
    max_pair_delta_ns = int(case["max_pair_delta_ns"])
    transforms = case.get("sensor_to_map_transforms", {})
    valid: list[dict[str, Any]] = []
    rejected: list[str] = []
    for observation in case["observations"]:
        converted, transform_error = _transform_to_map(observation, expected_frame, transforms)
        error = transform_error or _valid_observation(
            converted, expected_frame, now_timestamp_ns, max_age_ns
        )
        if error:
            rejected.append(f"{observation['source_mode']}:{error}")
        else:
            valid.append(converted)
    if not valid:
        return ReplayDecision(case["case_id"], "no_valid_observation", None, None, rejected)
    if len(valid) == 1:
        item = valid[0]
        return ReplayDecision(case["case_id"], "single_source", item["source_mode"],
                              list(item["position_m"]), rejected)
    stamps = [int(item["sample_timestamp_ns"]) for item in valid]
    if max(stamps) - min(stamps) > max_pair_delta_ns:
        rejected.extend(f"{item['source_mode']}:time_mismatch" for item in valid)
        return ReplayDecision(case["case_id"], "no_synchronized_pair", None, None, rejected)
    position, _ = weighted_position(
        [item["position_m"] for item in valid],
        [item["covariance_m2"] for item in valid],
    )
    return ReplayDecision(case["case_id"], "fused", "SOURCE_FUSED",
                          position.tolist(), rejected)


def replay_document(document: dict[str, Any]) -> dict[str, Any]:
    """Return a functional test report and assert each declared expected status."""
    if document.get("data_type") != "test_sample":
        raise ValueError("replay document must be marked data_type=test_sample")
    decisions = [replay_case(case) for case in document["cases"]]
    cases_by_id = {case["case_id"]: case for case in document["cases"]}
    expected_statuses = {
        case["case_id"]: case["expected_status"] for case in document["cases"]
    }
    failed = [
        decision.case_id for decision in decisions
        if decision.status != expected_statuses[decision.case_id]
    ]
    measured = [(decision.position_m, cases_by_id[decision.case_id]["truth_position_m"], decision)
                for decision in decisions if decision.position_m is not None]
    metrics = {
        "data_type": "test_sample",
        "is_measured_result": False,
        "position": position_metrics([item[0] for item in measured], [item[1] for item in measured]),
        "latency": latency_metrics(
            [cases_by_id[item[2].case_id]["sample_timestamp_for_metrics_ns"] for item in measured],
            [cases_by_id[item[2].case_id]["receive_timestamp_for_metrics_ns"] for item in measured],
        ),
        "interpretation": "synthetic functional output only; not a localization result",
    }
    return {
        "data_type": "test_sample",
        "purpose": "functional interface, fusion, and degradation checks only",
        "is_measured_result": False,
        "accuracy_claim": "none",
        "cases": [decision.__dict__ for decision in decisions],
        "metrics": metrics,
        "passed": not failed,
        "failed_case_ids": failed,
    }
