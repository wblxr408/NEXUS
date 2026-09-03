"""Load and store bundle problems as JSON.

The schema mirrors the factor dataclasses field for field, so a run record can
be replayed without a python driver.  ``simulation/markerless_bundle_example_v01.json``
is the reference document for the layout.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .bundle_solver import BundleProblem, PoseState
from .factors import (BearingFactor, CatalogPriorFactor, ContourFactor, ControlPointFactor,
                      Factor, PlatformPoseFactor, RelativePoseFactor, SegmentUwbFactor, SupportPlaneFactor)


_FACTOR_TYPES = {
    "F1": BearingFactor, "F2": ContourFactor, "F3": SupportPlaneFactor, "F4": ControlPointFactor,
    "F5": RelativePoseFactor, "F6": SegmentUwbFactor, "F7": PlatformPoseFactor, "F8": CatalogPriorFactor,
}
_ARRAY_FIELDS = {
    "point_target_m", "normalized_xy", "observed_pixel_xy", "normal_xy", "camera_matrix", "point_map_m",
    "rotation_i_j", "translation_i_j_m", "slam_position", "uwb_position_m", "rotation_map_camera",
    "translation_map_camera_m", "translation_map_target_m",
}


def _pose(entry: dict[str, Any]) -> PoseState:
    return PoseState(np.asarray(entry["rotation"], dtype=float), np.asarray(entry["translation_m"], dtype=float))


def factor_from_dict(entry: dict[str, Any]) -> Factor:
    kind = str(entry["kind"])
    if kind not in _FACTOR_TYPES:
        raise ValueError(f"unknown factor kind: {kind}")
    fields = {key: value for key, value in entry.items() if key != "kind"}
    for name in list(fields):
        if name in _ARRAY_FIELDS:
            fields[name] = np.asarray(fields[name], dtype=float)
    if "symmetries" in fields:
        fields["symmetries"] = tuple(np.asarray(rotation, dtype=float) for rotation in fields["symmetries"])
    return _FACTOR_TYPES[kind](**fields)


def problem_from_dict(document: dict[str, Any]) -> BundleProblem:
    return BundleProblem(
        target_poses={int(key): _pose(value) for key, value in document["initial_target_poses"].items()},
        camera_poses={int(key): _pose(value) for key, value in document["camera_poses"].items()},
        factors=[factor_from_dict(entry) for entry in document["factors"]],
        segment_scale=float(document.get("segment_scale", 1.0)),
        segment_bias_m=np.asarray(document.get("segment_bias_m", [0.0, 0.0, 0.0]), dtype=float),
    )


def load_problem(path: str | Path) -> tuple[BundleProblem, dict[str, Any]]:
    """Return the problem plus the untouched document for frame id and quality."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    return problem_from_dict(document), document
