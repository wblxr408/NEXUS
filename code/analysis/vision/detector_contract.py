"""Detector-to-GDR-Net crop contract; this is not a detector implementation.

An external detector supplies class and pixel box.  Keeping this boundary
separate lets the upstream GDR-Net architecture remain unchanged and prevents
test-time crops from silently falling back to BOP ground-truth boxes.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def load_detector_predictions(path: str | Path) -> dict[str, list[dict[str, Any]]]:
    """Load canonical detector JSON keyed by BOP frame id.

    Required record fields are ``class_id``, ``bbox_xywh_px`` and
    ``confidence``.  ``obj_id`` is accepted as an alias for detector adapters
    that already use BOP object IDs.
    """
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    frames = document.get("frames", document)
    if not isinstance(frames, dict):
        raise ValueError("detector predictions must be a frame-keyed object")
    normalized: dict[str, list[dict[str, Any]]] = {}
    for frame_id, detections in frames.items():
        if not isinstance(detections, list):
            raise ValueError(f"detections for frame {frame_id} must be a list")
        records = []
        for detection in detections:
            class_id = detection.get("class_id", detection.get("obj_id"))
            bbox = detection.get("bbox_xywh_px", detection.get("bbox_xywh"))
            if not isinstance(class_id, int) or not isinstance(bbox, list) or len(bbox) != 4:
                raise ValueError(f"frame {frame_id}: each detection needs integer class_id and bbox_xywh_px")
            confidence = float(detection.get("confidence", 1.0))
            if not 0.0 <= confidence <= 1.0:
                raise ValueError(f"frame {frame_id}: confidence must be in [0, 1]")
            x, y, width, height = (float(value) for value in bbox)
            if width <= 0.0 or height <= 0.0:
                raise ValueError(f"frame {frame_id}: bbox width/height must be positive")
            records.append({"class_id": class_id, "bbox_xywh_px": [x, y, width, height], "confidence": confidence})
        normalized[str(int(frame_id))] = records
    return normalized
