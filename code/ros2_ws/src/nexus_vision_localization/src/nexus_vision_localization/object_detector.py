"""Local YOLO detection-only ONNX adapter, independent of ROS and target IDs."""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection2D:
    class_id: int
    label: str
    confidence: float
    bbox_xywh_px: tuple

    def as_dict(self):
        return {"class_id": self.class_id, "label": self.label,
                "confidence": self.confidence, "bbox_xywh_px": list(self.bbox_xywh_px)}


def letterbox(image, input_size_wh, padding_value=114):
    image = np.asarray(image)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 1:
        raise ValueError("YOLO requires an 8-bit BGR image")
    width, height = input_size_wh
    ratio = min(width / image.shape[1], height / image.shape[0])
    resized_width, resized_height = max(1, round(image.shape[1] * ratio)), max(1, round(image.shape[0] * ratio))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    left, top = (width - resized_width) // 2, (height - resized_height) // 2
    canvas = np.full((height, width, 3), padding_value, dtype=np.uint8)
    canvas[top:top + resized_height, left:left + resized_width] = resized
    blob = cv2.dnn.blobFromImage(canvas, scalefactor=1 / 255., swapRB=True, crop=False)
    return blob, (resized_width / image.shape[1], resized_height / image.shape[0], left, top)


def decode_yolo(output, image_size_wh, transform, class_names, *, architecture, layout,
                confidence_threshold=.35, iou_threshold=.5, maximum_detections=100):
    if architecture not in {"yolo_v8_detection", "yolo_v5_detection"} or layout not in {"channels_first", "anchors_first"}:
        raise ValueError("unsupported YOLO output format")
    if (not 0 < confidence_threshold <= 1 or not 0 < iou_threshold <= 1
            or not isinstance(maximum_detections, int) or maximum_detections < 1):
        raise ValueError("invalid detection confidence/NMS settings")
    raw = np.asarray(output, dtype=float)
    if raw.ndim != 3 or raw.shape[0] != 1 or not np.all(np.isfinite(raw)):
        raise ValueError("YOLO output must be finite with batch size one")
    rows = raw[0].T if layout == "channels_first" else raw[0]
    offset = 4 if architecture == "yolo_v8_detection" else 5
    if rows.shape[1] != offset + len(class_names) or not class_names:
        raise ValueError("YOLO class names do not match model output channels")
    probabilities = rows[:, 4:]
    if np.any(probabilities < 0) or np.any(probabilities > 1):
        raise ValueError("YOLO scores must be activated probabilities")
    if len(rows) == 0:
        return []
    classes = np.argmax(rows[:, offset:], axis=1)
    scores = rows[np.arange(len(rows)), offset + classes]
    if offset == 5:
        scores *= rows[:, 4]
    width, height = image_size_wh
    sx, sy, left, top = transform
    boxes = []
    candidates = []
    for index in np.flatnonzero(scores >= confidence_threshold):
        center_x, center_y, box_width, box_height = rows[index, :4]
        if min(box_width, box_height) <= 0:
            continue
        x0 = float(np.clip((center_x - box_width / 2 - left) / sx, 0., width))
        y0 = float(np.clip((center_y - box_height / 2 - top) / sy, 0., height))
        x1 = float(np.clip((center_x + box_width / 2 - left) / sx, 0., width))
        y1 = float(np.clip((center_y + box_height / 2 - top) / sy, 0., height))
        if x1 <= x0 or y1 <= y0:
            continue
        boxes.append([x0, y0, x1 - x0, y1 - y0])
        candidates.append(index)
    retained = []
    for class_id in sorted(set(classes[candidates])):
        group = [i for i, original in enumerate(candidates) if classes[original] == class_id]
        selected = cv2.dnn.NMSBoxes([boxes[i] for i in group], [float(scores[candidates[i]]) for i in group],
                                    confidence_threshold, iou_threshold)
        retained.extend(group[int(i)] for i in np.asarray(selected).reshape(-1))
    retained.sort(key=lambda i: -scores[candidates[i]])
    return [Detection2D(int(classes[candidates[i]]), class_names[int(classes[candidates[i]])],
                        float(scores[candidates[i]]), tuple(boxes[i])) for i in retained[:maximum_detections]]


class ObjectDetectorOnnx:
    def __init__(self, manifest_path):
        path = Path(manifest_path)
        with path.open(encoding="utf-8") as stream:
            self.manifest = json.load(stream)
        manifest = self.manifest
        required = {"schema_version", "architecture", "model_file", "sha256", "input_name", "output_name",
                    "input_size_wh", "padding_value", "layout", "class_names", "source", "license"}
        if not isinstance(manifest, dict) or not required.issubset(manifest):
            raise ValueError("YOLO model manifest is incomplete")
        if (manifest["schema_version"] != 1 or manifest["architecture"] not in {"yolo_v8_detection", "yolo_v5_detection"}
                or manifest["layout"] not in {"channels_first", "anchors_first"}):
            raise ValueError("unsupported YOLO model format")
        size = manifest["input_size_wh"]
        if (not isinstance(size, list) or len(size) != 2
                or any(not isinstance(v, int) or isinstance(v, bool) or v < 8 for v in size)):
            raise ValueError("YOLO input dimensions must be positive integers >=8")
        if not isinstance(manifest["padding_value"], int) or not 0 <= manifest["padding_value"] <= 255:
            raise ValueError("YOLO padding must be a byte")
        if not isinstance(manifest["class_names"], list) or not manifest["class_names"]:
            raise ValueError("YOLO manifest requires ordered class names")
        for value in manifest["class_names"] + [manifest[key] for key in ("model_file", "sha256", "input_name", "output_name", "source", "license")]:
            if not isinstance(value, str) or not value.strip():
                raise ValueError("YOLO manifest names/provenance must be nonempty strings")
        model = path.parent / manifest["model_file"]
        if hashlib.sha256(model.read_bytes()).hexdigest() != manifest["sha256"].lower():
            raise ValueError("YOLO model SHA256 mismatch")
        self.network = cv2.dnn.readNetFromONNX(str(model))
        self.network.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.network.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def infer(self, image, **settings):
        blob, transform = letterbox(image, self.manifest["input_size_wh"], self.manifest["padding_value"])
        self.network.setInput(blob, self.manifest["input_name"])
        output = self.network.forward(self.manifest["output_name"])
        return decode_yolo(output, (image.shape[1], image.shape[0]), transform, self.manifest["class_names"],
                           architecture=self.manifest["architecture"], layout=self.manifest["layout"], **settings)
