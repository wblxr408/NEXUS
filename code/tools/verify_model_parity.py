"""Verify OpenCV CPU outputs against separately generated PyTorch references."""

import argparse
import json
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

from nexus_vision_localization.object_detector import ObjectDetectorOnnx, decode_yolo, letterbox
from nexus_vision_localization.superpoint_frontend import SuperPointOnnx


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    detector = ObjectDetectorOnnx(args.root / "detector/detector_slim_manifest.json")
    superpoint = SuperPointOnnx(args.root / "superpoint/superpoint_manifest.json")
    cv2.setNumThreads(4)
    report = dict(opencv=cv2.__version__, backend="OpenCV CPU", threads=4, samples=[])
    for path in sorted((args.root / "parity").glob("*.npz")):
        data = np.load(path)
        detector.network.setInput(data["detector_blob"], "images")
        started = perf_counter()
        actual = detector.network.forward("output0")
        elapsed = (perf_counter() - started) * 1000
        np.testing.assert_allclose(actual, data["detector_output"], rtol=1e-4, atol=.002)
        image = data["image"]
        _, transform = letterbox(image, (640, 640))
        kwargs = dict(architecture="yolo_v11_detection", layout="channels_first")
        expected = decode_yolo(data["detector_output"], (image.shape[1], image.shape[0]), transform,
                               detector.manifest["class_names"], **kwargs)
        detections = detector.infer(image)
        assert [x.class_id for x in detections] == [x.class_id for x in expected]
        np.testing.assert_allclose([x.bbox_xywh_px for x in detections],
                                   [x.bbox_xywh_px for x in expected], atol=.01)
        item = dict(file=path.name, detector_max_abs=float(np.max(abs(actual - data["detector_output"]))),
                    detector_forward_ms=elapsed, detections=[x.as_dict() for x in detections])
        if "superpoint_blob" in data:
            superpoint.network.setInput(data["superpoint_blob"], "image")
            started = perf_counter()
            logits, descriptors = superpoint.network.forward(["logits", "descriptors"])
            item["superpoint_forward_ms"] = (perf_counter() - started) * 1000
            np.testing.assert_allclose(logits, data["logits"], atol=1e-4, rtol=1e-4)
            np.testing.assert_allclose(descriptors, data["descriptors"], atol=1e-4, rtol=1e-4)
            item["superpoint_logits_max_abs"] = float(np.max(abs(logits - data["logits"])))
            features = superpoint.infer(image).features()
            assert len(features.points_px) > 0
            item["superpoint_keypoints"] = len(features.points_px)
        report["samples"].append(item)
    assert len(report["samples"]) >= 8
    (args.root / "model_parity.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
