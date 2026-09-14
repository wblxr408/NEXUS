"""Produce checksum-bound ONNX Runtime INT8 edge assets from a fixed ONNX model."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def letterbox(image, size, padding):
    width, height = size
    ratio = min(width / image.shape[1], height / image.shape[0])
    resized_width, resized_height = max(1, round(image.shape[1] * ratio)), max(1, round(image.shape[0] * ratio))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((height, width, 3), padding, np.uint8)
    left, top = (width - resized_width) // 2, (height - resized_height) // 2
    canvas[top:top + resized_height, left:left + resized_width] = resized
    return np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.)


def verify_detector_scores(source_model, quantized_model, input_name, images, input_size_wh, padding):
    """Reject a graph that erased meaningful YOLO confidence outputs."""
    import onnxruntime as ort
    reference = ort.InferenceSession(str(source_model), providers=["CPUExecutionProvider"])
    candidate = ort.InferenceSession(str(quantized_model), providers=["CPUExecutionProvider"])
    reference_peak = candidate_peak = 0.0
    for path in images[:min(16, len(images))]:
        image = cv2.imread(str(path))
        if image is None:
            continue
        blob = letterbox(image, input_size_wh, padding)
        output = reference.run(None, {input_name: blob})[0]
        quantized = candidate.run(None, {input_name: blob})[0]
        if output.ndim == 3 and output.shape[1] > 4 and quantized.shape == output.shape:
            reference_peak = max(reference_peak, float(np.max(output[:, 4:, :])))
            candidate_peak = max(candidate_peak, float(np.max(quantized[:, 4:, :])))
    if reference_peak >= 0.05 and candidate_peak < max(1e-4, reference_peak * 0.01):
        raise RuntimeError("INT8 quantization erased detector confidence scores; asset was rejected")


class CalibrationImages:
    def __init__(self, images, input_name, input_size_wh, padding):
        self.images, self.input_name, self.input_size_wh, self.padding = iter(images), input_name, input_size_wh, padding

    def get_next(self):
        try:
            path = next(self.images)
        except StopIteration:
            return None
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"calibration image unreadable: {path}")
        return {self.input_name: letterbox(image, self.input_size_wh, self.padding)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True, help="FP32 ONNX model")
    parser.add_argument("--manifest", type=Path, required=True, help="matching FP32 manifest")
    parser.add_argument("--output", type=Path, required=True, help="new, empty edge asset directory")
    parser.add_argument("--calibration-images", type=Path, required=True,
                        help="directory of representative labelled/unlabelled BGR images")
    parser.add_argument("--threads", type=int, default=1)
    args = parser.parse_args()
    if (not args.model.is_file() or not args.manifest.is_file() or args.output.exists() or args.threads < 1
            or not args.calibration_images.is_dir()):
        raise ValueError("model/manifest/calibration images must exist, output must be new, and threads must be positive")
    source = json.loads(args.manifest.read_text(encoding="utf-8"))
    source_format = source.get("model_format", "onnx" if args.model.suffix.lower() == ".onnx" else None)
    source_precision = source.get("precision", "float32")
    if (source_format != "onnx" or source_precision != "float32"
            or source.get("sha256", "").lower() != digest(args.model)):
        raise ValueError("manifest must checksum the supplied FP32 ONNX model")
    try:
        from onnxruntime.quantization import CalibrationMethod, QuantFormat, QuantType, quantize_static
    except ImportError as error:
        raise RuntimeError("onnxruntime-tools/quantization support is required") from error
    args.output.mkdir(parents=True)
    target = args.output / "model_int8.onnx"
    images = sorted(path for suffix in ("*.jpg", "*.jpeg", "*.png", "*.bmp") for path in args.calibration_images.rglob(suffix))
    if not images:
        raise ValueError("calibration-images contains no supported images")
    if "input_size_wh" not in source or "input_name" not in source:
        raise ValueError("manifest lacks model input geometry for static quantization")
    # This detector uses opset 12: per-channel QDQ cannot encode its axis
    # attribute there. Per-tensor QDQ retains a float output for the existing
    # postprocessor and avoids QOperator's all-zero score output on this model.
    quantize_static(str(args.model), str(target),
                    CalibrationImages(images[:min(256, len(images))], source["input_name"], source["input_size_wh"],
                                      int(source.get("padding_value", 114))),
                    quant_format=QuantFormat.QDQ, activation_type=QuantType.QInt8, weight_type=QuantType.QInt8,
                    per_channel=False, calibrate_method=CalibrationMethod.MinMax,
                    op_types_to_quantize=["Conv", "Gemm", "MatMul"])
    try:
        verify_detector_scores(args.model, target, source["input_name"], images, source["input_size_wh"],
                               int(source.get("padding_value", 114)))
    except Exception:
        target.unlink(missing_ok=True)
        raise
    document = {**source, "model_format": "onnx", "model_file": target.name, "sha256": digest(target), "precision": "int8",
                "inference_device": "onnxruntime_cpu", "intra_op_threads": args.threads,
                "quantization": {"method": "onnxruntime_static_qdq_weighted_ops", "weight_type": "QInt8", "activation_type": "QInt8",
                                 "calibration_images": len(images[:min(256, len(images))]), "per_channel": False,
                                 "quantized_ops": ["Conv", "Gemm", "MatMul"],
                                 "source_model_sha256": digest(args.model)}}
    (args.output / "manifest.json").write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"model": str(target), "manifest": str(args.output / "manifest.json"),
                      "sha256": document["sha256"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
