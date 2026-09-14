"""Export a trained Ultralytics detector without altering the source checkpoint."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--image-size", type=int, default=640)
    args = parser.parse_args()
    if args.image_size < 32 or args.image_size % 32:
        raise ValueError("image-size must be a multiple of 32 and >= 32")
    args.output.mkdir(parents=True, exist_ok=False)
    copied = args.output / "detector.pt"
    shutil.copy2(args.checkpoint, copied)
    from ultralytics import YOLO
    model = YOLO(str(copied))
    exported = Path(model.export(format="onnx", imgsz=args.image_size, batch=1, opset=12,
                                 simplify=False, dynamic=False, nms=False, device="cpu"))
    import onnx
    from onnxslim import slim
    optimized = slim(onnx.load(exported))
    onnx.checker.check_model(optimized)
    exported = exported.with_stem(exported.stem + "_slim")
    onnx.save(optimized, exported)
    manifest = dict(schema_version=1, architecture="yolo_v11_detection",
                    model_file=exported.name, sha256=hashlib.sha256(exported.read_bytes()).hexdigest(),
                    model_format="onnx", inference_device="onnxruntime_cpu", precision="float32",
                    intra_op_threads=1,
                    input_name="images", output_name="output0", input_size_wh=[args.image_size, args.image_size],
                    padding_value=114, layout="channels_first",
                    class_names=[model.names[i] for i in range(len(model.names))],
                    source="E023 CARLA RRD single-episode fine-tune; " + str(args.checkpoint),
                    checkpoint_sha256=hashlib.sha256(copied.read_bytes()).hexdigest(),
                    license="AGPL-3.0; Ultralytics https://github.com/ultralytics/ultralytics",
                    graph_transform="onnxslim 0.1.34 static constant folding",
                    validation_scope="simulation_candidate_not_real_target_validated")
    (args.output / "detector_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
