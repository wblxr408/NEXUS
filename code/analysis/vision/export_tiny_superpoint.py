"""Export a trained Tiny-SuperPoint state at an explicit edge input size."""

import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--dynamic-input", action="store_true",
                        help="Export dynamic H/W axes so adaptive image_scale reduces actual tensor work")
    args = parser.parse_args()
    if (not args.checkpoint.is_file() or args.output.exists() or min(args.width, args.height) < 8
            or args.width % 8 or args.height % 8 or args.threads < 1):
        raise ValueError("checkpoint/output/input dimensions/threads are invalid")
    import torch
    from train_tiny_superpoint import tiny_superpoint
    model = tiny_superpoint(torch).eval()
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu", weights_only=True))
    args.output.mkdir(parents=True)
    path = args.output / "tiny_superpoint.onnx"
    dynamic_axes = None
    if args.dynamic_input:
        dynamic_axes = {"image": {2: "height", 3: "width"},
                        "logits": {2: "coarse_height", 3: "coarse_width"},
                        "descriptors": {2: "coarse_height", 3: "coarse_width"}}
    torch.onnx.export(model, torch.zeros(1, 1, args.height, args.width), str(path), input_names=["image"],
                      output_names=["logits", "descriptors"], opset_version=12, do_constant_folding=True,
                      dynamic_axes=dynamic_axes, dynamo=False)
    manifest = {"schema_version": 1, "architecture": "superpoint_dense_v1", "descriptor_sampling": "superpoint_mit_v1",
                "feature_space_id": "tiny_superpoint_distilled_v1", "feature_space_compatible_with": "superpoint_mit_v1",
                "model_file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "model_format": "onnx",
                "inference_device": "onnxruntime_cpu", "precision": "float32", "intra_op_threads": args.threads,
                "input_name": "image", "detector_output": "logits", "descriptor_output": "descriptors",
                "input_size_wh": [args.width, args.height], "source": f"Tiny-SuperPoint state {args.checkpoint}",
                "dynamic_input": args.dynamic_input,
                "license": "student architecture project-local; teacher MIT SuperPoint",
                "checkpoint_sha256": hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                "validation_scope": "distillation_candidate_not_field_validated"}
    (args.output / "superpoint_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"model": str(path), "manifest": str(args.output / "superpoint_manifest.json")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
