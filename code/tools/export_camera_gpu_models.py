"""Export the existing trained models as fixed-shape CUDA TorchScript assets."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import torch


def save_model(model, sample, manifest, output, name):
    output.mkdir(parents=True, exist_ok=False)
    with torch.inference_mode():
        model(sample)  # Initialize shape-dependent detector grids on CUDA.
        traced = torch.jit.trace(model, sample, strict=False)
        result = traced(sample)
    tensors = result if isinstance(result, tuple) else (result,)
    assert all(tensor.is_cuda for tensor in tensors)
    path = output / (name + "_gpu.pt")
    torch.jit.save(traced, str(path))
    manifest.update(model_format="torchscript", inference_device="cuda:0", precision="float32",
                    model_file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    export_torch_version=torch.__version__, export_gpu=torch.cuda.get_device_name(0))
    (output / (name + "_manifest.json")).write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"CUDA_EXPORT_OK {path} outputs={[tuple(t.shape) for t in tensors]}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--previous-assets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required; CPU export inference is disabled")
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    from ultralytics import YOLO
    detector = YOLO(str(args.checkpoint)).model.eval().cuda()
    head = detector.model[-1]
    head.export, head.format, head.dynamic = True, "torchscript", False
    manifest = json.loads((args.previous_assets / "detector/detector_slim_manifest.json").read_text())
    manifest.pop("graph_transform", None)
    save_model(detector, torch.zeros(1, 3, 640, 640, device="cuda"), manifest, args.output / "detector", "detector")

    spec = importlib.util.spec_from_file_location("superpoint_mit", args.previous_assets / "superpoint_pytorch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    superpoint = module.SuperPoint().eval()
    superpoint.load_state_dict(torch.load(args.previous_assets / "superpoint_v6_from_tf.pth", weights_only=True))

    class Dense(torch.nn.Module):
        def __init__(self, network):
            super().__init__()
            self.network = network

        def forward(self, image):
            features = self.network.backbone(image)
            return self.network.detector(features), self.network.descriptor(features)

    manifest = json.loads((args.previous_assets / "superpoint/superpoint_manifest.json").read_text())
    manifest["license"] = "MIT; original LICENSE at ../LICENSE_superpoint.txt"
    save_model(Dense(superpoint).eval().cuda(), torch.zeros(1, 1, 480, 640, device="cuda"),
               manifest, args.output / "superpoint", "superpoint")
    (args.output / "LICENSE_superpoint.txt").write_bytes((args.previous_assets / "LICENSE_superpoint.txt").read_bytes())


if __name__ == "__main__":
    main()
