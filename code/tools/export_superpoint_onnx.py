"""Export dense heads of the MIT-licensed rpautrat/SuperPoint model."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import torch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-module", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("superpoint_mit", args.source_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = module.SuperPoint().eval()
    model.load_state_dict(torch.load(args.weights, map_location="cpu", weights_only=True), strict=True)

    class Dense(torch.nn.Module):
        def __init__(self, network):
            super().__init__()
            self.network = network

        def forward(self, image):
            features = self.network.backbone(image)
            return self.network.detector(features), self.network.descriptor(features)

    args.output.mkdir(parents=True, exist_ok=False)
    path = args.output / "superpoint.onnx"
    network = Dense(model).eval()
    torch.onnx.export(network, torch.zeros(1, 1, 480, 640), str(path),
                      input_names=["image"], output_names=["logits", "descriptors"],
                      opset_version=12, do_constant_folding=True, dynamo=False)
    manifest = dict(schema_version=1, architecture="superpoint_dense_v1",
                    descriptor_sampling="superpoint_mit_v1", model_file=path.name,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest(), input_name="image",
                    detector_output="logits", descriptor_output="descriptors", input_size_wh=[640, 480],
                    source="https://github.com/rpautrat/SuperPoint/tree/1411bbd68c50163555d39c1b26e9e046ebd48f27",
                    license="MIT; see ../LICENSE_superpoint.txt",
                    weights_sha256=hashlib.sha256(args.weights.read_bytes()).hexdigest(),
                    source_sha256=hashlib.sha256(args.source_module.read_bytes()).hexdigest())
    (args.output / "superpoint_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
