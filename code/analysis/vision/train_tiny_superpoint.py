"""Distil a compact sparse-feature student from the deployed SuperPoint heads.

The student preserves the 65-channel detector and 256-dimensional descriptor
contract consumed by ``SuperPointOnnx``.  It is trained only against explicit
teacher outputs on supplied images; it never invents correspondence labels.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def image_paths(root):
    return sorted(path for suffix in ("*.jpg", "*.jpeg", "*.png", "*.bmp") for path in root.rglob(suffix))


def tiny_superpoint(torch):
    class TinySuperPoint(torch.nn.Module):
        def __init__(self):
            super().__init__()
            def block(source, target):
                return torch.nn.Sequential(torch.nn.Conv2d(source, target, 3, padding=1), torch.nn.ReLU(inplace=True),
                                           torch.nn.Conv2d(target, target, 3, padding=1), torch.nn.ReLU(inplace=True))
            self.stage1 = block(1, 24)
            self.stage2 = block(24, 48)
            self.stage3 = block(48, 80)
            self.stage4 = block(80, 112)
            self.pool = torch.nn.MaxPool2d(2, 2)
            self.detector = torch.nn.Sequential(torch.nn.Conv2d(112, 112, 3, padding=1), torch.nn.ReLU(inplace=True),
                                                torch.nn.Conv2d(112, 65, 1))
            self.descriptor = torch.nn.Sequential(torch.nn.Conv2d(112, 192, 3, padding=1), torch.nn.ReLU(inplace=True),
                                                  torch.nn.Conv2d(192, 256, 1))

        def forward(self, image):
            features = self.pool(self.stage1(image))
            features = self.pool(self.stage2(features))
            features = self.pool(self.stage3(features))
            features = self.stage4(features)
            descriptor = self.descriptor(features)
            return self.detector(features), descriptor / torch.clamp(torch.linalg.vector_norm(descriptor, dim=1, keepdim=True), min=1e-8)
    return TinySuperPoint()


def batches(paths, batch_size, device, torch):
    for start in range(0, len(paths), batch_size):
        values = []
        for path in paths[start:start + batch_size]:
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f"unreadable image: {path}")
            image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
            values.append(image)
        yield torch.from_numpy(np.stack(values)[:, None].astype(np.float32) / 255.).to(device)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--teacher", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    if not args.teacher.is_file() or not args.images.is_dir() or args.output.exists() or args.epochs < 1 or args.batch < 1:
        raise ValueError("teacher/images/output/epochs/batch are invalid")
    paths = image_paths(args.images)
    if not paths:
        raise ValueError("no distillation images")
    import torch
    import torch.nn.functional as functional
    device = torch.device(args.device)
    teacher = torch.jit.load(str(args.teacher), map_location=device).eval()
    student = tiny_superpoint(torch).to(device).train()
    optimizer = torch.optim.AdamW(student.parameters(), lr=2e-3, weight_decay=1e-5)
    for _ in range(args.epochs):
        for image in batches(paths, args.batch, device, torch):
            # ``no_grad`` keeps the teacher frozen while producing ordinary
            # tensors that can participate in the student's backward pass.
            with torch.no_grad():
                teacher_logits, teacher_descriptors = teacher(image)
                teacher_descriptors = functional.normalize(teacher_descriptors, dim=1)
            logits, descriptors = student(image)
            # Sparse SuperPoint cells have a dominant dustbin class. Give the
            # teacher's actual 64 pixel-location probabilities more weight so
            # a student cannot minimise loss by predicting a flat dustbin map.
            temperature = .5
            teacher_probability = functional.softmax(teacher_logits / temperature, dim=1)
            student_probability = functional.softmax(logits / temperature, dim=1)
            keypoint_weight = 1. + 96. * teacher_probability[:, :64]
            detector_loss = ((student_probability[:, :64] - teacher_probability[:, :64]).square() * keypoint_weight).mean()
            detector_loss += .1 * temperature ** 2 * functional.kl_div(
                functional.log_softmax(logits / temperature, dim=1),
                teacher_probability, reduction="mean")
            descriptor_loss = (1. - (descriptors * teacher_descriptors).sum(dim=1)).mean()
            loss = detector_loss + 3. * descriptor_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
    args.output.mkdir(parents=True)
    checkpoint = args.output / "tiny_superpoint_state.pt"
    torch.save(student.cpu().state_dict(), checkpoint)
    student.eval()
    model = args.output / "tiny_superpoint.onnx"
    torch.onnx.export(student, torch.zeros(1, 1, 480, 640), str(model), input_names=["image"],
                      output_names=["logits", "descriptors"], opset_version=12, do_constant_folding=True, dynamo=False)
    manifest = {"schema_version": 1, "architecture": "superpoint_dense_v1", "descriptor_sampling": "superpoint_mit_v1",
                "feature_space_id": "tiny_superpoint_distilled_v1", "feature_space_compatible_with": "superpoint_mit_v1",
                "model_file": model.name, "sha256": hashlib.sha256(model.read_bytes()).hexdigest(), "model_format": "onnx",
                "inference_device": "onnxruntime_cpu", "precision": "float32", "intra_op_threads": 4,
                "input_name": "image", "detector_output": "logits", "descriptor_output": "descriptors", "input_size_wh": [640, 480],
                "source": f"distilled from {args.teacher}", "license": "student architecture project-local; teacher MIT SuperPoint",
                "teacher_sha256": hashlib.sha256(args.teacher.read_bytes()).hexdigest(), "training_images": len(paths),
                "training_epochs": args.epochs, "validation_scope": "distillation_candidate_not_field_validated"}
    (args.output / "superpoint_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"model": str(model), "manifest": str(args.output / "superpoint_manifest.json"),
                      "parameters": sum(value.numel() for value in student.parameters())}, ensure_ascii=False))


if __name__ == "__main__":
    main()
