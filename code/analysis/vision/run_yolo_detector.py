#!/usr/bin/env python3
"""Train or run an external Ultralytics YOLO detector for GDR-Net crops.

This wrapper deliberately sits before GDR-Net.  It neither changes upstream
GDR-Net nor injects BOP ground-truth bounding boxes at inference.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from ultralytics import YOLO
import yaml


def train(args: argparse.Namespace) -> None:
    model = YOLO(args.model)
    # Ultralytics resolves a relative ``path: .`` against its current working
    # directory on Windows, which may be a UNC project directory rather than
    # the dataset YAML's directory. Materialize an absolute local copy.
    data_path = Path(args.data).resolve()
    data = yaml.safe_load(data_path.read_text(encoding="utf-8"))
    if not Path(data.get("path", ".")).is_absolute():
        data["path"] = str(data_path.parent)
    resolved_path = data_path.with_name("dataset.resolved.yaml")
    resolved_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    model.train(data=str(resolved_path), epochs=args.epochs, imgsz=args.image_size, batch=args.batch_size,
                project=args.project, name=args.name, seed=args.seed, deterministic=True, pretrained=True,
                amp=not args.disable_amp, workers=args.workers)


def predict(args: argparse.Namespace) -> None:
    model = YOLO(args.weights)
    records: dict[str, list[dict]] = {}
    results = model.predict(source=args.images, imgsz=args.image_size, conf=args.confidence,
                            save=False, verbose=False, stream=True)
    for result in results:
        frame_id = str(int(Path(result.path).stem))
        detections = []
        for box in result.boxes:
            x, y, width, height = (float(value) for value in box.xywh[0].tolist())
            # Dataset YOLO labels are zero based; BOP/GDR-Net object IDs start at one.
            detections.append({"class_id": int(box.cls[0]) + 1, "bbox_xywh_px": [x - width / 2.0, y - height / 2.0, width, height],
                               "confidence": float(box.conf[0])})
        records[frame_id] = detections
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(records, indent=2) + "\n", encoding="utf-8")
    print(f"wrote detector outputs for {len(records)} frames to {destination}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    train_parser = subcommands.add_parser("train")
    train_parser.add_argument("--data", required=True, help="detector_yolo/dataset.yaml")
    train_parser.add_argument("--model", default="yolo11n.pt")
    train_parser.add_argument("--epochs", type=int, default=100)
    train_parser.add_argument("--image-size", type=int, default=960)
    train_parser.add_argument("--batch-size", type=int, default=8)
    train_parser.add_argument("--project", required=True)
    train_parser.add_argument("--name", default="nexus_sandbox_targets")
    train_parser.add_argument("--seed", type=int, default=20260830)
    train_parser.add_argument("--disable-amp", action="store_true", help="avoid Ultralytics AMP self-test downloads in offline Windows runs")
    train_parser.add_argument("--workers", type=int, default=0, help="data-loader workers; 0 avoids CUDA DLL respawn failures through WSL UNC paths")
    infer_parser = subcommands.add_parser("predict")
    infer_parser.add_argument("--weights", required=True)
    infer_parser.add_argument("--images", required=True)
    infer_parser.add_argument("--output", required=True)
    infer_parser.add_argument("--image-size", type=int, default=960)
    infer_parser.add_argument("--confidence", type=float, default=0.25)
    args = parser.parse_args()
    train(args) if args.command == "train" else predict(args)


if __name__ == "__main__":
    main()
