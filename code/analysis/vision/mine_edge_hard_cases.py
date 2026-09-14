"""Mine reviewable detector hard cases; never manufacture object labels.

The teacher runs on images which have only negative/background annotations or
unlabelled operational captures.  High-confidence false positives and
low-confidence expected detections are emitted as a review manifest.  A
human/annotation tool must accept or reject every candidate before it enters
the Nano student's YOLO dataset.
"""

import argparse
import json
from pathlib import Path

import cv2
from ultralytics import YOLO


def images(root):
    return sorted(path for suffix in ("*.jpg", "*.jpeg", "*.png", "*.bmp") for path in root.rglob(suffix))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--teacher", required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confidence", type=float, default=.25)
    parser.add_argument("--device", default="0")
    parser.add_argument("--source-condition", required=True,
                        help="e.g. occlusion, glare, similar_instance, NLOS_background")
    args = parser.parse_args()
    if not args.images.is_dir() or not .0 < args.confidence <= 1 or not args.source_condition.strip():
        raise ValueError("invalid mining inputs")
    paths = images(args.images)
    if not paths:
        raise ValueError("no supported images found")
    if args.output.exists():
        raise ValueError("output manifest already exists")
    teacher = YOLO(args.teacher)
    records = []
    for path, result in zip(paths, teacher.predict([str(path) for path in paths], conf=args.confidence,
                                                     device=args.device, stream=True, verbose=False)):
        image = cv2.imread(str(path))
        if image is None:
            continue
        boxes = []
        for box in result.boxes:
            x0, y0, x1, y1 = (float(value) for value in box.xyxy[0].tolist())
            boxes.append({"class_id": int(box.cls[0]), "confidence": float(box.conf[0]),
                          "bbox_xyxy_px": [x0, y0, x1, y1], "review_state": "pending"})
        records.append({"image": str(path.resolve()), "condition": args.source_condition,
                        "teacher_candidates": boxes, "review_required": True})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"schema_version": 1, "teacher": args.teacher,
                                       "records": records, "instruction": "Review every candidate before YOLO export."},
                                      ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
