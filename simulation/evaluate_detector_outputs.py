#!/usr/bin/env python3
"""Evaluate external detector class+bbox output against held-out COCO labels."""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def iou(first: list[float], second: list[float]) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[0] + first[2], second[0] + second[2]), min(first[1] + first[3], second[1] + second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    union = first[2] * first[3] + second[2] * second[3] - intersection
    return intersection / union if union > 0 else 0.0


def average_precision(predictions: list[dict], truth: dict[int, list[dict]], threshold: float) -> tuple[float, int, int, int]:
    matched: dict[int, set[int]] = defaultdict(set)
    true_positive, false_positive = [], []
    for prediction in sorted(predictions, key=lambda item: item["confidence"], reverse=True):
        choices = truth.get(prediction["image_id"], [])
        best_index, best_iou = -1, 0.0
        for index, item in enumerate(choices):
            overlap = iou(prediction["bbox"], item["bbox"])
            if index not in matched[prediction["image_id"]] and overlap > best_iou:
                best_index, best_iou = index, overlap
        if best_index >= 0 and best_iou >= threshold:
            matched[prediction["image_id"]].add(best_index)
            true_positive.append(1); false_positive.append(0)
        else:
            true_positive.append(0); false_positive.append(1)
    positives = sum(len(values) for values in truth.values())
    if positives == 0:
        return 0.0, 0, len(predictions), 0
    tp = np.cumsum(true_positive, dtype=float)
    fp = np.cumsum(false_positive, dtype=float)
    recalls = tp / positives
    precisions = tp / np.maximum(tp + fp, 1.0)
    padded_recall = np.concatenate(([0.0], recalls, [1.0]))
    padded_precision = np.concatenate(([0.0], precisions, [0.0]))
    for index in range(len(padded_precision) - 1, 0, -1):
        padded_precision[index - 1] = max(padded_precision[index - 1], padded_precision[index])
    changes = np.where(padded_recall[1:] != padded_recall[:-1])[0]
    ap = float(np.sum((padded_recall[changes + 1] - padded_recall[changes]) * padded_precision[changes + 1]))
    count_tp = int(tp[-1]) if len(tp) else 0
    return ap, count_tp, len(predictions) - count_tp, positives - count_tp


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--detections", required=True)
    parser.add_argument("--split", default="test")
    parser.add_argument("--out", required=True)
    parser.add_argument("--iou", type=float, default=0.5)
    args = parser.parse_args()
    root = Path(args.dataset) / ("train_pbr" if args.split == "train" else args.split) / "000001"
    coco = json.loads((root / "instances_coco.json").read_text(encoding="utf-8"))
    frame_to_image = {str(item["sequence"]): item["id"] for item in coco["images"]}
    truth: dict[int, dict[int, list[dict]]] = defaultdict(lambda: defaultdict(list))
    for annotation in coco["annotations"]:
        truth[annotation["category_id"]][annotation["image_id"]].append(annotation)
    raw = json.loads(Path(args.detections).read_text(encoding="utf-8"))
    predictions: dict[int, list[dict]] = defaultdict(list)
    active_frames = 0
    for frame_id, records in raw.get("frames", raw).items():
        if records:
            active_frames += 1
        image_id = frame_to_image.get(str(int(frame_id)))
        if image_id is None:
            continue
        for record in records:
            class_id = int(record.get("class_id", record.get("obj_id")))
            predictions[class_id].append({"image_id": image_id, "bbox": list(record.get("bbox_xywh_px", record.get("bbox_xywh"))), "confidence": float(record["confidence"])})
    per_class, total_tp, total_fp, total_fn = {}, 0, 0, 0
    for category in coco["categories"]:
        class_id = int(category["id"])
        ap, tp, fp, fn = average_precision(predictions[class_id], truth[class_id], args.iou)
        per_class[str(class_id)] = {"class_name": category["name"], "ap": ap, "tp": tp, "fp": fp, "fn": fn}
        total_tp += tp; total_fp += fp; total_fn += fn
    precision = total_tp / (total_tp + total_fp) if total_tp + total_fp else 0.0
    recall = total_tp / (total_tp + total_fn) if total_tp + total_fn else 0.0
    report = {"algorithm": "ultralytics_yolo_external_detector", "split": args.split, "iou_threshold": args.iou,
              "expected_instances": total_tp + total_fn, "detected_instances": total_tp + total_fp,
              "true_positive": total_tp, "false_positive": total_fp, "false_negative": total_fn,
              "precision": precision, "recall": recall, "f1": 2 * precision * recall / (precision + recall) if precision + recall else 0.0,
              "map50": float(np.mean([item["ap"] for item in per_class.values()])),
              "frame_availability": active_frames / len(coco["images"]), "per_class": per_class}
    destination = Path(args.out); destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
