#!/usr/bin/env python3
"""Validate the generated detector/pose dataset without reading map truth as input."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def _load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    args = parser.parse_args()
    root = Path(args.dataset)
    manifest, calibration, classes, splits = (_load(root / "dataset_manifest.json"), _load(root / "calibration" / "camera_info.json"),
                                               _load(root / "target_classes.json"), _load(root / "split_manifest.json"))
    width, height = calibration["image_width_px"], calibration["image_height_px"]
    camera_matrix = np.asarray(calibration["K"], dtype=float)
    assert camera_matrix.shape == (3, 3) and camera_matrix[0, 0] > 0 and camera_matrix[1, 1] > 0
    assert len(classes["categories"]) == 10
    assert splits["no_trajectory_overlap"]
    report = {"dataset_id": manifest["dataset_id"], "image_size": [width, height], "splits": {}, "status": "pass"}
    for split in ("train", "val", "test"):
        split_dir = root / ("train_pbr" if split == "train" else split) / "000001"
        coco = _load(split_dir / "instances_coco.json")
        scene_gt = _load(split_dir / "scene_gt.json")
        scene_camera = _load(split_dir / "scene_camera.json")
        assert len(coco["images"]) == splits["splits"][split]["frame_count"]
        assert len(scene_gt) == len(coco["images"]) == len(scene_camera)
        image_ids = {item["id"] for item in coco["images"]}
        visible_by_class = {category["id"]: 0 for category in classes["categories"]}
        for annotation in coco["annotations"]:
            assert annotation["image_id"] in image_ids
            x, y, box_width, box_height = annotation["bbox"]
            assert 0 <= x < width and 0 <= y < height and box_width > 0 and box_height > 0
            assert x + box_width <= width and y + box_height <= height
            mask = split_dir / annotation["mask_file"]
            assert mask.is_file()
            visible_by_class[annotation["category_id"]] += 1
        for image in coco["images"][:1]:
            pixels = cv2.imread(str(split_dir / image["file_name"]), cv2.IMREAD_COLOR)
            assert pixels is not None and pixels.shape[:2] == (height, width)
        report["splits"][split] = {"frames": len(coco["images"]), "visible_annotations": len(coco["annotations"]),
                                    "visible_by_class": visible_by_class}
        yolo_split = "train" if split == "train" else split
        yolo_images = root / "detector_yolo" / "images" / yolo_split
        yolo_labels = root / "detector_yolo" / "labels" / yolo_split
        assert len(list(yolo_images.glob("*.png"))) == len(coco["images"])
        assert len(list(yolo_labels.glob("*.txt"))) == len(coco["images"])
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
