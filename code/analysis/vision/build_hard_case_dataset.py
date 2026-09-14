"""Build deterministic labelled visual hard cases for the Nano detector.

This is augmentation, not pseudo-labelling: every output starts from a human
or simulator YOLO label file.  The manifest keeps the exact source, condition
and seed so a hard-case subset can be removed or re-generated later.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_labels(path):
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        values = line.split()
        if len(values) != 5:
            raise ValueError(f"invalid YOLO label row: {path}")
        category, *box = values
        if not category.isdigit() or any(not 0. <= float(value) <= 1. for value in box):
            raise ValueError(f"invalid YOLO label value: {path}")
        rows.append((int(category), *(float(value) for value in box)))
    return rows


def motion_blur(image, rng):
    size = int(rng.integers(5, 18)) | 1
    kernel = np.zeros((size, size), np.float32)
    angle = float(rng.uniform(0., np.pi))
    line = cv2.line(kernel, (size // 2, size // 2),
                    (int(size // 2 + np.cos(angle) * size), int(size // 2 + np.sin(angle) * size)), 1., 1)
    kernel = line / max(1., line.sum())
    return cv2.filter2D(image, -1, kernel)


def occlusion(image, rng):
    result = image.copy()
    height, width = result.shape[:2]
    for _ in range(int(rng.integers(1, 4))):
        x0, y0 = int(rng.integers(0, max(1, width - width // 4))), int(rng.integers(0, max(1, height - height // 4)))
        x1, y1 = min(width, x0 + int(rng.integers(width // 12, width // 3))), min(height, y0 + int(rng.integers(height // 12, height // 3)))
        colour = rng.integers(0, 256, 3).tolist()
        cv2.rectangle(result, (x0, y0), (x1, y1), colour, -1)
    return result


def clutter_and_low_contrast(image, rng):
    result = cv2.convertScaleAbs(image, alpha=float(rng.uniform(.45, .8)), beta=int(rng.integers(-20, 31)))
    height, width = result.shape[:2]
    overlay = result.copy()
    for _ in range(int(rng.integers(12, 28))):
        center = tuple(int(value) for value in rng.integers([0, 0], [width, height]))
        cv2.circle(overlay, center, int(rng.integers(2, max(3, min(width, height) // 15))),
                   rng.integers(0, 256, 3).tolist(), -1)
    return cv2.addWeighted(result, .78, overlay, .22, 0.)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True, help="source YOLO YAML")
    parser.add_argument("--output", type=Path, required=True, help="new hard-case dataset root")
    parser.add_argument("--seed", type=int, default=20260906)
    parser.add_argument("--splits", default="train,val", help="comma-separated source splits")
    args = parser.parse_args()
    if not args.dataset.is_file() or args.output.exists():
        raise ValueError("source dataset must exist and output must be new")
    source = yaml.safe_load(args.dataset.read_text(encoding="utf-8"))
    if source.get("path"):
        declared = Path(source["path"])
        root = declared if declared.is_absolute() else (args.dataset.parent / declared).resolve()
    else:
        root = args.dataset.parent.resolve()
    selected = [item.strip() for item in args.splits.split(",") if item.strip()]
    if not selected or "train" not in selected or any(item not in source for item in selected):
        raise ValueError("requested YOLO dataset splits must include train and exist in source")
    transforms = {"motion_blur": motion_blur, "occlusion": occlusion, "clutter_low_contrast": clutter_and_low_contrast}
    rng = np.random.default_rng(args.seed)
    records = []
    args.output.mkdir(parents=True)
    for split in selected:
        image_root = (root / source[split]).resolve()
        if not image_root.is_dir():
            raise ValueError(f"image split missing: {image_root}")
        for image_path in sorted(path for suffix in ("*.jpg", "*.jpeg", "*.png") for path in image_root.rglob(suffix)):
            relative = image_path.relative_to(image_root)
            label_path = root / "labels" / split / relative.with_suffix(".txt")
            if not label_path.is_file():
                raise ValueError(f"label missing: {label_path}")
            parse_labels(label_path)
            image = cv2.imread(str(image_path))
            if image is None:
                raise ValueError(f"image unreadable: {image_path}")
            for condition, transform in transforms.items():
                output_image = args.output / "images" / split / relative.with_stem(relative.stem + "_" + condition)
                output_label = args.output / "labels" / split / relative.with_stem(relative.stem + "_" + condition).with_suffix(".txt")
                output_image.parent.mkdir(parents=True, exist_ok=True)
                output_label.parent.mkdir(parents=True, exist_ok=True)
                augmented = transform(image, rng)
                if not cv2.imwrite(str(output_image), augmented):
                    raise RuntimeError(f"failed to write: {output_image}")
                output_label.write_bytes(label_path.read_bytes())
                records.append({"source_image": str(image_path), "source_label_sha256": digest(label_path),
                                "image": str(output_image.relative_to(args.output)), "condition": condition})
    names = source.get("names")
    if not isinstance(names, list) or not names:
        raise ValueError("source YOLO dataset requires nonempty ordered names")
    document = {"path": str(args.output), "train": "images/train", "val": "images/val", "test": "images/test",
                "nc": int(source.get("nc", len(names))), "names": names, "source_dataset": str(args.dataset.resolve()),
                "label_policy": "source YOLO boxes retained for labelled partial occlusion", "seed": args.seed}
    # Do not claim an absent split exists; YOLO supports only train for mining.
    for split in ("val", "test"):
        if not (args.output / "images" / split).is_dir():
            document.pop(split)
    (args.output / "dataset.yaml").write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    (args.output / "hard_case_manifest.json").write_text(json.dumps({"schema_version": 1, "records": records}, ensure_ascii=False, indent=2),
                                                          encoding="utf-8")


if __name__ == "__main__":
    main()
