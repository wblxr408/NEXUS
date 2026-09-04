#!/usr/bin/env python3
"""Prepare one RRD episode for exploratory detection training, with time gaps.

Labels are simulator-projected boxes, not measured masks. Splits are consecutive
subclips of ONE episode, and must not be reported as independent-flight tests.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def time_splits(times, train_fraction=.6, val_fraction=.2, gap_seconds=1.):
    times = np.asarray(times, dtype=float)
    if (len(times) < 3 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)
            or min(train_fraction, val_fraction) <= 0 or train_fraction + val_fraction >= 1
            or not np.isfinite(gap_seconds) or gap_seconds < 0):
        raise ValueError("invalid timestamps or split parameters")
    duration = times[-1] - times[0]
    first = times[0] + duration * train_fraction
    second = times[0] + duration * (train_fraction + val_fraction)
    splits = np.full(len(times), "gap", dtype=object)
    splits[times < first] = "train"
    splits[(times >= first + gap_seconds) & (times < second)] = "val"
    splits[times >= second + gap_seconds] = "test"
    if any(not np.any(splits == split) for split in ("train", "val", "test")):
        raise ValueError("time gaps leave an empty split")
    return splits.tolist()


def normalized_box(box, width, height):
    xyxy = np.asarray(box, dtype=float)
    if xyxy.shape != (4,) or not np.isfinite(xyxy).all() or width <= 0 or height <= 0:
        raise ValueError("invalid box or image size")
    x1, y1, x2, y2 = xyxy
    if x2 <= x1 or y2 <= y1:
        raise ValueError("inverted or empty box")
    x1, x2 = np.clip([x1, x2], 0., width)
    y1, y2 = np.clip([y1, y2], 0., height)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("box outside image")
    return [(x1 + x2) / (2 * width), (y1 + y2) / (2 * height),
            (x2 - x1) / width, (y2 - y1) / height]


def prepare(episode, output, gap_seconds=1.):
    episode, output = Path(episode).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    manifest = json.loads((episode / "manifest.json").read_text())
    camera = json.loads((episode / "calibration/camera.json").read_text())
    frames = read_rows(episode / "frames.jsonl")
    rows = read_rows(episode / "vision_target_observations.jsonl")
    frame_ids = [row["frame"] for row in frames]
    if len(set(frame_ids)) != len(frames) or len(frames) != manifest["frames"]:
        raise ValueError("duplicate/missing frames")
    splits = time_splits([r["simulation"]["elapsed"] for r in frames], gap_seconds=gap_seconds)
    classes = sorted({r["class_id"] for r in rows})
    by_frame = {frame: [] for frame in frame_ids}
    for row in rows:
        if row["frame"] not in by_frame or row["episode_id"] != manifest["episode_id"]:
            raise ValueError("observation outside episode")
        by_frame[row["frame"]].append(row)
    output.mkdir(parents=True, exist_ok=False)
    records, hashes = [], {}
    for split in ("train", "val", "test"):
        (output / "images" / split).mkdir(parents=True)
        (output / "labels" / split).mkdir(parents=True)
    for row, split in zip(frames, splits):
        frame = row["frame"]
        source = episode / "sensors/rgb_mono" / f"{frame:06d}.png"
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (camera["height"], camera["width"]):
            raise ValueError(f"unreadable/wrong-size image: {source}")
        digest = hashlib.sha256(image.tobytes()).hexdigest()
        labels = []
        for obs in by_frame[frame]:
            if obs["valid"] and obs["bbox"]["visible"]:
                values = normalized_box(obs["bbox"]["bbox_xyxy"], camera["width"], camera["height"])
                labels.append(str(classes.index(obs["class_id"])) + " " + " ".join(f"{x:.9f}" for x in values))
        records.append({"frame": frame, "simulation_elapsed": row["simulation"]["elapsed"],
                        "split": split, "source_image": str(source.relative_to(episode)),
                        "decoded_bgr_sha256": digest, "labels": len(labels)})
        if split == "gap":
            continue
        if digest in hashes and hashes[digest] != split:
            raise ValueError("identical decoded images cross split boundary")
        hashes[digest] = split
        if not cv2.imwrite(str(output / "images" / split / source.name), image):
            raise OSError("image write failed")
        (output / "labels" / split / f"{frame:06d}.txt").write_text("\n".join(labels) + "\n")
    data = {"path": ".", "train": "images/train", "val": "images/val", "test": "images/test", "names": classes}
    (output / "dataset.yaml").write_text(yaml.safe_dump(data, sort_keys=False))
    report = {"source_kind": "CARLA_RRD_simulation", "episode_id": manifest["episode_id"],
              "split_method": "single_episode_contiguous_subclips_with_time_gaps",
              "independent_episode_test": False, "gap_seconds": gap_seconds,
              "label_source": "simulator_projected_3d_bounding_boxes", "classes": classes,
              "counts": {s: splits.count(s) for s in ("train", "val", "test", "gap")},
              "input_image_channels": "converted to BGR 3-channel PNG without resizing", "frames": records}
    (output / "split_manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "frames"}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--gap-seconds", type=float, default=1.)
    args = parser.parse_args()
    prepare(args.episode, args.output, args.gap_seconds)
