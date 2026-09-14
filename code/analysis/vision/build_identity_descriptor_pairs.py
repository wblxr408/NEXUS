"""Build labelled 256-D instance descriptor pairs from RRD target annotations."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def roi_descriptor(features, bbox):
    selected = features.in_roi(bbox)
    if len(selected.descriptors) < 2:
        return None
    descriptor = selected.descriptors.mean(axis=0)
    norm = float(np.linalg.norm(descriptor))
    return None if norm < 1e-8 else (descriptor / norm).astype(np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--observations", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--superpoint-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-frames", type=int, default=180)
    args = parser.parse_args()
    if (not args.observations.is_file() or not args.images.is_dir() or not args.superpoint_manifest.is_file()
            or args.output.exists() or args.maximum_frames < 2):
        raise ValueError("identity pair inputs are invalid")
    from nexus_vision_localization.superpoint_frontend import SuperPointOnnx
    by_frame = {}
    for line in args.observations.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        if row.get("valid") and row.get("bbox", {}).get("visible"):
            by_frame.setdefault(int(row["frame"]), []).append(row)
    frames = sorted(by_frame)
    if len(frames) > args.maximum_frames:
        indices = np.linspace(0, len(frames) - 1, args.maximum_frames, dtype=int)
        frames = [frames[index] for index in indices]
    images = {path.stem: path for suffix in ("*.jpg", "*.jpeg", "*.png") for path in args.images.rglob(suffix)}
    model = SuperPointOnnx(args.superpoint_manifest)
    descriptors, negative = {}, []
    for frame in frames:
        path = images.get(f"{frame:06d}")
        if path is None:
            continue
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"unreadable RRD image: {path}")
        features = model.infer(image).features(maximum_points=1024)
        current = []
        for row in by_frame[frame]:
            x0, y0, x1, y1 = row["bbox"]["bbox_xyxy"]
            descriptor = roi_descriptor(features, (x0, y0, x1 - x0, y1 - y0))
            if descriptor is not None:
                descriptors.setdefault(row["target_id"], []).append((frame, descriptor))
                current.append((row["target_id"], descriptor))
        for index, (identifier, reference) in enumerate(current):
            for other, candidate in current[index + 1:]:
                if identifier != other:
                    negative.append((reference, candidate))
    positive = []
    for values in descriptors.values():
        for (_, reference), (_, candidate) in zip(values, values[1:]):
            positive.append((reference, candidate))
    count = min(len(positive), len(negative))
    if count < 8:
        raise ValueError("insufficient labelled identity descriptor pairs")
    pairs = positive[:count] + negative[:count]
    reference = np.stack([pair[0] for pair in pairs]).astype(np.float32)
    candidate = np.stack([pair[1] for pair in pairs]).astype(np.float32)
    same = np.array([1] * count + [0] * count, dtype=np.uint8)
    np.savez_compressed(args.output, reference=reference, candidate=candidate, same=same,
                        metadata=json.dumps({"source": str(args.observations), "frames": len(frames),
                                             "positive": count, "negative": count,
                                             "labels": "same target_id across frame / distinct target_id within frame"}))
    print(json.dumps({"pairs": int(len(same)), "positive": int(count), "negative": int(count),
                      "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
