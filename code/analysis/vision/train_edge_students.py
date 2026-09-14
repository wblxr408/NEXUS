"""Train/export edge students without conflating simulation labels with field proof.

The detector student uses an explicit YOLO Nano checkpoint.  Identity student
inputs are saved SuperPoint descriptor pairs labelled by instance identity;
the script fits a compact projection suitable for LightweightIdentityTransformer.
No labels are inferred from the student's own predictions.
"""

import argparse
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np


def train_detector(args):
    from ultralytics import YOLO
    dataset = Path(args.dataset).resolve()
    if not dataset.is_file():
        raise ValueError("detector dataset YAML is required")
    import yaml
    document = yaml.safe_load(dataset.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not document.get("train") or not document.get("val"):
        raise ValueError("detector dataset requires train and val splits")
    declared_root = Path(document.get("path", "."))
    root = declared_root if declared_root.is_absolute() else (dataset.parent / declared_root).resolve()
    for split in ("train", "val", "test"):
        if split in document:
            candidate = Path(document[split])
            document[split] = str(candidate if candidate.is_absolute() else (root / candidate).resolve())
    document.pop("path", None)
    # Ultralytics otherwise resolves path:. against its process CWD.  The
    # temporary absolute-path YAML keeps source labels untouched.
    temporary = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", encoding="utf-8", delete=False)
    try:
        yaml.safe_dump(document, temporary, sort_keys=False)
        temporary.close()
        training_data = temporary.name
    except Exception:
        temporary.close()
        Path(temporary.name).unlink(missing_ok=True)
        raise
    # A teacher checkpoint is provenance for hard-example mining / offline KD;
    # it must never silently turn the supposed Nano student into a large model.
    model = YOLO(args.student)
    try:
        results = model.train(data=training_data, epochs=args.epochs, imgsz=args.image_size, batch=args.batch,
                              device=args.device, deterministic=True, amp=False, project=str(args.output), name="detector_student")
    finally:
        Path(temporary.name).unlink(missing_ok=True)
    best = Path(results.save_dir) / "weights/best.pt"
    if not best.is_file():
        raise RuntimeError("YOLO training did not produce best.pt")
    return {"kind": "detector_student", "architecture": "yolo11n", "weights": str(best),
            "sha256": hashlib.sha256(best.read_bytes()).hexdigest(), "dataset": str(dataset),
            "teacher": args.teacher or None, "student_initialization": args.student,
            "training_strategy": "supervised_nano_with_labelled_hard_cases; teacher_candidates_require_review",
            "label_provenance": args.label_provenance}


def train_identity(args):
    data = np.load(args.pairs, allow_pickle=False)
    reference, candidate, same = data["reference"], data["candidate"], data["same"]
    if reference.ndim != 2 or reference.shape != candidate.shape or reference.shape[1] != 256 or same.shape != (len(reference),):
        raise ValueError("identity pair archive requires Nx256 reference/candidate and N labels")
    if not np.all(np.isfinite(reference)) or not np.all(np.isfinite(candidate)) or not np.all(np.isin(same, [0, 1])):
        raise ValueError("identity training data is invalid")
    # Fisher-style compact projection: discriminative, deterministic, and
    # exported as the q/k/v matrices consumed by the online attention block.
    difference = reference - candidate
    positive, negative = difference[same.astype(bool)], difference[~same.astype(bool)]
    if not len(positive) or not len(negative):
        raise ValueError("identity student requires positive and hard-negative pairs")
    covariance = np.cov(difference.T) + np.eye(256) * 1e-4
    direction = np.linalg.solve(covariance, positive.mean(axis=0) - negative.mean(axis=0))
    direction /= max(1e-8, np.linalg.norm(direction))
    basis = np.eye(256, 32, dtype=np.float32)
    basis[:, 0] = direction.astype(np.float32)
    args.output.mkdir(parents=True, exist_ok=False)
    path = args.output / "identity_student.npz"
    np.savez_compressed(path, query=basis, key=basis, value=basis,
                        metadata=json.dumps({"label_provenance": args.label_provenance, "pairs": int(len(same)),
                                             "positive": int(same.sum()), "negative": int((1 - same).sum())}))
    return {"kind": "identity_student", "weights": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "label_provenance": args.label_provenance}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=("detector", "identity"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label-provenance", required=True)
    parser.add_argument("--dataset")
    parser.add_argument("--teacher", default="")
    parser.add_argument("--student", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0")
    parser.add_argument("--pairs", type=Path)
    args = parser.parse_args()
    if not args.label_provenance.strip():
        raise ValueError("label provenance is required")
    if args.kind == "detector":
        if not args.dataset or args.epochs < 1 or args.image_size < 32 or args.batch < 1:
            raise ValueError("detector arguments are invalid")
        report = train_detector(args)
    else:
        if args.pairs is None or not args.pairs.is_file():
            raise ValueError("identity descriptor-pair archive is required")
        report = train_identity(args)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
