#!/usr/bin/env python3
"""Train and infer a small upstream-GDRN model on NEXUS synthetic BOP data.

This is a deliberately narrow custom-data adapter: it reuses the upstream
GDRN backbone, dense-coordinate head and ConvPnP head, while supplying a
minimal PyTorch loop for the project's parameterised BOP records.  It does not
claim compatibility with the authors' LM/LM-O/YCB-V training recipe, and it
uses BOP ground-truth ROI boxes as the detector input for both train and test.

Run it with the dedicated Windows ``gdr_net`` Conda interpreter.  The output
prediction JSON contains only model output poses (metres) and can be evaluated
from WSL with ``pose_evaluate_cli.py``.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from gdr_net_numpy_compat import apply_numpy2_compat


PROJECT_ROOT = Path(__file__).resolve().parents[3]
UPSTREAM_ROOT = PROJECT_ROOT / "code/analysis/vision/third_party/gdr_net"
sys.path.insert(0, str(UPSTREAM_ROOT))
apply_numpy2_compat()

import torch
from mmcv import Config
from torch.utils.data import DataLoader, Dataset

from detectron2.utils.events import EventStorage
from core.gdrn_modeling.models import GDRN
from core.utils.data_utils import crop_resize_by_warp_affine


@dataclass(frozen=True)
class PoseSample:
    image_path: Path
    frame_id: str
    object_id: int
    bbox_xywh: tuple[float, float, float, float]
    rotation: np.ndarray
    translation_m: np.ndarray
    extent_m: np.ndarray


class BopPoseDataset(Dataset):
    """One GT ROI per BOP object instance, with no map-frame truth access."""

    def __init__(self, root: Path, split: str, pad_scale: float = 1.5):
        self.root = root
        self.scene_root = root / split / "000001"
        self.pad_scale = float(pad_scale)
        if not self.scene_root.is_dir():
            raise FileNotFoundError(f"BOP scene not found: {self.scene_root}")
        gt = json.loads((self.scene_root / "scene_gt.json").read_text(encoding="utf-8"))
        gt_info = json.loads((self.scene_root / "scene_gt_info.json").read_text(encoding="utf-8"))
        model_info = json.loads((root / "models" / "models_info.json").read_text(encoding="utf-8"))
        samples: list[PoseSample] = []
        for frame_id, objects in gt.items():
            infos = gt_info[frame_id]
            if len(objects) != len(infos):
                raise ValueError(f"scene GT/info instance count mismatch for frame {frame_id}")
            image_path = self.scene_root / "rgb" / f"{int(frame_id):06d}.png"
            for object_gt, info in zip(objects, infos):
                bbox = info["bbox_visib"]
                if bbox[2] <= 0 or bbox[3] <= 0:
                    continue
                object_id = int(object_gt["obj_id"])
                details = model_info[str(object_id)]
                samples.append(PoseSample(
                    image_path=image_path,
                    frame_id=str(int(frame_id)),
                    object_id=object_id,
                    bbox_xywh=tuple(float(value) for value in bbox),
                    rotation=np.asarray(object_gt["cam_R_m2c"], dtype=np.float32).reshape(3, 3),
                    translation_m=np.asarray(object_gt["cam_t_m2c"], dtype=np.float32).reshape(3) * 0.001,
                    extent_m=np.asarray([details["size_x"], details["size_y"], details["size_z"]], dtype=np.float32) * 0.001,
                ))
        if not samples:
            raise ValueError(f"no visible BOP instances in {self.scene_root}")
        self.samples = samples
        # The compact experiment repeatedly visits the same synthetic ROIs.
        # Retaining uint8 crops removes WSL/Windows filesystem latency from
        # each epoch while keeping the dataset below 100 MiB.
        self.roi_images = [self._read_crop(sample) for sample in samples]

    def __len__(self) -> int:
        return len(self.samples)

    def limit_frames(self, frame_count: int) -> None:
        """Keep complete early BOP frames for a bounded smoke training run."""
        if frame_count <= 0:
            return
        allowed = {str(index) for index in range(frame_count)}
        keep = [index for index, sample in enumerate(self.samples) if sample.frame_id in allowed]
        self.samples = [self.samples[index] for index in keep]
        self.roi_images = [self.roi_images[index] for index in keep]
        if not self.samples:
            raise ValueError(f"no instances remain after limiting to {frame_count} frames")

    def _read_crop(self, sample: PoseSample) -> np.ndarray:
        image = cv2.imread(str(sample.image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(sample.image_path)
        x, y, width, height = sample.bbox_xywh
        center = np.asarray([x + 0.5 * width, y + 0.5 * height], dtype=np.float32)
        scale = max(width, height) * self.pad_scale
        # This is the same affine crop primitive used by the upstream mapper.
        return np.ascontiguousarray(crop_resize_by_warp_affine(image, center, scale, 256, interpolation=cv2.INTER_LINEAR).transpose(2, 0, 1))

    def __getitem__(self, index: int) -> dict[str, Any]:
        sample = self.samples[index]
        return {
            "roi_img": torch.from_numpy(self.roi_images[index].astype(np.float32) / 255.0),
            "rotation": torch.from_numpy(sample.rotation),
            "translation_m": torch.from_numpy(sample.translation_m),
            "extent_m": torch.from_numpy(sample.extent_m),
            "frame_id": sample.frame_id,
            "object_id": sample.object_id,
        }


def _configure_model() -> torch.nn.Module:
    """Build a compact GDRN direct-pose configuration without upstream data IO."""
    config_path = UPSTREAM_ROOT / "configs/gdrn/lm/a6_cPnP_lm13.py"
    cfg = Config.fromfile(str(config_path))
    cfg.MODEL.DEVICE = "cuda"
    cfg.MODEL.WEIGHTS = ""
    cfg.MODEL.CDPN.BACKBONE.NUM_LAYERS = 18
    cfg.MODEL.CDPN.BACKBONE.PRETRAINED = ""
    cfg.MODEL.CDPN.BACKBONE.INPUT_RES = 256
    cfg.MODEL.CDPN.BACKBONE.OUTPUT_RES = 64
    cfg.MODEL.CDPN.ROT_HEAD.NUM_FILTERS = 64
    cfg.MODEL.CDPN.ROT_HEAD.NUM_GN_GROUPS = 16
    cfg.MODEL.CDPN.ROT_HEAD.NUM_LAYERS = 3
    cfg.MODEL.CDPN.ROT_HEAD.NUM_CLASSES = 1
    cfg.MODEL.CDPN.ROT_HEAD.NUM_REGIONS = 2
    # The custom loop trains GDRN's direct learned-PnP outputs. Dense-head
    # supervision is intentionally disabled because this renderer does not yet
    # emit per-pixel surface XYZ maps; it remains in the forward path.
    cfg.MODEL.CDPN.ROT_HEAD.XYZ_LW = 0.0
    cfg.MODEL.CDPN.ROT_HEAD.MASK_LW = 0.0
    cfg.MODEL.CDPN.ROT_HEAD.REGION_LW = 0.0
    cfg.MODEL.CDPN.PNP_NET.PNP_HEAD_CFG = dict(type="ConvPnPNet", norm="GN", num_gn_groups=32, drop_prob=0.0)
    cfg.MODEL.CDPN.PNP_NET.R_ONLY = False
    cfg.MODEL.CDPN.PNP_NET.WITH_2D_COORD = False
    cfg.MODEL.CDPN.PNP_NET.REGION_ATTENTION = False
    cfg.MODEL.CDPN.PNP_NET.MASK_ATTENTION = "none"
    cfg.MODEL.CDPN.PNP_NET.ROT_TYPE = "ego_rot6d"
    cfg.MODEL.CDPN.PNP_NET.TRANS_TYPE = "trans"
    cfg.MODEL.CDPN.PNP_NET.PM_LW = 0.0
    cfg.MODEL.CDPN.PNP_NET.ROT_LOSS_TYPE = "L2"
    cfg.MODEL.CDPN.PNP_NET.ROT_LW = 1.0
    cfg.MODEL.CDPN.PNP_NET.TRANS_LOSS_TYPE = "MSE"
    cfg.MODEL.CDPN.PNP_NET.TRANS_LOSS_DISENTANGLE = False
    cfg.MODEL.CDPN.PNP_NET.TRANS_LW = 5.0
    cfg.MODEL.CDPN.PNP_NET.CENTROID_LW = 0.0
    cfg.MODEL.CDPN.PNP_NET.Z_LW = 0.0
    # build_model_optimizer needs this even though the custom loop uses AdamW.
    cfg.SOLVER.BASE_LR = 1e-3
    cfg.SOLVER.OPTIMIZER_CFG = dict(type="RMSprop", lr=1e-3, momentum=0.0, weight_decay=0.0)
    model, _ = GDRN.build_model_optimizer(cfg)
    return model


def _loss_inputs(batch: dict[str, Any], device: torch.device) -> tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
    image = batch["roi_img"].to(device, non_blocking=True)
    rotation = batch["rotation"].to(device, non_blocking=True)
    translation = batch["translation_m"].to(device, non_blocking=True)
    extent = batch["extent_m"].to(device, non_blocking=True)
    batch_size = image.shape[0]
    zeros_mask = torch.zeros((batch_size, 64, 64), dtype=torch.float32, device=device)
    zeros_xyz = torch.zeros((batch_size, 3, 64, 64), dtype=torch.float32, device=device)
    zeros_ratio = torch.zeros((batch_size, 3), dtype=torch.float32, device=device)
    inputs = dict(
        x=image,
        gt_xyz=zeros_xyz,
        gt_mask_trunc=zeros_mask,
        gt_mask_visib=zeros_mask,
        gt_mask_obj=zeros_mask,
        gt_region=zeros_mask,
        gt_ego_rot=rotation,
        gt_trans=translation,
        gt_trans_ratio=zeros_ratio,
        roi_extents=extent,
        do_loss=True,
    )
    return inputs, {"image": image, "extent": extent}


def _train(model: torch.nn.Module, dataset: BopPoseDataset, epochs: int, batch_size: int, learning_rate: float, seed: int) -> list[dict[str, float]]:
    device = torch.device("cuda")
    generator = torch.Generator().manual_seed(seed)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=0, pin_memory=True, generator=generator)
    optimizer = torch.optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=1e-5)
    history: list[dict[str, float]] = []
    for epoch in range(1, epochs + 1):
        model.train()
        total_loss = 0.0
        for batch in loader:
            inputs, _ = _loss_inputs(batch, device)
            optimizer.zero_grad(set_to_none=True)
            # GDRN's training branch records its native losses through
            # Detectron2 EventStorage; no custom pose tensor is fabricated.
            with EventStorage():
                _, losses = model(**inputs)
            loss = sum(losses.values())
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=10.0)
            optimizer.step()
            total_loss += float(loss.detach().cpu()) * inputs["x"].shape[0]
        mean_loss = total_loss / len(dataset)
        if epoch == 1 or epoch % 10 == 0 or epoch == epochs:
            report = {"epoch": float(epoch), "loss": mean_loss}
            history.append(report)
            print(f"epoch={epoch} loss={mean_loss:.8f}", flush=True)
    return history


def _infer(model: torch.nn.Module, dataset: BopPoseDataset) -> tuple[dict[str, list[dict[str, Any]]], list[float]]:
    device = torch.device("cuda")
    loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=0, pin_memory=True)
    predictions: dict[str, list[dict[str, Any]]] = {}
    runtimes_ms: list[float] = []
    model.eval()
    with torch.no_grad():
        for batch in loader:
            image = batch["roi_img"].to(device, non_blocking=True)
            extent = batch["extent_m"].to(device, non_blocking=True)
            torch.cuda.synchronize()
            start = time.perf_counter()
            output = model(image, roi_extents=extent, do_loss=False)
            torch.cuda.synchronize()
            runtimes_ms.append((time.perf_counter() - start) * 1000.0)
            rotation = output["rot"].detach().cpu().numpy()[0]
            translation = output["trans"].detach().cpu().numpy()[0]
            frame_id = str(int(batch["frame_id"][0]))
            predictions.setdefault(frame_id, []).append({
                "obj_id": int(batch["object_id"][0]),
                "R": rotation.tolist(),
                "t_m": translation.tolist(),
                "runtime_ms": runtimes_ms[-1],
            })
    return predictions, runtimes_ms


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-dataset", required=True)
    parser.add_argument("--test-dataset", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=20260828)
    parser.add_argument("--train-frame-limit", type=int, default=0, help="optional bounded training subset; 0 keeps all frames")
    parser.add_argument("--checkpoint", help="existing custom GDRN checkpoint to load")
    parser.add_argument("--skip-training", action="store_true", help="only run inference from --checkpoint")
    args = parser.parse_args(argv)
    if not torch.cuda.is_available():
        raise RuntimeError("the custom GDRN training run requires CUDA in the dedicated Windows environment")
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    train_set = BopPoseDataset(Path(args.train_dataset), "train_pbr")
    train_set.limit_frames(args.train_frame_limit)
    test_set = BopPoseDataset(Path(args.test_dataset), "test")
    print(f"train_instances={len(train_set)} test_instances={len(test_set)}", flush=True)
    model = _configure_model()
    if args.checkpoint:
        checkpoint = torch.load(args.checkpoint, map_location="cuda")
        model.load_state_dict(checkpoint["model_state_dict"])
    if args.skip_training:
        if not args.checkpoint:
            raise ValueError("--skip-training requires --checkpoint")
        history = [{"loaded_checkpoint": str(args.checkpoint)}]
    else:
        history = _train(model, train_set, args.epochs, args.batch_size, args.learning_rate, args.seed)
    predictions, runtimes_ms = _infer(model, test_set)
    if not args.skip_training:
        torch.save({"model_state_dict": model.state_dict(), "seed": args.seed}, output_dir / "gdrn_synthetic_checkpoint.pth")
    (output_dir / "predictions_camera_to_target.json").write_text(json.dumps(predictions, indent=2) + "\n", encoding="utf-8")
    (output_dir / "training_summary.json").write_text(json.dumps({
        "algorithm_name": "upstream_gdrn_custom_synthetic_direct_pose",
        "upstream_commit": "1be9fe73292fd748087aa88d7bf987434f271ebb",
        "train_instance_count": len(train_set),
        "test_instance_count": len(test_set),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "learning_rate": args.learning_rate,
        "seed": args.seed,
        "train_frame_limit": args.train_frame_limit,
        "checkpoint": args.checkpoint,
        "roi_source": "BOP scene_gt_info bbox_visib (GT ROI condition)",
        "runtime_mean_ms": float(np.mean(runtimes_ms)),
        "runtime_p95_ms": float(np.percentile(runtimes_ms, 95)),
        "history": history,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output_dir / 'predictions_camera_to_target.json'}", flush=True)


if __name__ == "__main__":
    main()
