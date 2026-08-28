#!/usr/bin/env python3
"""Generate upstream-GDRN compatible ROI XYZ/mask supervision from BOP poses.

The input models and poses are the parameterised simulation proxies.  This
script does not alter GDR-Net: it materialises the dense object-coordinate and
visibility labels that its native training loss expects.  Labels are written
per BOP object instance to avoid storing impractically large full-resolution
XYZ images.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def _read_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    lines = path.read_text(encoding="ascii").splitlines()
    vertex_count = int(next(line.split()[2] for line in lines if line.startswith("element vertex ")))
    face_count = int(next(line.split()[2] for line in lines if line.startswith("element face ")))
    header_end = lines.index("end_header") + 1
    vertices = np.asarray([[float(value) for value in line.split()[:3]] for line in lines[header_end:header_end + vertex_count]], dtype=np.float32) * 0.001
    faces = np.asarray([[int(value) for value in line.split()[1:4]] for line in lines[header_end + vertex_count:header_end + vertex_count + face_count]], dtype=np.int32)
    return vertices, faces


def _affine(center: np.ndarray, scale: float, resolution: int) -> np.ndarray:
    """Same no-rotation crop transform used by upstream crop_resize_by_warp_affine."""
    source = np.asarray([
        center,
        center + np.array([0.0, -scale * 0.5], dtype=np.float32),
        center + np.array([-scale * 0.5, 0.0], dtype=np.float32),
    ], dtype=np.float32)
    destination = np.asarray([
        [resolution * 0.5, resolution * 0.5],
        [resolution * 0.5, 0.0],
        [0.0, resolution * 0.5],
    ], dtype=np.float32)
    return cv2.getAffineTransform(source, destination)


def _apply_affine(points: np.ndarray, affine: np.ndarray) -> np.ndarray:
    return points @ affine[:, :2].T + affine[:, 2]


def _rasterize(vertices_m: np.ndarray, faces: np.ndarray, rotation: np.ndarray, translation_m: np.ndarray,
               camera: np.ndarray, extent_m: np.ndarray, affine: np.ndarray, resolution: int) -> tuple[np.ndarray, np.ndarray]:
    """Perspective-correct triangle rasterisation of normalized local XYZ."""
    vertices_camera = (rotation @ vertices_m.T).T + translation_m
    projected_h = vertices_camera @ camera.T
    projected = projected_h[:, :2] / projected_h[:, 2:3]
    projected = _apply_affine(projected, affine)
    xyz = np.zeros((resolution, resolution, 3), dtype=np.float32)
    depth = np.full((resolution, resolution), np.inf, dtype=np.float32)
    for face in faces:
        z = vertices_camera[face, 2]
        if np.any(z <= 1e-5):
            continue
        triangle = projected[face]
        denominator = (triangle[1, 1] - triangle[2, 1]) * (triangle[0, 0] - triangle[2, 0]) + (triangle[2, 0] - triangle[1, 0]) * (triangle[0, 1] - triangle[2, 1])
        if abs(float(denominator)) < 1e-8:
            continue
        low = np.maximum(np.floor(triangle.min(axis=0)).astype(int), 0)
        high = np.minimum(np.ceil(triangle.max(axis=0)).astype(int), resolution - 1)
        if np.any(high < low):
            continue
        xx, yy = np.meshgrid(np.arange(low[0], high[0] + 1, dtype=np.float32), np.arange(low[1], high[1] + 1, dtype=np.float32))
        weight0 = ((triangle[1, 1] - triangle[2, 1]) * (xx - triangle[2, 0]) + (triangle[2, 0] - triangle[1, 0]) * (yy - triangle[2, 1])) / denominator
        weight1 = ((triangle[2, 1] - triangle[0, 1]) * (xx - triangle[2, 0]) + (triangle[0, 0] - triangle[2, 0]) * (yy - triangle[2, 1])) / denominator
        weight2 = 1.0 - weight0 - weight1
        inside = (weight0 >= -1e-5) & (weight1 >= -1e-5) & (weight2 >= -1e-5)
        reciprocal_depth = weight0 / z[0] + weight1 / z[1] + weight2 / z[2]
        triangle_depth = np.where(inside, 1.0 / reciprocal_depth, np.inf)
        current_depth = depth[low[1]:high[1] + 1, low[0]:high[0] + 1]
        update = triangle_depth < current_depth
        if not np.any(update):
            continue
        perspective_weights = np.stack((weight0 / z[0], weight1 / z[1], weight2 / z[2]), axis=-1) / reciprocal_depth[..., None]
        local = perspective_weights @ vertices_m[face]
        normalized = local / extent_m + 0.5
        current_xyz = xyz[low[1]:high[1] + 1, low[0]:high[0] + 1]
        current_xyz[update] = normalized[update]
        current_depth[update] = triangle_depth[update]
    return xyz, np.isfinite(depth)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--resolution", type=int, default=64)
    parser.add_argument("--pad-scale", type=float, default=1.5)
    parser.add_argument("--splits", nargs="+", default=("train_pbr", "val", "test"))
    args = parser.parse_args()
    root = Path(args.dataset)
    models_info = json.loads((root / "models" / "models_info.json").read_text(encoding="utf-8"))
    model_cache = {int(object_id): _read_ply(root / "models" / f"obj_{int(object_id):06d}.ply") for object_id in models_info}
    total, ious = 0, []
    for split in args.splits:
        scene = root / split / "000001"
        if not scene.is_dir():
            continue
        output = scene / "xyz_crop"
        if output.exists() and any(output.iterdir()):
            raise FileExistsError(f"refusing to overwrite geometry labels: {output}")
        output.mkdir(exist_ok=True)
        gt = json.loads((scene / "scene_gt.json").read_text(encoding="utf-8"))
        infos = json.loads((scene / "scene_gt_info.json").read_text(encoding="utf-8"))
        cameras = json.loads((scene / "scene_camera.json").read_text(encoding="utf-8"))
        for frame_id, objects in gt.items():
            camera = np.asarray(cameras[frame_id]["cam_K"], dtype=np.float32).reshape(3, 3)
            for instance_index, (object_gt, info) in enumerate(zip(objects, infos[frame_id])):
                bbox = np.asarray(info["bbox_visib"], dtype=np.float32)
                if bbox[2] <= 0 or bbox[3] <= 0:
                    continue
                object_id = int(object_gt["obj_id"])
                vertices, faces = model_cache[object_id]
                extent = np.asarray([models_info[str(object_id)][name] for name in ("size_x", "size_y", "size_z")], dtype=np.float32) * 0.001
                center = bbox[:2] + bbox[2:] * 0.5
                affine = _affine(center, float(max(bbox[2:]) * args.pad_scale), args.resolution)
                rotation = np.asarray(object_gt["cam_R_m2c"], dtype=np.float32).reshape(3, 3)
                translation = np.asarray(object_gt["cam_t_m2c"], dtype=np.float32) * 0.001
                xyz, rendered_mask = _rasterize(vertices, faces, rotation, translation, camera, extent, affine, args.resolution)
                full_mask = cv2.imread(str(scene / "mask_visib" / f"{int(frame_id):06d}_{instance_index:06d}.png"), cv2.IMREAD_GRAYSCALE)
                if full_mask is None:
                    raise FileNotFoundError(f"missing visible mask for {split}/{frame_id}/{instance_index}")
                source_mask = cv2.warpAffine(full_mask, affine, (args.resolution, args.resolution), flags=cv2.INTER_NEAREST) > 0
                mask = source_mask & rendered_mask
                union = source_mask | rendered_mask
                ious.append(float(np.count_nonzero(mask) / max(1, np.count_nonzero(union))))
                xyz[~mask] = 0.0
                np.savez_compressed(output / f"{int(frame_id):06d}_{instance_index:06d}.npz", xyz=xyz.astype(np.float16), mask=mask.astype(np.uint8))
                total += 1
    manifest = {"format": "gdrn_roi_xyz_mask_v01", "resolution": args.resolution, "pad_scale": args.pad_scale,
                "xyz": "normalized local object coordinates: xyz_m / model_extent_m + 0.5", "label_count": total,
                "raster_vs_visible_mask_iou_mean": float(np.mean(ious)), "raster_vs_visible_mask_iou_min": float(np.min(ious))}
    (root / "geometry_label_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
