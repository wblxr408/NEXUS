#!/usr/bin/env python3
"""Degradation and stress injection for chain B/C robustness evaluation.

Implements the injection column of the optimization plan table 8.1 and the
scenario rows of table 10.4.  Every injector is a pure function over an image
(or a range vector) plus a ROI/mask and explicit parameters; it returns the new
data and one metadata dict that records the *measured* injected amount, never
the requested nominal amount.  Images may be RGB or BGR: channel order is
preserved and only the fill palettes are colour-ordered.

Scope: this generates degraded evaluation inputs for our own simulated dataset.
It is not an attack tool.  ``printed_patch`` prints a fixed pattern and
``roi_shift`` reproduces the *downstream consequence* of a shifted
segmentation; neither optimizes anything against a network.  See the notes on
those two functions for the references that plan 8.1 cites as design input
only.  The backdoor row of table 8.1 (``arXiv 2512.19058``) stays a threat
analysis and is deliberately not implemented.

Outputs carry ``not_ground_truth: true``: degraded frames are evaluation
inputs, never truth.
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path
from typing import Any

import cv2
import numpy as np

CLUTTER_PALETTE = ((16, 16, 16), (244, 244, 244), (14, 220, 240), (235, 24, 190), (250, 236, 20))

# Scenario rows of plan 10.4.  ``applies_to`` says which dataset stream the row
# perturbs; the CLI copies the untouched streams so the output stays evaluable.
DEGRADATION_SCENARIOS: dict[str, dict[str, Any]] = {
    "no_degradation": {"injector": "none", "applies_to": "none", "plan_row": "no_degradation", "params": {}},
    "occlusion_30": {"injector": "occlusion", "applies_to": "rgb", "plan_row": "occlusion_30_50_70", "params": {"fraction": 0.30, "kind": "rectangle"}},
    "occlusion_50": {"injector": "occlusion", "applies_to": "rgb", "plan_row": "occlusion_30_50_70", "params": {"fraction": 0.50, "kind": "rectangle"}},
    "occlusion_70": {"injector": "occlusion", "applies_to": "rgb", "plan_row": "occlusion_30_50_70", "params": {"fraction": 0.70, "kind": "polygon"}},
    "exposure_over": {"injector": "exposure", "applies_to": "rgb", "plan_row": "over_under_hard_shadow", "params": {"mode": "over", "gamma": 0.50, "gain": 1.75}},
    "exposure_under": {"injector": "exposure", "applies_to": "rgb", "plan_row": "over_under_hard_shadow", "params": {"mode": "under", "gamma": 2.00, "gain": 0.32}},
    "hard_shadow": {"injector": "exposure", "applies_to": "rgb", "plan_row": "over_under_hard_shadow", "params": {"mode": "hard_shadow", "shadow_strength": 0.72, "shadow_direction_deg": 35.0}},
    "motion_blur": {"injector": "motion_blur", "applies_to": "rgb", "plan_row": "motion_blur", "params": {"kernel_length_px": 25}},
    "background_clutter": {"injector": "background_clutter", "applies_to": "rgb", "plan_row": "background_clutter", "params": {"count": 18, "size_px": [40, 140]}},
    "uwb_nlos_positive_bias": {"injector": "uwb_nlos", "applies_to": "uwb_ranges", "plan_row": "uwb_nlos_positive_bias", "params": {"anchor_indices": [0, 2], "bias_m": 0.35}},
    "printed_patch": {"injector": "printed_patch", "applies_to": "rgb", "plan_row": "printed_patch", "params": {"seed": 20260901, "blocks": 8, "pattern": "checker", "coverage": 0.35}},
    "roi_shift": {"injector": "roi_shift", "applies_to": "mask", "plan_row": "roi_shift", "params": {"shift_xy_px": [24, -18]}},
}

EXPOSURE_DEFAULTS = {"over": (0.55, 1.60), "under": (1.90, 0.35), "hard_shadow": (1.00, 1.00)}


def _roi(mask: Any) -> np.ndarray:
    roi = np.asarray(mask) > 0
    if roi.ndim != 2 or not roi.any():
        raise ValueError("injector needs a non-empty 2-D ROI mask")
    return roi


def _bbox(roi: np.ndarray) -> tuple[int, int, int, int]:
    ys, xs = np.nonzero(roi)
    return int(ys.min()), int(ys.max()), int(xs.min()), int(xs.max())


def _occluder_rectangle(roi: np.ndarray, fraction: float, rng: np.random.Generator) -> np.ndarray:
    y0, y1, x0, x1 = _bbox(roi)
    side = int(rng.integers(0, 4))
    counts = roi[y0:y1 + 1, x0:x1 + 1].sum(axis=1 if side < 2 else 0).astype(float)
    if side in (1, 3):
        counts = counts[::-1]
    depth = min(int(np.searchsorted(np.cumsum(counts), fraction * roi.sum()) + 1), len(counts))
    occluder = np.zeros(roi.shape, dtype=bool)
    if side == 0:
        occluder[y0:y0 + depth, x0:x1 + 1] = True
    elif side == 1:
        occluder[y1 - depth + 1:y1 + 1, x0:x1 + 1] = True
    elif side == 2:
        occluder[y0:y1 + 1, x0:x0 + depth] = True
    else:
        occluder[y0:y1 + 1, x1 - depth + 1:x1 + 1] = True
    return occluder


def _occluder_polygon(roi: np.ndarray, fraction: float, rng: np.random.Generator, vertices: int = 5) -> np.ndarray:
    ys, xs = np.nonzero(roi)
    center = np.array([xs.mean(), ys.mean()])
    angles = np.sort(rng.uniform(0.0, 2.0 * np.pi, vertices))
    radii = rng.uniform(0.65, 1.0, vertices)
    y0, y1, x0, x1 = _bbox(roi)
    high, low, target = float(np.hypot(x1 - x0 + 1, y1 - y0 + 1)), 0.0, fraction * roi.sum()
    chosen, filled = None, np.zeros(roi.shape, dtype=bool)
    for _ in range(16):
        scale = 0.5 * (low + high)
        polygon = np.rint(center + scale * radii[:, None] * np.column_stack((np.cos(angles), np.sin(angles)))).astype(np.int32)
        canvas = np.zeros(roi.shape, dtype=np.uint8)
        cv2.fillPoly(canvas, [polygon], 1)
        filled = canvas.astype(bool)
        if (roi & filled).sum() >= target:
            chosen, high = filled, scale
        else:
            low = scale
    return chosen if chosen is not None else filled


def occlusion(image: np.ndarray, mask: Any, fraction: float, *, kind: str = "rectangle",
              rng: np.random.Generator | None = None, fill: tuple[int, int, int] = (16, 16, 16)) -> tuple[np.ndarray, dict[str, Any]]:
    """Occlude part of the ROI with a rectangle or polygon (plan 8.1, 30/50/70%).

    ``fraction`` is the requested ROI coverage; ``actual_fraction`` in the
    metadata is measured from the mask intersection after drawing.
    """
    rng = np.random.default_rng() if rng is None else rng
    roi = _roi(mask)
    occluder = _occluder_rectangle(roi, fraction, rng) if kind == "rectangle" else _occluder_polygon(roi, fraction, rng)
    covered = int((roi & occluder).sum())
    result = image.copy()
    result[occluder] = np.asarray(fill, dtype=image.dtype)
    return result, {"injector": "occlusion", "kind": kind, "target_fraction": float(fraction),
                    "actual_fraction": covered / float(roi.sum()), "roi_px": int(roi.sum()),
                    "occluded_roi_px": covered, "occluder_px": int(occluder.sum()), "fill": list(fill)}


def exposure(image: np.ndarray, mask: Any = None, *, mode: str = "over", gamma: float | None = None, gain: float | None = None,
             shadow_strength: float = 0.72, shadow_direction_deg: float = 35.0) -> tuple[np.ndarray, dict[str, Any]]:
    """Over-exposure, under-exposure or a hard directional shadow (plan 8.1).

    ``over``/``under`` use gamma plus gain; ``hard_shadow`` adds a directional
    linear brightness ramp.  Clipping is measured, not assumed.
    """
    if mode not in EXPOSURE_DEFAULTS:
        raise ValueError(f"unsupported exposure mode: {mode}")
    default_gamma, default_gain = EXPOSURE_DEFAULTS[mode]
    gamma_used = float(default_gamma if gamma is None else gamma)
    gain_used = float(default_gain if gain is None else gain)
    values = np.asarray(image, dtype=np.float32) / 255.0
    scaled = gain_used * np.power(values, gamma_used)
    metadata: dict[str, Any] = {"injector": "exposure", "mode": mode, "gamma": gamma_used, "gain": gain_used}
    if mode == "hard_shadow":
        height, width = image.shape[:2]
        radians = np.radians(shadow_direction_deg)
        grid_y, grid_x = np.mgrid[0:height, 0:width]
        projection = grid_x * np.cos(radians) + grid_y * np.sin(radians)
        ramp = (projection - projection.min()) / max(float(projection.max() - projection.min()), 1e-9)
        scaled = scaled * (1.0 - float(shadow_strength) * ramp)[:, :, None]
        metadata |= {"shadow_strength": float(shadow_strength), "shadow_direction_deg": float(shadow_direction_deg)}
    result = np.clip(scaled * 255.0, 0.0, 255.0).astype(np.uint8)
    metadata |= {"saturated_fraction": float(np.mean(scaled >= 1.0)), "crushed_fraction": float(np.mean(scaled <= 2.0 / 255.0)),
                 "mean_level_before": float(values.mean() * 255.0), "mean_level_after": float(result.mean())}
    if mask is not None:
        roi = _roi(mask)
        metadata |= {"roi_mean_level_before": float(image[roi].mean()), "roi_mean_level_after": float(result[roi].mean())}
    return result, metadata


def image_motion_direction_deg(position_a_map_m: Any, position_b_map_m: Any, rotation_camera_from_map: Any) -> float:
    """Image-plane direction of apparent target motion between two camera positions.

    ``rotation_camera_from_map`` maps ``map`` vectors into the camera frame:
    that is the matrix stored per frame as ``R_map_camera_<r><c>`` in
    ``ground_truth/camera_pose_map.csv`` (the header name follows the dataset,
    the content is ``R_camera_map``).  A static target moves on the image plane
    opposite to the camera translation, so the returned angle is measured on
    ``-delta`` in pixel axes (x right, y down), in degrees.
    """
    delta = np.asarray(rotation_camera_from_map, dtype=float) @ (np.asarray(position_b_map_m, dtype=float) - np.asarray(position_a_map_m, dtype=float))
    return float(np.degrees(np.arctan2(-delta[1], -delta[0])))


def motion_blur(image: np.ndarray, mask: Any = None, *, kernel_length_px: int = 25, direction_deg: float = 0.0) -> tuple[np.ndarray, dict[str, Any]]:
    """Convolve with a linear kernel along the trajectory direction (plan 8.1).

    Use :func:`image_motion_direction_deg` to derive ``direction_deg`` from two
    neighbouring camera positions instead of guessing it.
    """
    length = max(int(kernel_length_px), 1)
    kernel = np.zeros((length, length), dtype=np.float32)
    center = (length - 1) / 2.0
    radians = np.radians(direction_deg)
    half = np.array([np.cos(radians), np.sin(radians)]) * center
    cv2.line(kernel, tuple(np.rint([center - half[0], center - half[1]]).astype(int)),
             tuple(np.rint([center + half[0], center + half[1]]).astype(int)), 1.0, 1)
    kernel /= float(kernel.sum())
    result = cv2.filter2D(image, -1, kernel, borderType=cv2.BORDER_REPLICATE)
    metadata = {"injector": "motion_blur", "kernel_length_px": length, "direction_deg": float(direction_deg),
                "kernel_nonzero_px": int(np.count_nonzero(kernel))}
    if mask is not None:
        roi = _roi(mask)
        before = float(np.abs(cv2.Laplacian(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.CV_32F))[roi].mean())
        after = float(np.abs(cv2.Laplacian(cv2.cvtColor(result, cv2.COLOR_BGR2GRAY), cv2.CV_32F))[roi].mean())
        metadata |= {"roi_sharpness_ratio": after / max(before, 1e-9)}
    return result, metadata


def background_clutter(image: np.ndarray, mask: Any, *, count: int = 18, size_px: tuple[int, int] = (40, 140),
                       rng: np.random.Generator | None = None) -> tuple[np.ndarray, dict[str, Any]]:
    """Paste high-contrast distractor blocks outside the ROI (plan 8.1).

    ROI pixels are restored after drawing, so the target appearance is
    untouched and only the background statistics change.
    """
    rng = np.random.default_rng() if rng is None else rng
    roi = _roi(mask)
    height, width = image.shape[:2]
    canvas, drawn, patches, rejected = image.copy(), np.zeros(roi.shape, dtype=np.uint8), [], 0
    low, high = int(size_px[0]), int(size_px[1])
    for _ in range(int(count)):
        for attempt in range(8):
            center = np.array([int(rng.integers(0, width)), int(rng.integers(0, height))])
            if not roi[center[1], center[0]]:
                break
            rejected += 1
        size = np.array([int(rng.integers(low, high + 1)), int(rng.integers(low, high + 1))])
        color = tuple(int(value) for value in CLUTTER_PALETTE[int(rng.integers(0, len(CLUTTER_PALETTE)))])
        kind = "rectangle" if rng.random() < 0.5 else "polygon"
        if kind == "rectangle":
            corner_a, corner_b = center - size // 2, center + size // 2
            cv2.rectangle(canvas, tuple(corner_a), tuple(corner_b), color, -1)
            cv2.rectangle(drawn, tuple(corner_a), tuple(corner_b), 1, -1)
            polygon = None
        else:
            angles = np.sort(rng.uniform(0.0, 2.0 * np.pi, int(rng.integers(3, 7))))
            radii = rng.uniform(0.5, 1.0, len(angles)) * size.mean() / 2.0
            polygon = np.rint(center + radii[:, None] * np.column_stack((np.cos(angles), np.sin(angles)))).astype(np.int32)
            cv2.fillPoly(canvas, [polygon], color)
            cv2.fillPoly(drawn, [polygon], 1)
        patches.append({"kind": kind, "center_px": center.tolist(), "size_px": size.tolist(), "color": list(color),
                        **({"polygon_px": polygon.tolist()} if polygon is not None else {})})
    result = np.where(roi[:, :, None], image, canvas)
    return result, {"injector": "background_clutter", "requested_count": int(count), "patch_count": len(patches),
                    "rejected_placements": rejected, "clutter_px_outside_roi": int((drawn.astype(bool) & ~roi).sum()),
                    "roi_px_changed": int(np.count_nonzero(np.any(result != image, axis=2) & roi)), "patches": patches}


def uwb_nlos(ranges_m: Any, anchor_indices: Any, bias_m: Any, *, jitter_m: float = 0.0,
             rng: np.random.Generator | None = None) -> tuple[np.ndarray, dict[str, Any]]:
    """Add a positive range bias to selected anchors (plan 8.1 UWB NLOS row).

    NLOS lengthens the measured path, so the injected offset is strictly
    positive; this is not zero-mean noise.  Operates on ranges in metres, not
    on images.
    """
    ranges = np.asarray(ranges_m, dtype=float).copy()
    indices = np.asarray(anchor_indices, dtype=int)
    if indices.size and (indices.min() < 0 or indices.max() >= ranges.size):
        raise ValueError("anchor_indices outside the range vector")
    bias = np.broadcast_to(np.asarray(bias_m, dtype=float), indices.shape).astype(float).copy()
    if np.any(bias <= 0.0):
        raise ValueError("uwb_nlos bias must be positive")
    if jitter_m > 0.0:
        rng = np.random.default_rng() if rng is None else rng
        bias = bias + np.abs(rng.normal(0.0, jitter_m, indices.shape))
    ranges[indices] += bias
    return ranges, {"injector": "uwb_nlos", "anchor_indices": indices.tolist(), "applied_bias_m": bias.tolist(),
                    "mean_bias_m": float(bias.mean()) if bias.size else 0.0, "jitter_m": float(jitter_m),
                    "affected_anchor_count": int(indices.size), "anchor_count": int(ranges.size)}


def _pattern(seed: int, blocks: int, pattern: str) -> np.ndarray:
    if pattern == "checker":
        grid = np.indices((blocks, blocks)).sum(axis=0) % 2
    elif pattern == "pseudo_random":
        grid = np.random.default_rng(int(seed)).integers(0, 2, size=(blocks, blocks))
    else:
        raise ValueError(f"unsupported printed pattern: {pattern}")
    tile = np.where(grid[:, :, None] > 0, np.array([250, 250, 250], dtype=np.uint8), np.array([8, 8, 8], dtype=np.uint8))
    return cv2.resize(tile.astype(np.uint8), (blocks * 32, blocks * 32), interpolation=cv2.INTER_NEAREST)


def printed_patch(image: np.ndarray, mask: Any, *, seed: int = 20260901, blocks: int = 8, pattern: str = "checker",
                  coverage: float = 0.35, jitter: float = 0.12, rng: np.random.Generator | None = None) -> tuple[np.ndarray, dict[str, Any]]:
    """Warp a fixed high-contrast printed pattern onto the target surface.

    This is a static-pattern robustness stress test: the pattern is fully
    determined by ``seed``/``blocks``/``pattern`` and nothing is optimized.  No
    gradient, loss or search over the pattern is implemented, and none should
    be added here.  ``arXiv 2108.07229``, cited by plan 8.1, is used as a
    REFERENCE for *why* patch placement on the object surface is a meaningful
    pose-sensitivity probe; its attack algorithm is not reproduced.  Pixels are
    written only inside the ROI, i.e. on the object itself.  ``coverage`` is the
    requested share of the ROI *bounding box* area; the achieved share of ROI
    pixels is measured and reported separately.
    """
    rng = np.random.default_rng(int(seed)) if rng is None else rng
    roi = _roi(mask)
    y0, y1, x0, x1 = _bbox(roi)
    side = np.sqrt(max(float(coverage), 1e-6))
    half = np.array([(x1 - x0 + 1) * side / 2.0, (y1 - y0 + 1) * side / 2.0])
    ys, xs = np.nonzero(roi)
    center = np.array([xs.mean(), ys.mean()])
    corners = center + half * np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    corners = corners + rng.uniform(-jitter, jitter, corners.shape) * half
    texture = _pattern(seed, int(blocks), pattern)
    source = np.array([[0.0, 0.0], [texture.shape[1] - 1.0, 0.0], [texture.shape[1] - 1.0, texture.shape[0] - 1.0], [0.0, texture.shape[0] - 1.0]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, corners.astype(np.float32))
    size = (image.shape[1], image.shape[0])
    warped = cv2.warpPerspective(texture, homography, size, flags=cv2.INTER_NEAREST)
    warped_mask = cv2.warpPerspective(np.ones(texture.shape[:2], dtype=np.uint8), homography, size, flags=cv2.INTER_NEAREST) > 0
    applied = warped_mask & roi
    result = image.copy()
    result[applied] = warped[applied]
    return result, {"injector": "printed_patch", "pattern": pattern, "pattern_seed": int(seed), "blocks": int(blocks),
                    "target_bbox_coverage": float(coverage), "actual_roi_coverage": float(applied.sum() / roi.sum()),
                    "patch_quad_px": corners.round(2).tolist(), "roi_px": int(roi.sum()), "patched_px": int(applied.sum()),
                    "optimization": "none; fixed pattern, reference arXiv 2108.07229 not reproduced"}


def roi_shift(mask: Any, shift_xy_px: Any) -> tuple[np.ndarray, dict[str, Any]]:
    """Translate the ROI/mask handed to the downstream solver (plan 8.1).

    This is a *consequence* simulation of a shifted segmentation/attention map,
    used to measure how the localization chain reacts when its ROI is wrong.
    The plan explicitly requires not publishing the attack itself, so no
    attention-shift attack (``arXiv 2203.00302``) or backdoor
    (``arXiv 2512.19058``) is implemented: only the shifted ROI is produced.
    """
    original = np.asarray(mask)
    shift = np.asarray(shift_xy_px, dtype=float).reshape(2)
    affine = np.array([[1.0, 0.0, shift[0]], [0.0, 1.0, shift[1]]], dtype=np.float32)
    shifted = cv2.warpAffine(original, affine, (original.shape[1], original.shape[0]), flags=cv2.INTER_NEAREST, borderValue=0)
    before, after = original > 0, shifted > 0
    union = int((before | after).sum())
    return shifted, {"injector": "roi_shift", "shift_x_px": float(shift[0]), "shift_y_px": float(shift[1]),
                     "roi_px": int(before.sum()), "shifted_roi_px": int(after.sum()),
                     "retained_fraction": float((before & after).sum() / max(before.sum(), 1)),
                     "iou": float((before & after).sum() / union) if union else 0.0}


def _camera_rows(dataset: Path, split: str) -> dict[int, dict[str, Any]]:
    path = dataset / "ground_truth" / "camera_pose_map.csv"
    rows: dict[int, dict[str, Any]] = {}
    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["split"] != split:
                continue
            rotation = np.array([float(row[f"R_map_camera_{r}{c}"]) for r in range(3) for c in range(3)]).reshape(3, 3)
            rows[int(row["sequence"])] = {"position_map_m": np.array([float(row["camera_x_m"]), float(row["camera_y_m"]), float(row["camera_z_m"])]),
                                          "rotation_camera_from_map": rotation, "trajectory_id": row["trajectory_id"]}
    return rows


def _blur_direction_deg(cameras: dict[int, dict[str, Any]], sequence: int) -> float:
    current = cameras[sequence]
    neighbours = [s for s in (sequence + 1, sequence - 1) if s in cameras and cameras[s]["trajectory_id"] == current["trajectory_id"]]
    if not neighbours:
        return 0.0
    other = cameras[neighbours[0]]
    first, second = (current, other) if neighbours[0] > sequence else (other, current)
    return image_motion_direction_deg(first["position_map_m"], second["position_map_m"], current["rotation_camera_from_map"])


def _mirror_dataset_metadata(dataset: Path, out_root: Path, scene: str) -> None:
    for name in ("camera.yaml", "target_classes.json", "dataset_manifest.json", "split_manifest.json"):
        if (dataset / name).is_file():
            shutil.copy2(dataset / name, out_root / name)
    for name in ("models", "calibration", "ground_truth"):
        if (dataset / name).is_dir():
            shutil.copytree(dataset / name, out_root / name, dirs_exist_ok=True)
    for name in ("scene_gt.json", "scene_gt_info.json", "scene_camera.json", "instances_coco.json"):
        if (dataset / scene / name).is_file():
            shutil.copy2(dataset / scene / name, out_root / scene / name)
    if (dataset / scene / "labels").is_dir():
        shutil.copytree(dataset / scene / "labels", out_root / scene / "labels", dirs_exist_ok=True)


def _apply_frame(scenario: str, image: np.ndarray, masks: list[tuple[str, np.ndarray]], rng: np.random.Generator,
                 blur_direction_deg: float) -> tuple[np.ndarray, list[tuple[str, np.ndarray]], list[dict[str, Any]]]:
    """Apply one scenario to a frame; returns image, masks and per-call metadata."""
    injector, params = DEGRADATION_SCENARIOS[scenario]["injector"], dict(DEGRADATION_SCENARIOS[scenario]["params"])
    records: list[dict[str, Any]] = []
    if injector in {"none", "uwb_nlos"}:
        return image, masks, records
    if injector == "exposure":
        image, metadata = exposure(image, masks[0][1] if masks else None, **params)
        return image, masks, [metadata]
    if injector == "motion_blur":
        image, metadata = motion_blur(image, masks[0][1] if masks else None, direction_deg=blur_direction_deg, **params)
        return image, masks, [metadata]
    if injector == "background_clutter":
        union = np.zeros(image.shape[:2], dtype=bool)
        for _, mask in masks:
            union |= mask > 0
        image, metadata = background_clutter(image, union, rng=rng, size_px=tuple(params.pop("size_px")), **params)
        return image, masks, [metadata]
    if injector == "roi_shift":
        shifted = []
        for name, mask in masks:
            mask_out, metadata = roi_shift(mask, params["shift_xy_px"])
            shifted.append((name, mask_out))
            records.append({"mask": name, **metadata})
        return image, shifted, records
    for name, mask in masks:
        if injector == "occlusion":
            image, metadata = occlusion(image, mask, params["fraction"], kind=params["kind"], rng=rng)
        else:
            image, metadata = printed_patch(image, mask, rng=rng, **params)
        records.append({"mask": name, **metadata})
    return image, masks, records


def inject_split(dataset: Path, out_root: Path, split: str, scenario: str, seed: int) -> dict[str, Any]:
    """Write one degraded copy of ``split`` and return its manifest."""
    if scenario not in DEGRADATION_SCENARIOS:
        raise ValueError(f"unknown scenario: {scenario}; available: {sorted(DEGRADATION_SCENARIOS)}")
    scene = f"{'train_pbr' if split == 'train' else split}/000001"
    source = dataset / scene
    if not (source / "rgb").is_dir():
        raise FileNotFoundError(f"missing rgb directory: {source / 'rgb'}")
    for directory in ("rgb", "mask", "mask_visib", "labels"):
        (out_root / scene / directory).mkdir(parents=True, exist_ok=True)
    _mirror_dataset_metadata(dataset, out_root, scene)
    cameras = _camera_rows(dataset, split)
    definition = DEGRADATION_SCENARIOS[scenario]
    frames = []
    for rgb_path in sorted((source / "rgb").glob("*.png")):
        sequence = int(rgb_path.stem)
        image = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        mask_paths = sorted((source / "mask").glob(f"{rgb_path.stem}_*.png"))
        masks = [(path.name, cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)) for path in mask_paths]
        masks = [(name, mask) for name, mask in masks if np.any(mask > 0)]
        direction = _blur_direction_deg(cameras, sequence) if sequence in cameras else 0.0
        image_out, masks_out, records = _apply_frame(scenario, image, masks, np.random.default_rng([seed, sequence]), direction)
        cv2.imwrite(str(out_root / scene / "rgb" / rgb_path.name), image_out)
        written = dict(masks_out)
        for path in mask_paths:
            mask_out = written.get(path.name, cv2.imread(str(path), cv2.IMREAD_GRAYSCALE))
            cv2.imwrite(str(out_root / scene / "mask" / path.name), mask_out)
            cv2.imwrite(str(out_root / scene / "mask_visib" / path.name), mask_out)
        frames.append({"sequence": sequence, "rgb": f"{scene}/rgb/{rgb_path.name}", "mask_count": len(mask_paths),
                       "blur_direction_deg": direction, "injections": records})
    manifest = {"generator": "simulation/inject_degradations.py", "source_dataset": str(dataset), "split": split, "scene": scene,
                "scenario": scenario, "plan_reference": "docs/优化方案.md 8.1 / 10.4", "injector": definition["injector"],
                "applies_to": definition["applies_to"], "requested_params": definition["params"], "seed": int(seed),
                "frame_count": len(frames), "frames": frames, "not_ground_truth": True,
                "note": "Degraded frames are chain B/C evaluation inputs only; ground truth stays the source dataset truth."}
    if definition["applies_to"] == "uwb_ranges":
        manifest["uwb_range_injection"] = {**definition["params"], "status": "not_applied_here",
                                          "reason": "this dataset carries no anchor ranges; call uwb_nlos() on the UWB range stream with these params"}
    (out_root / "degradation_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, help="root of a generate_target_detection_dataset.py output")
    parser.add_argument("--scenario", default="all", help="scenario name from DEGRADATION_SCENARIOS, or all")
    parser.add_argument("--out", required=True)
    parser.add_argument("--split", default="test")
    parser.add_argument("--seed", type=int, default=20260901)
    args = parser.parse_args()
    dataset, output = Path(args.dataset), Path(args.out)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite existing degradation output: {output}")
    scenarios = list(DEGRADATION_SCENARIOS) if args.scenario == "all" else [args.scenario]
    for scenario in scenarios:
        manifest = inject_split(dataset, output / scenario, args.split, scenario, args.seed)
        print(f"{scenario}: {manifest['frame_count']} frames -> {output / scenario}")


if __name__ == "__main__":
    main()
