"""Small, dependency-light adapter for the upstream GDR-Net outputs.

The upstream project is a complete Detectron2/PyTorch training and evaluation
stack.  This module deliberately keeps the NEXUS boundary small: a caller can
feed the dense object-coordinate and visibility maps produced by GDR-Net and
receive the same 3x4 camera pose used by the rest of the offline analysis
tools.  It is useful before a GPU, BOP dataset, and checkpoint are available,
and does not import PyTorch at module import time.

Coordinates are expected in metres.  GDR-Net checkpoints commonly emit
normalised coordinates; pass ``object_extent_m`` to convert those maps before
solving.  The numerical solver below is an inference fallback (OpenCV PnP),
not a claim that this fallback is differentiable Patch-PnP.  Once the upstream
model is installed, its dense predictions can be passed to exactly the same
entry point.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def _finite_array(value: Any, name: str, ndim: int | None = None) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if ndim is not None and array.ndim != ndim:
        raise ValueError(f"{name} must be a {ndim}-D array")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def _camera_matrix(value: Any) -> np.ndarray:
    matrix = _finite_array(value, "camera_matrix", 2)
    if matrix.shape != (3, 3) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera_matrix must be a finite 3x3 matrix with positive focal lengths")
    return matrix


def _pixel_grid(height: int, width: int, bbox_xyxy: Any | None = None) -> np.ndarray:
    """Return HxWx2 pixel centres, optionally mapping an ROI to image pixels."""
    ys, xs = np.indices((height, width), dtype=np.float64)
    if bbox_xyxy is None:
        return np.stack((xs, ys), axis=-1)
    bbox = _finite_array(bbox_xyxy, "bbox_xyxy").reshape(-1)
    if bbox.size != 4 or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        raise ValueError("bbox_xyxy must be [xmin, ymin, xmax, ymax] with positive extent")
    # Pixel centres in the output ROI map to the corresponding image extent.
    x = bbox[0] + (xs + 0.5) * (bbox[2] - bbox[0]) / width - 0.5
    y = bbox[1] + (ys + 0.5) * (bbox[3] - bbox[1]) / height - 0.5
    return np.stack((x, y), axis=-1)


def estimate_pose_from_dense_correspondences(
    object_coordinate_map: Any,
    visibility_mask: Any,
    camera_matrix: Any,
    *,
    distortion: Any | None = None,
    bbox_xyxy: Any | None = None,
    object_extent_m: Any | None = None,
    max_points: int = 2048,
    confidence_threshold: float = 0.5,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Solve a pose from a GDR-Net dense coordinate map.

    The map may be HxWx3 or 3xHxW, and the visibility map HxW or 1xHxW.  A
    deterministic, confidence-ranked subset keeps the smoke path fast and
    makes repeated experiments reproducible.  ``R`` maps object coordinates
    into the camera frame and ``t`` is in metres.
    """
    coordinates = _finite_array(object_coordinate_map, "object_coordinate_map", 3)
    if coordinates.shape[-1] == 3:
        coordinates = coordinates
    elif coordinates.shape[0] == 3:
        coordinates = coordinates.transpose(1, 2, 0)
    else:
        raise ValueError("object_coordinate_map must have shape HxWx3 or 3xHxW")
    height, width, _ = coordinates.shape

    confidence = _finite_array(visibility_mask, "visibility_mask")
    if confidence.ndim == 3 and confidence.shape[0] == 1:
        confidence = confidence[0]
    if confidence.shape != (height, width):
        raise ValueError("visibility_mask must match the coordinate map spatial shape")
    if np.nanmax(confidence) > 1.0 or np.nanmin(confidence) < 0.0:
        raise ValueError("visibility_mask values must be in [0, 1]")

    if object_extent_m is not None:
        extent = _finite_array(object_extent_m, "object_extent_m").reshape(-1)
        if extent.size == 1:
            extent = np.repeat(extent, 3)
        if extent.size != 3 or np.any(extent <= 0):
            raise ValueError("object_extent_m must contain one or three positive values")
        # GDR-Net's xyz head uses [0, 1] object-normalised coordinates.
        coordinates = (coordinates - 0.5) * extent.reshape(1, 1, 3)

    camera = _camera_matrix(camera_matrix)
    if distortion is None:
        distortion_array = np.zeros(5, dtype=np.float64)
    else:
        distortion_array = _finite_array(distortion, "distortion").reshape(-1)
    if max_points < 4:
        raise ValueError("max_points must be at least four")

    pixels = _pixel_grid(height, width, bbox_xyxy)
    valid = (confidence >= float(confidence_threshold)) & np.all(np.isfinite(coordinates), axis=-1)
    object_points = coordinates[valid]
    image_points = pixels[valid]
    weights = confidence[valid]
    if object_points.shape[0] < 4:
        raise ValueError("at least four visible dense correspondences are required")

    if object_points.shape[0] > max_points:
        # Stable top-confidence sampling; ties retain row-major order.
        keep = np.argsort(-weights, kind="stable")[: int(max_points)]
        object_points, image_points, weights = object_points[keep], image_points[keep], weights[keep]

    use_ransac = object_points.shape[0] >= 6
    if use_ransac:
        ok, rotation, translation, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            camera,
            distortion_array,
            flags=cv2.SOLVEPNP_EPNP,
            reprojectionError=8.0,
            confidence=0.999,
            iterationsCount=100,
        )
    else:
        ok, rotation, translation = cv2.solvePnP(
            object_points, image_points, camera, distortion_array, flags=cv2.SOLVEPNP_ITERATIVE
        )
        inliers = np.arange(object_points.shape[0], dtype=np.int32).reshape(-1, 1)
    if not ok or rotation is None or translation is None:
        raise ValueError("GDR-Net correspondence PnP failed")
    rotation_matrix, _ = cv2.Rodrigues(rotation)
    projected, _ = cv2.projectPoints(object_points, rotation, translation, camera, distortion_array)
    residual = projected.reshape(-1, 2) - image_points
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    inlier_count = int(np.asarray(inliers).size) if inliers is not None else object_points.shape[0]
    diagnostics = {
        "solver": "opencv_solve_pnp_ransac" if use_ransac else "opencv_solve_pnp",
        "correspondence_count": int(object_points.shape[0]),
        "inlier_count": inlier_count,
        "visible_fraction": float(np.mean(valid)),
        "reprojection_error_px": rms,
        "differentiable": False,
    }
    return rotation_matrix.astype(np.float64), translation.reshape(3).astype(np.float64), diagnostics


def run_gdr_net(**inputs: Any) -> AlgorithmResult:
    """Algorithm-router runner for GDR-Net dense predictions.

    ``coordinates``/``visibility`` are aliases accepted for convenient JSON
    and NumPy callers.  Image-to-network inference remains an explicit future
    step because the upstream model requires a checkpoint and its Detectron2
    environment; no fabricated pose is returned when those inputs are absent.
    """
    from algorithms.contracts import AlgorithmResult
    coordinates = inputs.get("object_coordinate_map", inputs.get("coordinates"))
    visibility = inputs.get("visibility_mask", inputs.get("visibility"))
    if coordinates is None or visibility is None:
        raise ValueError(
            "GDR-Net runner requires dense 'coordinates' and 'visibility' predictions; "
            "load an upstream checkpoint separately before calling the adapter"
        )
    rotation, translation, diagnostics = estimate_pose_from_dense_correspondences(
        coordinates,
        visibility,
        inputs.get("camera_matrix"),
        distortion=inputs.get("distortion"),
        bbox_xyxy=inputs.get("bbox_xyxy"),
        object_extent_m=inputs.get("object_extent_m"),
        max_points=int(inputs.get("max_points", 2048)),
        confidence_threshold=float(inputs.get("confidence_threshold", 0.5)),
    )
    pose = np.concatenate((rotation, translation.reshape(3, 1)), axis=1)
    covariance = np.eye(6, dtype=np.float64)
    # The PnP smoke path has no calibrated uncertainty model. Keep it explicit.
    covariance *= float(inputs.get("covariance_diag", 1.0))
    metadata = {
        "upstream": "THU-DA-6D-Pose-Group/GDR-Net",
        "upstream_commit": "1be9fe73292fd748087aa88d7bf987434f271ebb",
        "coordinate_frame": str(inputs.get("frame", "camera")),
        "unit": "m",
        **diagnostics,
    }
    return AlgorithmResult(pose, covariance, "vision.gdr_net", "vision", metadata=metadata)


__all__ = ["estimate_pose_from_dense_correspondences", "run_gdr_net"]
