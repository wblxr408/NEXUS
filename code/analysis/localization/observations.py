"""Convert detector/GDR-Net outputs into geometry factors without using net z."""

from __future__ import annotations

import numpy as np

from .factors import BearingFactor, SupportPlaneFactor, normalized_from_pixel


def dense_correspondence_factors(
    *, target_id: int, camera_id: int, object_coordinates: np.ndarray,
    image_pixels: np.ndarray, confidence: np.ndarray, camera_matrix: np.ndarray,
    extent_m: np.ndarray | None = None, coordinates_are_normalized: bool = False,
    minimum_confidence: float = 0.25, maximum_points: int = 256,
    pixel_sigma: float = 1.0, symmetries: tuple[np.ndarray, ...] = (),
) -> list[BearingFactor]:
    """Create F1 factors from paired 3-D object coordinates and image pixels.

    ``coordinates_are_normalized`` uses the GDR convention ``[-0.5, 0.5]``
    scaled by the per-object metric extent. Network translation/depth is not an
    argument and therefore cannot leak into these observations.
    """
    points = np.asarray(object_coordinates, dtype=float).reshape(-1, 3)
    pixels = np.asarray(image_pixels, dtype=float).reshape(-1, 2)
    scores = np.asarray(confidence, dtype=float).reshape(-1)
    if not (len(points) == len(pixels) == len(scores)):
        raise ValueError("object coordinates, pixels and confidence must have equal length")
    if coordinates_are_normalized:
        if extent_m is None:
            raise ValueError("normalized coordinates require extent_m")
        points = points * np.asarray(extent_m, dtype=float)
    valid = np.all(np.isfinite(points), axis=1) & np.all(np.isfinite(pixels), axis=1) & (scores >= minimum_confidence)
    indices = np.flatnonzero(valid)
    if len(indices) > maximum_points:
        order = np.argsort(scores[indices])[-maximum_points:]
        indices = indices[order]
    focal = float((camera_matrix[0, 0] + camera_matrix[1, 1]) / 2.0)
    return [
        BearingFactor(
            target_id, camera_id, points[index], normalized_from_pixel(pixels[index], camera_matrix),
            sigma_normalized=pixel_sigma / focal / max(np.sqrt(scores[index]), 0.1), symmetries=symmetries,
        )
        for index in indices
    ]


def bbox_bearing_factor(*, target_id: int, camera_id: int, bbox_xywh_px: np.ndarray,
                        camera_matrix: np.ndarray, confidence: float, pixel_sigma: float = 3.0) -> BearingFactor:
    """QuadricSLAM-style center bearing used only as the coarse fallback."""
    x, y, width, height = np.asarray(bbox_xywh_px, dtype=float)
    center = np.array([x + width / 2.0, y + height / 2.0])
    focal = float((camera_matrix[0, 0] + camera_matrix[1, 1]) / 2.0)
    return BearingFactor(target_id, camera_id, np.zeros(3), normalized_from_pixel(center, camera_matrix),
                         pixel_sigma / focal / max(np.sqrt(float(confidence)), 0.1))


def support_factor_from_catalog(target: dict, *, support_z_m: float, unit_scale: float = 0.001,
                                sigma_m: float = 0.003) -> SupportPlaneFactor:
    half_height = float(target["size_mm"][2]) * unit_scale / 2.0
    base_z = float(target["base_z_mm"]) * unit_scale
    return SupportPlaneFactor(int(target["object_id"]), half_height, support_z_m,
                              base_offset_m=base_z - support_z_m, sigma_m=sigma_m)
