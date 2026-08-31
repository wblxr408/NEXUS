"""Sparse correspondence-line extraction inspired by SRT3D/ICG."""

from __future__ import annotations

import cv2
import numpy as np

from .factors import ContourFactor


def mask_contour_samples(mask: np.ndarray, maximum_points: int = 96) -> tuple[np.ndarray, np.ndarray]:
    """Return contour pixels and outward unit normals from a binary mask."""
    binary = (np.asarray(mask) > 0).astype(np.uint8)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return np.empty((0, 2)), np.empty((0, 2))
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(float)
    step = max(1, int(np.ceil(len(contour) / maximum_points)))
    contour = contour[::step]
    previous, following = np.roll(contour, 1, axis=0), np.roll(contour, -1, axis=0)
    tangent = following - previous
    normals = np.column_stack((tangent[:, 1], -tangent[:, 0]))
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    return contour, normals


def search_correspondence_lines(gray: np.ndarray, predicted_pixels: np.ndarray, normals: np.ndarray,
                                radius_px: int = 8, minimum_gradient: float = 8.0) -> tuple[np.ndarray, np.ndarray]:
    """Search the strongest 1-D image gradient along each contour normal."""
    image = np.asarray(gray)
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gradient_x = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3)
    found, keep = [], []
    height, width = image.shape
    offsets = np.arange(-radius_px, radius_px + 1)
    for index, (pixel, normal) in enumerate(zip(predicted_pixels, normals)):
        candidates = pixel[None, :] + offsets[:, None] * normal[None, :]
        rounded = np.rint(candidates).astype(int)
        valid = (rounded[:, 0] >= 1) & (rounded[:, 0] < width - 1) & (rounded[:, 1] >= 1) & (rounded[:, 1] < height - 1)
        if not np.any(valid):
            continue
        valid_points = rounded[valid]
        response = np.abs(gradient_x[valid_points[:, 1], valid_points[:, 0]] * normal[0] + gradient_y[valid_points[:, 1], valid_points[:, 0]] * normal[1])
        best = int(np.argmax(response))
        if response[best] >= minimum_gradient:
            found.append(valid_points[best].astype(float)); keep.append(index)
    return np.asarray(found, dtype=float).reshape(-1, 2), np.asarray(keep, dtype=int)


def contour_factors(*, target_id: int, camera_id: int, points_target_m: np.ndarray,
                    predicted_pixels: np.ndarray, normals: np.ndarray, gray: np.ndarray,
                    camera_matrix: np.ndarray, sigma_px: float = 1.5,
                    symmetries: tuple[np.ndarray, ...] = ()) -> list[ContourFactor]:
    observed, keep = search_correspondence_lines(gray, predicted_pixels, normals)
    return [ContourFactor(target_id, camera_id, np.asarray(points_target_m)[index], observed[position],
                          np.asarray(normals)[index], camera_matrix, sigma_px, symmetries)
            for position, index in enumerate(keep)]
