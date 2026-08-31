"""Residual definitions for the markerless localization bundle.

Poses use ``T_map_frame``: rotations map local coordinates into ``map`` and
translations are metres in ``map``. Image observations are pixels unless the
field is explicitly named ``normalized_xy``. Every residual is whitened by its
declared one-sigma uncertainty before the chain weight is applied.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
from scipy.spatial.transform import Rotation


class StateView(Protocol):
    def target_pose(self, target_id: int) -> tuple[np.ndarray, np.ndarray]: ...
    def camera_pose(self, camera_id: int) -> tuple[np.ndarray, np.ndarray]: ...
    def segment_alignment(self) -> tuple[float, np.ndarray]: ...


def _sigma(value: float) -> float:
    if value <= 0.0:
        raise ValueError("factor sigma must be positive")
    return float(value)


def _project_pixel(point_camera: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    if point_camera[2] <= 1e-6:
        # A finite large residual lets TRF recover instead of producing NaNs.
        return np.array([1e4, 1e4], dtype=float)
    homogeneous = camera_matrix @ point_camera
    return homogeneous[:2] / homogeneous[2]


def _point_in_camera(
    point_local: np.ndarray,
    target_pose: tuple[np.ndarray, np.ndarray],
    camera_pose: tuple[np.ndarray, np.ndarray],
) -> np.ndarray:
    rotation_target, translation_target = target_pose
    rotation_camera, translation_camera = camera_pose
    point_map = rotation_target @ point_local + translation_target
    return rotation_camera.T @ (point_map - translation_camera)


def _candidate_points(point_local: np.ndarray, symmetries: tuple[np.ndarray, ...]) -> list[np.ndarray]:
    if not symmetries:
        return [point_local]
    return [rotation @ point_local for rotation in symmetries]


@dataclass(frozen=True)
class BearingFactor:
    """F1: target 3-D point observed as a normalized image coordinate."""

    target_id: int
    camera_id: int
    point_target_m: np.ndarray
    normalized_xy: np.ndarray
    sigma_normalized: float
    symmetries: tuple[np.ndarray, ...] = field(default_factory=tuple)
    kind: str = field(default="F1", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        observed = np.asarray(self.normalized_xy, dtype=float)
        candidates = []
        for point in _candidate_points(np.asarray(self.point_target_m, dtype=float), self.symmetries):
            camera_point = _point_in_camera(point, state.target_pose(self.target_id), state.camera_pose(self.camera_id))
            predicted = camera_point[:2] / max(camera_point[2], 1e-6)
            candidates.append((predicted - observed) / _sigma(self.sigma_normalized))
        return min(candidates, key=lambda value: float(value @ value))

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("target", self.target_id), ("camera", self.camera_id))


@dataclass(frozen=True)
class ContourFactor:
    """F2: one-dimensional correspondence-line distance in pixels."""

    target_id: int
    camera_id: int
    point_target_m: np.ndarray
    observed_pixel_xy: np.ndarray
    normal_xy: np.ndarray
    camera_matrix: np.ndarray
    sigma_px: float
    symmetries: tuple[np.ndarray, ...] = field(default_factory=tuple)
    kind: str = field(default="F2", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        normal = np.asarray(self.normal_xy, dtype=float)
        normal /= max(float(np.linalg.norm(normal)), 1e-12)
        observed = np.asarray(self.observed_pixel_xy, dtype=float)
        candidates = []
        for point in _candidate_points(np.asarray(self.point_target_m, dtype=float), self.symmetries):
            camera_point = _point_in_camera(point, state.target_pose(self.target_id), state.camera_pose(self.camera_id))
            pixel = _project_pixel(camera_point, np.asarray(self.camera_matrix, dtype=float))
            candidates.append(np.array([normal @ (pixel - observed) / _sigma(self.sigma_px)]))
        return min(candidates, key=lambda value: abs(float(value[0])))

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("target", self.target_id), ("camera", self.camera_id))


@dataclass(frozen=True)
class SupportPlaneFactor:
    """F3: object base lies on its catalog support elevation."""

    target_id: int
    half_height_m: float
    support_z_m: float
    base_offset_m: float = 0.0
    sigma_m: float = 0.003
    kind: str = field(default="F3", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        rotation, translation = state.target_pose(self.target_id)
        base_point = translation + rotation @ np.array([0.0, 0.0, -self.half_height_m])
        expected = self.support_z_m + self.base_offset_m
        return np.array([(base_point[2] - expected) / _sigma(self.sigma_m)])

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("target", self.target_id),)


@dataclass(frozen=True)
class ControlPointFactor:
    """F4: surveyed map control point reprojection into a camera."""

    camera_id: int
    point_map_m: np.ndarray
    observed_pixel_xy: np.ndarray
    camera_matrix: np.ndarray
    sigma_px: float = 1.0
    kind: str = field(default="F4", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        rotation, translation = state.camera_pose(self.camera_id)
        point_camera = rotation.T @ (np.asarray(self.point_map_m, dtype=float) - translation)
        pixel = _project_pixel(point_camera, np.asarray(self.camera_matrix, dtype=float))
        return (pixel - np.asarray(self.observed_pixel_xy, dtype=float)) / _sigma(self.sigma_px)

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("camera", self.camera_id),)


@dataclass(frozen=True)
class RelativePoseFactor:
    """F5: measured ``T_camera_i_camera_j`` relative-pose prior."""

    camera_i: int
    camera_j: int
    rotation_i_j: np.ndarray
    translation_i_j_m: np.ndarray
    sigma_rotation_rad: float = 0.02
    sigma_translation_m: float = 0.01
    kind: str = field(default="F5", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        rotation_i, translation_i = state.camera_pose(self.camera_i)
        rotation_j, translation_j = state.camera_pose(self.camera_j)
        predicted_rotation = rotation_i.T @ rotation_j
        predicted_translation = rotation_i.T @ (translation_j - translation_i)
        measured_rotation = np.asarray(self.rotation_i_j, dtype=float)
        rotation_error = Rotation.from_matrix(measured_rotation.T @ predicted_rotation).as_rotvec()
        translation_error = predicted_translation - np.asarray(self.translation_i_j_m, dtype=float)
        return np.r_[rotation_error / _sigma(self.sigma_rotation_rad), translation_error / _sigma(self.sigma_translation_m)]

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("camera", self.camera_i), ("camera", self.camera_j))


@dataclass(frozen=True)
class SegmentUwbFactor:
    """F6: segment-level scale and map gauge from SLAM to UWB positions."""

    slam_position: np.ndarray
    uwb_position_m: np.ndarray
    sigma_m: float = 0.04
    kind: str = field(default="F6", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        scale, bias = state.segment_alignment()
        predicted = scale * np.asarray(self.slam_position, dtype=float) + bias
        return (predicted - np.asarray(self.uwb_position_m, dtype=float)) / _sigma(self.sigma_m)

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("segment", None),)


@dataclass(frozen=True)
class PlatformPoseFactor:
    """F7: per-frame platform pose prior with explicit uncertainty."""

    camera_id: int
    rotation_map_camera: np.ndarray
    translation_map_camera_m: np.ndarray
    sigma_rotation_rad: float = 0.05
    sigma_translation_m: float = 0.05
    kind: str = field(default="F7", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        rotation, translation = state.camera_pose(self.camera_id)
        rotation_error = Rotation.from_matrix(np.asarray(self.rotation_map_camera).T @ rotation).as_rotvec()
        translation_error = translation - np.asarray(self.translation_map_camera_m, dtype=float)
        return np.r_[rotation_error / _sigma(self.sigma_rotation_rad), translation_error / _sigma(self.sigma_translation_m)]

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("camera", self.camera_id),)


@dataclass(frozen=True)
class CatalogPriorFactor:
    """F8: weak catalog position and upright-orientation regularizer."""

    target_id: int
    translation_map_target_m: np.ndarray
    yaw_rad: float
    sigma_position_m: float = 0.25
    sigma_upright_rad: float = 0.08
    sigma_yaw_rad: float = 0.5
    kind: str = field(default="F8", init=False)

    def residual(self, state: StateView) -> np.ndarray:
        rotation, translation = state.target_pose(self.target_id)
        rotvec = Rotation.from_matrix(rotation).as_rotvec()
        yaw = Rotation.from_matrix(rotation).as_euler("xyz")[2]
        yaw_error = np.arctan2(np.sin(yaw - self.yaw_rad), np.cos(yaw - self.yaw_rad))
        return np.r_[
            (translation - np.asarray(self.translation_map_target_m, dtype=float)) / _sigma(self.sigma_position_m),
            rotvec[:2] / _sigma(self.sigma_upright_rad),
            yaw_error / _sigma(self.sigma_yaw_rad),
        ]

    def dependencies(self) -> tuple[tuple[str, int | None], ...]:
        return (("target", self.target_id),)


Factor = BearingFactor | ContourFactor | SupportPlaneFactor | ControlPointFactor | RelativePoseFactor | SegmentUwbFactor | PlatformPoseFactor | CatalogPriorFactor


def normalized_from_pixel(pixel_xy: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    """Convert a pixel observation to the normalized pinhole image plane."""
    inverse = np.linalg.inv(np.asarray(camera_matrix, dtype=float))
    ray = inverse @ np.r_[np.asarray(pixel_xy, dtype=float), 1.0]
    return ray[:2] / ray[2]
