"""ROS-agnostic bridge between UVIO/OpenVINS output and NEXUS factors.

The upstream UVIO node can run in the ROS1 hardware workspace or in a ROS2
port.  This module only defines the stable numerical boundary consumed by the
ROS2 target solver: SI IMU samples, map-frame camera poses, F5 relative poses,
and F6 segment UWB alignment factors.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from localization.bundle_solver import PoseState
from localization.factors import RelativePoseFactor, SegmentUwbFactor


def _number(value: Any, name: str) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def scaled_imu_to_si(xacc: Any, yacc: Any, zacc: Any, xgyro: Any, ygyro: Any, zgyro: Any) -> tuple[np.ndarray, np.ndarray]:
    """Convert the vendor ``SCALED_IMU`` payload to m/s² and rad/s.

    This is deliberately separate from any ROS message class so both the UDP
    recorder and a ROS2 subscriber use exactly the same conversion.
    """
    acceleration = np.array([_number(xacc, "xacc"), _number(yacc, "yacc"), _number(zacc, "zacc")]) / 1000.0
    angular_velocity = np.array([_number(xgyro, "xgyro"), _number(ygyro, "ygyro"), _number(zgyro, "zgyro")]) / 1000.0
    return acceleration, angular_velocity


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    acceleration_mps2: np.ndarray
    angular_velocity_rps: np.ndarray

    def __post_init__(self):
        if int(self.timestamp_ns) <= 0:
            raise ValueError("IMU timestamp must be positive")
        for value, name in ((self.acceleration_mps2, "acceleration"), (self.angular_velocity_rps, "angular_velocity")):
            array = np.asarray(value, dtype=float)
            if array.shape != (3,) or not np.all(np.isfinite(array)):
                raise ValueError(f"{name} must be a finite 3-vector")


@dataclass(frozen=True)
class UvioPose:
    timestamp_ns: int
    rotation_map_camera: np.ndarray
    translation_map_camera_m: np.ndarray
    covariance: np.ndarray | None = None

    def __post_init__(self):
        rotation = np.asarray(self.rotation_map_camera, dtype=float)
        translation = np.asarray(self.translation_map_camera_m, dtype=float)
        if int(self.timestamp_ns) <= 0 or rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError("UVIO pose has an invalid timestamp, rotation, or translation")
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            raise ValueError("UVIO pose values must be finite")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) <= 0:
            raise ValueError("UVIO rotation must be a proper orthonormal matrix")


def _pose_from_payload(payload: Mapping[str, Any]) -> UvioPose:
    """Parse common ROS/JSON pose shapes without accepting a hidden ground truth."""
    timestamp = payload.get("timestamp_ns", payload.get("stamp_ns"))
    if timestamp is None:
        raise ValueError("UVIO pose requires timestamp_ns")
    rotation = payload.get("rotation_map_camera")
    translation = payload.get("translation_map_camera_m")
    if rotation is None and "quaternion_xyzw" in payload:
        rotation = Rotation.from_quat(np.asarray(payload["quaternion_xyzw"], dtype=float)).as_matrix()
    if translation is None:
        translation = payload.get("position_m")
    if rotation is None or translation is None:
        raise ValueError("UVIO pose requires map camera rotation and translation")
    return UvioPose(int(timestamp), np.asarray(rotation, dtype=float), np.asarray(translation, dtype=float),
                    None if payload.get("covariance") is None else np.asarray(payload["covariance"], dtype=float))


class VioBridge:
    """Turn successive UVIO poses and UWB samples into F5/F6 factors."""

    def __init__(self, *, sigma_rotation_rad: float = 0.02, sigma_translation_m: float = 0.01,
                 sigma_uwb_m: float = 0.04):
        if min(sigma_rotation_rad, sigma_translation_m, sigma_uwb_m) <= 0:
            raise ValueError("VIO bridge sigmas must be positive")
        self.sigma_rotation_rad = float(sigma_rotation_rad)
        self.sigma_translation_m = float(sigma_translation_m)
        self.sigma_uwb_m = float(sigma_uwb_m)
        self._poses: dict[int, UvioPose] = {}
        self._previous: tuple[int, UvioPose] | None = None

    def add_pose(self, camera_id: int, pose: UvioPose | Mapping[str, Any]) -> tuple[PoseState, RelativePoseFactor | None]:
        value = _pose_from_payload(pose) if isinstance(pose, Mapping) else pose
        identifier = int(camera_id)
        camera_pose = PoseState(np.asarray(value.rotation_map_camera), np.asarray(value.translation_map_camera_m))
        relative = None
        if self._previous is not None:
            previous_id, previous = self._previous
            rotation_i_j = previous.rotation_map_camera.T @ value.rotation_map_camera
            translation_i_j = previous.rotation_map_camera.T @ (value.translation_map_camera_m - previous.translation_map_camera_m)
            relative = RelativePoseFactor(previous_id, identifier, rotation_i_j, translation_i_j,
                                          self.sigma_rotation_rad, self.sigma_translation_m)
        self._poses[identifier] = value
        self._previous = (identifier, value)
        return camera_pose, relative

    def add_uwb(self, camera_id: int, uwb_position_m: np.ndarray, *, sigma_m: float | None = None) -> SegmentUwbFactor:
        identifier = int(camera_id)
        if identifier not in self._poses:
            raise ValueError("UWB sample has no matching UVIO pose")
        position = np.asarray(uwb_position_m, dtype=float).reshape(3)
        if not np.all(np.isfinite(position)):
            raise ValueError("UWB position must be finite and already in map metres")
        return SegmentUwbFactor(self._poses[identifier].translation_map_camera_m, position,
                                self.sigma_uwb_m if sigma_m is None else float(sigma_m))

    def reset(self) -> None:
        self._poses.clear()
        self._previous = None
