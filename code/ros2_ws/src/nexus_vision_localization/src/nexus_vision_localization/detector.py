from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerPose:
    marker_id: int
    rotation_vector: np.ndarray
    translation_vector: np.ndarray


def validate_camera_parameters(camera_matrix, distortion_coefficients):
    matrix = np.asarray(camera_matrix, dtype=float)
    distortion = np.asarray(distortion_coefficients, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("camera_matrix must be a finite 3x3 matrix")
    if distortion.ndim != 1 or distortion.size not in (4, 5, 8, 12, 14):
        raise ValueError("distortion_coefficients must have a supported flat length")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera focal lengths must be positive")
    return matrix, distortion


def marker_object_points(marker_size_m):
    size = float(marker_size_m)
    if not np.isfinite(size) or size <= 0:
        raise ValueError("marker_size_m must be positive")
    half = size / 2.0
    return np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)


def solve_marker_pose(corners, camera_matrix, distortion_coefficients, marker_size_m):
    image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if image_points.shape != (4, 2) or not np.all(np.isfinite(image_points)):
        raise ValueError("corners must contain four finite image points")
    matrix, distortion = validate_camera_parameters(camera_matrix, distortion_coefficients)
    ok, rotation, translation = cv2.solvePnP(
        marker_object_points(marker_size_m), image_points, matrix, distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok or not np.all(np.isfinite(translation)):
        raise ValueError("solvePnP failed to produce a finite pose")
    return rotation.reshape(3), translation.reshape(3)


def reprojection_error(corners, camera_matrix, distortion_coefficients,
                       rotation_vector, translation_vector, marker_size_m):
    matrix, distortion = validate_camera_parameters(camera_matrix, distortion_coefficients)
    projected, _ = cv2.projectPoints(
        marker_object_points(marker_size_m), np.asarray(rotation_vector, dtype=float),
        np.asarray(translation_vector, dtype=float), matrix, distortion,
    )
    measured = np.asarray(corners, dtype=float).reshape(-1, 2)
    if measured.shape != (4, 2):
        raise ValueError("corners must contain four image points")
    return float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - measured) ** 2, axis=1))))


def detect_aruco_markers(image, dictionary_name="DICT_4X4_50"):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco support is unavailable in this environment")
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"unknown ArUco dictionary: {dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    if ids is None:
        return []
    return [(int(marker_id), corner.reshape(4, 2)) for marker_id, corner in zip(ids[:, 0], corners)]
