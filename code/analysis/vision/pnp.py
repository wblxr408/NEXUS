import cv2
import numpy as np


def solve_pnp(object_points, image_points, camera_matrix, distortion):
    object_points = np.asarray(object_points, dtype=float)
    image_points = np.asarray(image_points, dtype=float)
    camera_matrix = np.asarray(camera_matrix, dtype=float)
    distortion = np.asarray(distortion, dtype=float)
    if object_points.shape[0] != image_points.shape[0] or object_points.shape[0] < 4:
        raise ValueError("at least four matching object/image points are required")
    if object_points.shape[1] != 3 or image_points.shape[1] != 2 or camera_matrix.shape != (3, 3):
        raise ValueError("invalid point or camera matrix shape")
    if (not np.all(np.isfinite(object_points)) or not np.all(np.isfinite(image_points))
            or not np.all(np.isfinite(camera_matrix)) or not np.all(np.isfinite(distortion))):
        raise ValueError("PnP inputs must be finite")
    if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
        raise ValueError("camera focal lengths must be positive")
    ok, rotation, translation = cv2.solvePnP(object_points, image_points, camera_matrix, distortion)
    if not ok:
        raise ValueError("solvePnP failed")
    return rotation.reshape(3), translation.reshape(3)


def reprojection_error(object_points, image_points, camera_matrix, distortion, rotation, translation):
    object_points = np.asarray(object_points, dtype=float)
    image_points = np.asarray(image_points, dtype=float)
    if object_points.ndim != 2 or object_points.shape[1] != 3 or image_points.shape != (object_points.shape[0], 2):
        raise ValueError("object and image points must have matching Nx3/Nx2 shapes")
    projected, _ = cv2.projectPoints(object_points, rotation, translation, camera_matrix, distortion)
    if not np.all(np.isfinite(projected)) or not np.all(np.isfinite(image_points)):
        raise ValueError("reprojection inputs must be finite")
    return float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
