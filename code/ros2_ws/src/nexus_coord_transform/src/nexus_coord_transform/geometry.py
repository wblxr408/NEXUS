import numpy as np


CARLA_TO_MAP_AXIS = np.diag([1.0, -1.0, 1.0])
CARLA_ACTOR_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                                    [1.0, 0.0, 0.0],
                                    [0.0, -1.0, 0.0]])
BODY_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])


def carla_rotation_matrix(pitch_deg, yaw_deg, roll_deg):
    """Return CARLA's actor-to-world rotation (pitch, yaw, roll in degrees).

    CARLA's ``get_matrix`` convention is not the generic RzRyRx expression;
    retaining the explicit formula avoids a silent sign/axis inversion.
    """
    theta, psi, phi = np.radians([pitch_deg, yaw_deg, roll_deg])
    ct, st, cp, sp, cf, sf = np.cos(theta), np.sin(theta), np.cos(psi), np.sin(psi), np.cos(phi), np.sin(phi)
    return np.array([
        [ct * cp, cp * st * sf - sp * cf, -cp * st * cf - sp * sf],
        [ct * sp, sp * st * sf + cp * cf, -sp * st * cf + cp * sf],
        [st, -ct * sf, ct * cf],
    ])


def carla_point_to_map(point_world_c):
    point = np.asarray(point_world_c, dtype=float).reshape(3)
    if not np.all(np.isfinite(point)):
        raise ValueError("CARLA point must be finite")
    return CARLA_TO_MAP_AXIS @ point


def carla_rotation_to_map(rotation_world_c):
    rotation = np.asarray(rotation_world_c, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(rotation)) or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("CARLA rotation must be orthonormal")
    result = CARLA_TO_MAP_AXIS @ rotation @ CARLA_TO_MAP_AXIS
    if np.linalg.det(result) <= 0:
        raise ValueError("converted CARLA rotation must be proper")
    return result


def carla_camera_pose(location_world_c, pitch_deg, yaw_deg, roll_deg):
    """Convert a CARLA camera actor pose to ``R_map_camera, t_map_camera``."""
    rotation_actor = carla_rotation_matrix(pitch_deg, yaw_deg, roll_deg)
    rotation_map_camera = CARLA_TO_MAP_AXIS @ rotation_actor @ CARLA_ACTOR_TO_OPTICAL
    translation_map_camera = carla_point_to_map(location_world_c)
    if np.linalg.det(rotation_map_camera) <= 0 or not np.allclose(rotation_map_camera.T @ rotation_map_camera, np.eye(3), atol=1e-6):
        raise ValueError("CARLA camera conversion did not produce a proper rotation")
    return rotation_map_camera, translation_map_camera


def body_to_camera_rotation(mount_pitch_deg=0.0):
    """REP-103 body (x forward, y left, z up) to optical (x right, y down, z forward)."""
    angle = np.radians(float(mount_pitch_deg))
    cy, sy = np.cos(angle), np.sin(angle)
    mount = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    result = mount @ BODY_TO_OPTICAL
    if np.linalg.det(result) <= 0:
        raise ValueError("body-to-camera rotation must be proper")
    return result


def apply_rigid_transform(rotation, translation, point):
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(translation, dtype=float).reshape(3)
    point = np.asarray(point, dtype=float).reshape(3)
    if not all(np.all(np.isfinite(value)) for value in (rotation, translation, point)):
        raise ValueError("rigid transform values must be finite")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("rotation must be orthonormal")
    return rotation @ point + translation


def rotate_position_covariance(covariance, rotation):
    covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(covariance)):
        raise ValueError("position covariance must be finite")
    if not np.allclose(covariance, covariance.T, atol=1e-9):
        raise ValueError("position covariance must be symmetric")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("rotation must be orthonormal")
    return rotation @ covariance @ rotation.T


def direct_target_measurement(target_in_sensor, sensor_to_map_rotation, sensor_to_map_translation):
    """Direct target measurement path: target_link observation to map."""
    return apply_rigid_transform(sensor_to_map_rotation, sensor_to_map_translation,
                                 target_in_sensor)


def platform_relative_measurement(platform_in_map, platform_rotation_map,
                                  target_relative_platform):
    """Indirect path: platform pose plus target-relative observation to map."""
    return apply_rigid_transform(platform_rotation_map, platform_in_map,
                                 target_relative_platform)


def centimeters_to_meters(values):
    array = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError("coordinates must be finite")
    return array / 100.0


def meters_to_centimeters(values):
    array = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError("coordinates must be finite")
    return array * 100.0


def ned_to_enu(values):
    array = np.asarray(values, dtype=float)
    if array.shape[-1] != 3 or not np.all(np.isfinite(array)):
        raise ValueError("NED coordinates must be finite vectors with three components")
    return np.stack((array[..., 1], array[..., 0], -array[..., 2]), axis=-1)


def quaternion_to_matrix(quaternion_xyzw):
    q = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
    if not np.all(np.isfinite(q)):
        raise ValueError("quaternion must be finite")
    norm = np.linalg.norm(q)
    if norm <= 0:
        raise ValueError("quaternion norm must be positive")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix):
    rotation = np.asarray(matrix, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix must be finite")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("rotation matrix must be orthonormal")
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2.0
        return np.array([(rotation[2, 1] - rotation[1, 2]) / scale,
                         (rotation[0, 2] - rotation[2, 0]) / scale,
                         (rotation[1, 0] - rotation[0, 1]) / scale,
                         scale / 4.0])
    index = int(np.argmax(np.diag(rotation)))
    other = [(index + 1) % 3, (index + 2) % 3]
    scale = np.sqrt(max(1e-15, 1.0 + rotation[index, index] - rotation[other[0], other[0]] - rotation[other[1], other[1]])) * 2.0
    result = np.zeros(4)
    result[index] = scale / 4.0
    result[3] = (rotation[other[1], other[0]] - rotation[other[0], other[1]]) / scale
    result[other[0]] = (rotation[other[0], index] + rotation[index, other[0]]) / scale
    result[other[1]] = (rotation[other[1], index] + rotation[index, other[1]]) / scale
    return result


def umeyama_alignment(source, target):
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must have matching N x 3 shapes")
    if source.shape[0] < 3 or not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("at least three finite point pairs are required")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    centered_source = source - source_mean
    centered_target = target - target_mean
    covariance = centered_target.T @ centered_source / source.shape[0]
    u, _, vh = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vh) < 0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vh
    translation = target_mean - rotation @ source_mean
    return rotation, translation
