"""Target pose composition with platform, extrinsic and observation uncertainty."""

import numpy as np

from .reference_geometry import pose_covariance, rigid_transform


def _skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def compose_target_pose(transform_map_body, transform_body_camera, transform_camera_target,
                        platform_covariance, extrinsic_covariance, relative_covariance, *, joint_covariance=None):
    """Return T_map_target and ROS [translation, fixed-axis rotation] covariance.

    Platform covariance uses map translation and map fixed-axis rotation (ROS).
    Extrinsics use body translation and right rotation in camera; the relative
    observation uses camera translation and right rotation in target, as PnP.
    An 18x18 joint covariance may supply cross-correlations in that same order.
    Otherwise the three supplied covariance blocks are assumed independent.
    All transforms must refer to the image sample time, not receipt time.
    """
    body, camera, relative = (rigid_transform(item) for item in
                              (transform_map_body, transform_body_camera, transform_camera_target))
    covariances = [pose_covariance(item) for item in (platform_covariance, extrinsic_covariance, relative_covariance)]
    rotation_body, rotation_camera = body[:3, :3], body[:3, :3] @ camera[:3, :3]
    output = body @ camera @ relative
    platform_jacobian = np.eye(6)
    offset_map = rotation_body @ (camera[:3, 3] + camera[:3, :3] @ relative[:3, 3])
    platform_jacobian[:3, 3:] = -_skew(offset_map)
    extrinsic_jacobian = np.zeros((6, 6))
    extrinsic_jacobian[:3, :3] = rotation_body
    extrinsic_jacobian[:3, 3:] = -rotation_camera @ _skew(relative[:3, 3])
    extrinsic_jacobian[3:, 3:] = rotation_camera
    observation_jacobian = np.zeros((6, 6))
    observation_jacobian[:3, :3] = rotation_camera
    observation_jacobian[3:, 3:] = output[:3, :3]
    jacobian = np.column_stack((platform_jacobian, extrinsic_jacobian, observation_jacobian))
    if joint_covariance is None:
        joint = np.zeros((18, 18))
        for index, covariance in enumerate(covariances):
            joint[index * 6:index * 6 + 6, index * 6:index * 6 + 6] = covariance
    else:
        joint = np.asarray(joint_covariance, dtype=float)
        if (joint.shape != (18, 18) or not np.all(np.isfinite(joint)) or not np.allclose(joint, joint.T, atol=1e-9)
                or np.linalg.eigvalsh(joint).min() < -1e-12):
            raise ValueError("joint pose covariance must be positive semidefinite 18x18")
        for index, covariance in enumerate(covariances):
            if not np.allclose(joint[index * 6:index * 6 + 6, index * 6:index * 6 + 6], covariance, atol=1e-12, rtol=1e-8):
                raise ValueError("joint covariance diagonal blocks disagree with supplied marginals")
    covariance = jacobian @ joint @ jacobian.T
    return output, (covariance + covariance.T) / 2
