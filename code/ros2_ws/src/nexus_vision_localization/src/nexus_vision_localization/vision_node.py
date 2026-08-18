#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from nexus_msgs.msg import TargetObservation

from .detector import detect_aruco_markers, reprojection_error, solve_marker_pose


def rotation_matrix_to_quaternion(matrix):
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return np.array([(matrix[2, 1] - matrix[1, 2]) / scale,
                         (matrix[0, 2] - matrix[2, 0]) / scale,
                         (matrix[1, 0] - matrix[0, 1]) / scale,
                         0.25 * scale])
    index = int(np.argmax(np.diag(matrix)))
    next_index = (index + 1) % 3
    last_index = (index + 2) % 3
    scale = math.sqrt(max(1e-15, 1.0 + matrix[index, index] - matrix[next_index, next_index] - matrix[last_index, last_index])) * 2.0
    quaternion = np.zeros(4)
    quaternion[index] = 0.25 * scale
    quaternion[3] = (matrix[last_index, next_index] - matrix[next_index, last_index]) / scale
    quaternion[next_index] = (matrix[next_index, index] + matrix[index, next_index]) / scale
    quaternion[last_index] = (matrix[last_index, index] + matrix[index, last_index]) / scale
    return quaternion


class VisionLocalizationNode(Node):
    def __init__(self):
        super().__init__("nexus_vision_localization")
        self.declare_parameter("marker_size_m", 0.0)
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("target_prefix", "target")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("default_confidence", 0.0)
        self._bridge = CvBridge()
        self._camera_matrix = None
        self._distortion = None
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._marker_size = float(self.get_parameter("marker_size_m").value)
        self._target_prefix = str(self.get_parameter("target_prefix").value)
        self._dictionary = str(self.get_parameter("dictionary").value)
        self._confidence = float(self.get_parameter("default_confidence").value)
        self._publisher = self.create_publisher(TargetObservation, "/nexus/vision/target_observation", 10)
        self.create_subscription(CameraInfo, "~/camera_info", self._camera_info_callback, 10)
        self.create_subscription(Image, "~/image", self._image_callback, 10)
        if self._marker_size <= 0:
            self.get_logger().error("marker_size_m must be set to a positive real camera marker size")
        self.get_logger().warning("waiting for camera_info and image input; no sensor source is synthesized")

    def _camera_info_callback(self, message):
        self._camera_matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        self._distortion = np.asarray(message.d, dtype=float)
        try:
            from .detector import validate_camera_parameters
            validate_camera_parameters(self._camera_matrix, self._distortion)
        except ValueError as error:
            self.get_logger().error(str(error))
            self._camera_matrix = None

    def _image_callback(self, message):
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self.get_logger().error("rejecting image with an unset sample timestamp")
            return
        if self._camera_matrix is None or self._distortion is None:
            self.get_logger().error("waiting for valid camera_info before processing images")
            return
        if self._marker_size <= 0:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
            markers = detect_aruco_markers(image, self._dictionary)
        except (RuntimeError, ValueError, cv2.error) as error:
            self.get_logger().error(f"target detector unavailable or failed: {error}")
            return
        if not markers:
            self.get_logger().warning("no target marker detected in image")
            return
        for marker_id, corners in markers:
            try:
                rotation, translation = solve_marker_pose(
                    corners, self._camera_matrix, self._distortion, self._marker_size)
                error = reprojection_error(
                    corners, self._camera_matrix, self._distortion,
                    rotation, translation, self._marker_size)
            except (ValueError, cv2.error) as exc:
                self.get_logger().warning(f"rejecting marker {marker_id}: {exc}")
                continue
            rotation_matrix, _ = cv2.Rodrigues(rotation)
            quaternion = rotation_matrix_to_quaternion(rotation_matrix)
            output = TargetObservation()
            output.header = message.header
            output.header.frame_id = message.header.frame_id or self._camera_frame
            output.target_id = f"{self._target_prefix}_{marker_id}"
            output.source_mode = TargetObservation.SOURCE_DIRECT_VISION
            output.pose = Pose()
            output.pose.position.x = float(translation[0])
            output.pose.position.y = float(translation[1])
            output.pose.position.z = float(translation[2])
            output.pose.orientation.x = float(quaternion[0])
            output.pose.orientation.y = float(quaternion[1])
            output.pose.orientation.z = float(quaternion[2])
            output.pose.orientation.w = float(quaternion[3])
            output.covariance = [float("nan")] * 36
            output.confidence = float(max(0.0, min(1.0, self._confidence)))
            self._publisher.publish(output)
            self.get_logger().debug(f"target {output.target_id} reprojection_error_px={error:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = VisionLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
