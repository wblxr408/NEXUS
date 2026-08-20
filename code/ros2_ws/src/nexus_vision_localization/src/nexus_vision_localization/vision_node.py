#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from nexus_msgs.msg import TargetObservation

from .detector import reprojection_error, solve_marker_pose


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
        self.declare_parameter("target_id", "target_0")
        self.declare_parameter("tag_id", 0)
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("default_confidence", 0.0)
        self._camera_matrix = None
        self._distortion = None
        self._camera_info_stamp_ns = None
        self._camera_info_frame = ""
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._marker_size = float(self.get_parameter("marker_size_m").value)
        self._target_id = str(self.get_parameter("target_id").value)
        self._tag_id = int(self.get_parameter("tag_id").value)
        self._tag_family = str(self.get_parameter("tag_family").value)
        self._confidence = float(self.get_parameter("default_confidence").value)
        self._publisher = self.create_publisher(TargetObservation, "/nexus/vision/target_observation", 10)
        self.create_subscription(CameraInfo, "~/camera_info", self._camera_info_callback, 10)
        self.create_subscription(AprilTagDetectionArray, "~/detections", self._detections_callback, 10)
        if self._marker_size <= 0:
            self.get_logger().error("marker_size_m must be set to the measured positive AprilTag edge length")
        self.get_logger().warning("waiting for camera_info and AprilTag detections; no sensor source is synthesized")

    def _camera_info_callback(self, message):
        self._camera_matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        self._distortion = np.asarray(message.d, dtype=float)
        self._camera_info_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self._camera_info_frame = message.header.frame_id
        try:
            from .detector import validate_camera_parameters
            validate_camera_parameters(self._camera_matrix, self._distortion)
        except ValueError as error:
            self.get_logger().error(str(error))
            self._camera_matrix = None

    def _detections_callback(self, message):
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self.get_logger().error("rejecting AprilTag detections with an unset sample timestamp")
            return
        if self._camera_matrix is None or self._distortion is None:
            self.get_logger().error("waiting for valid camera_info before processing AprilTag detections")
            return
        detection_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self._camera_info_stamp_ns != detection_stamp_ns:
            self.get_logger().error("rejecting AprilTag detections without same-timestamp camera_info")
            return
        if self._camera_info_frame and message.header.frame_id and self._camera_info_frame != message.header.frame_id:
            self.get_logger().error("rejecting AprilTag detections with camera_info frame mismatch")
            return
        if self._marker_size <= 0:
            return
        matching = [
            detection for detection in message.detections
            if detection.id == self._tag_id and detection.family == self._tag_family
        ]
        if not matching:
            self.get_logger().warning("no configured AprilTag detected")
            return
        detection = matching[0]
        corners = [[corner.x, corner.y] for corner in detection.corners]
        try:
            rotation, translation = solve_marker_pose(
                corners, self._camera_matrix, self._distortion, self._marker_size)
            error = reprojection_error(
                corners, self._camera_matrix, self._distortion,
                rotation, translation, self._marker_size)
        except ValueError as exc:
            self.get_logger().warning(f"rejecting AprilTag {detection.id}: {exc}")
            return
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        output = TargetObservation()
        output.header = message.header
        output.header.frame_id = message.header.frame_id or self._camera_frame
        output.target_id = self._target_id
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
