#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

from nexus_msgs.msg import TargetObservation

from nexus_vision_localization.detector import reprojection_error, solve_marker_pose


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
        self.declare_parameter("position_variance_m2", 0.0)
        self.declare_parameter("input_timeout_ms", 1000.0)
        self.declare_parameter("max_camera_info_age_ms", 1000.0)
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
        self._position_variance_m2 = float(
            self.get_parameter("position_variance_m2").value)
        self._input_timeout_ns = int(float(self.get_parameter("input_timeout_ms").value) * 1e6)
        self._max_camera_info_age_ns = int(
            float(self.get_parameter("max_camera_info_age_ms").value) * 1e6)
        self._last_detection_receive_ns = self.get_clock().now().nanoseconds
        self._last_valid_sample_ns = 0
        self._last_timeout_report_ns = 0
        self._publisher = self.create_publisher(TargetObservation, "/nexus/vision/target_observation", 10)
        self.create_subscription(CameraInfo, "~/camera_info", self._camera_info_callback, 10)
        self.create_subscription(AprilTagDetectionArray, "~/detections", self._detections_callback, 10)
        self.create_timer(0.25, self._input_watchdog)
        if self._marker_size <= 0:
            self.get_logger().error("marker_size_m must be set to the measured positive AprilTag edge length")
        self.get_logger().warning("waiting for camera_info and AprilTag detections; no sensor source is synthesized")

    @staticmethod
    def _stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def _receive_ns(self, sample_ns=0):
        return max(int(self.get_clock().now().nanoseconds), int(sample_ns))

    def _publish_invalid(self, header, reason):
        output = TargetObservation()
        output.header = header
        if not output.header.frame_id:
            output.header.frame_id = self._camera_info_frame or self._camera_frame
        sample_ns = self._stamp_ns(output.header)
        if sample_ns <= 0:
            now = self.get_clock().now().to_msg()
            output.header.stamp = now
            sample_ns = self._stamp_ns(output.header)
        output.receive_timestamp_ns = self._receive_ns(sample_ns)
        output.target_id = self._target_id
        output.source_mode = TargetObservation.SOURCE_DIRECT_VISION
        output.validity = TargetObservation.VALIDITY_INVALID
        output.invalid_reason = str(reason)
        output.last_valid_sample_timestamp_ns = self._last_valid_sample_ns
        output.unit = "m"
        output.pose.position.x = float("nan")
        output.pose.position.y = float("nan")
        output.pose.position.z = float("nan")
        output.pose.orientation.x = float("nan")
        output.pose.orientation.y = float("nan")
        output.pose.orientation.z = float("nan")
        output.pose.orientation.w = float("nan")
        output.covariance = [float("nan")] * 36
        output.confidence = 0.0
        self._publisher.publish(output)

    def _input_watchdog(self):
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns - self._last_detection_receive_ns <= self._input_timeout_ns:
            return
        if now_ns - self._last_timeout_report_ns < self._input_timeout_ns:
            return
        self._last_timeout_report_ns = now_ns
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._camera_info_frame or self._camera_frame
        self._publish_invalid(header, "no_detection_input")
        self.get_logger().warning("no AprilTag detection input received within input_timeout_ms")

    def _camera_info_callback(self, message):
        self._camera_matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        self._distortion = np.asarray(message.d, dtype=float)
        self._camera_info_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self._camera_info_frame = message.header.frame_id
        try:
            from nexus_vision_localization.detector import validate_camera_parameters
            validate_camera_parameters(self._camera_matrix, self._distortion)
        except ValueError as error:
            self.get_logger().error(str(error))
            self._camera_matrix = None

    def _detections_callback(self, message):
        self._last_detection_receive_ns = int(self.get_clock().now().nanoseconds)
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self.get_logger().error("rejecting AprilTag detections with an unset sample timestamp")
            self._publish_invalid(message.header, "missing_sample_timestamp")
            return
        if self._camera_matrix is None or self._distortion is None:
            self.get_logger().error("waiting for valid camera_info before processing AprilTag detections")
            self._publish_invalid(message.header, "missing_camera_info")
            return
        detection_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        camera_info_age_ns = detection_stamp_ns - int(self._camera_info_stamp_ns)
        if (self._camera_info_stamp_ns > 0
                and (camera_info_age_ns < 0
                     or camera_info_age_ns > self._max_camera_info_age_ns)):
            self.get_logger().error("rejecting AprilTag detections with stale or future camera_info")
            self._publish_invalid(message.header, "camera_info_stale")
            return
        if self._camera_info_frame and message.header.frame_id and self._camera_info_frame != message.header.frame_id:
            self.get_logger().error("rejecting AprilTag detections with camera_info frame mismatch")
            self._publish_invalid(message.header, "camera_info_frame_mismatch")
            return
        if self._marker_size <= 0:
            self._publish_invalid(message.header, "marker_size_unset")
            return
        if not math.isfinite(self._position_variance_m2) or self._position_variance_m2 <= 0:
            self._publish_invalid(message.header, "uncertainty_unset")
            return
        matching = [
            detection for detection in message.detections
            if detection.id == self._tag_id and detection.family == self._tag_family
        ]
        if not matching:
            self.get_logger().warning("no configured AprilTag detected")
            self._publish_invalid(message.header, "target_not_detected")
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
            self._publish_invalid(message.header, "pnp_failed")
            return
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        output = TargetObservation()
        output.header = message.header
        output.header.frame_id = message.header.frame_id or self._camera_frame
        sample_ns = self._stamp_ns(output.header)
        output.receive_timestamp_ns = self._receive_ns(sample_ns)
        output.target_id = self._target_id
        output.source_mode = TargetObservation.SOURCE_DIRECT_VISION
        output.validity = TargetObservation.VALIDITY_VALID
        output.invalid_reason = ""
        output.last_valid_sample_timestamp_ns = sample_ns
        output.unit = "m"
        output.pose = Pose()
        output.pose.position.x = float(translation[0])
        output.pose.position.y = float(translation[1])
        output.pose.position.z = float(translation[2])
        output.pose.orientation.x = float(quaternion[0])
        output.pose.orientation.y = float(quaternion[1])
        output.pose.orientation.z = float(quaternion[2])
        output.pose.orientation.w = float(quaternion[3])
        covariance = np.zeros((6, 6), dtype=float)
        covariance[0, 0] = covariance[1, 1] = covariance[2, 2] = self._position_variance_m2
        output.covariance = covariance.reshape(-1).tolist()
        output.confidence = float(max(0.0, min(1.0, self._confidence)))
        self._publisher.publish(output)
        self._last_valid_sample_ns = sample_ns
        self.get_logger().debug(f"target {output.target_id} reprojection_error_px={error:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = VisionLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
