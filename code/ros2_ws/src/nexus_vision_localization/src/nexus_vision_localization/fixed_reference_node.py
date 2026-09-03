#!/usr/bin/env python3
"""Surveyed AprilTags -> map platform pose; separate from tagged target output."""

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from nexus_vision_localization.detector import validate_camera_parameters
from nexus_vision_localization.reference_geometry import (
    SurveyedTag, board_to_map_body, pose_covariance, rigid_transform, solve_tag_board,
)
from nexus_vision_localization.vision_node import rotation_matrix_to_quaternion


def load_reference_calibration(path, allow_test=False):
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get("schema_version") != 1 or not data.get("calibration_id"):
        raise ValueError("reference calibration requires schema_version=1 and calibration_id")
    if data.get("calibration_status") != "measured" and not (allow_test and data.get("calibration_status") == "synthetic_test"):
        raise ValueError("reference calibration must be measured; synthetic data needs explicit opt-in")
    if data.get("reference_frame") != "map" or not data.get("camera_frame") or not data.get("tag_family"):
        raise ValueError("reference calibration must declare map, optical frame and tag family")
    if not isinstance(data.get("image_rectified"), bool):
        raise ValueError("image_rectified must be explicitly declared")
    rigid_transform(data["transform_body_camera"])
    pose_covariance(data["extrinsic_covariance"])
    pose_covariance(data["reference_covariance"])
    tags = [SurveyedTag(int(tag["id"]), float(tag["size_m"]), tag["transform_map_tag"]) for tag in data["tags"]]
    if not tags or len({tag.marker_id for tag in tags}) != len(tags):
        raise ValueError("reference map needs uniquely identified surveyed tags")
    if not np.isfinite(data["pixel_sigma_px"]) or data["pixel_sigma_px"] <= 0:
        raise ValueError("reference pixel uncertainty must be positive")
    return data, tags


class FixedReferenceNode(Node):
    def __init__(self):
        super().__init__("nexus_fixed_reference")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("allow_test_calibration", False)
        self.declare_parameter("detections_topic", "/apriltag/detections")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("max_age_ms", 300.0)
        self.calibration, self.tags = load_reference_calibration(
            str(self.get_parameter("calibration_file").value), bool(self.get_parameter("allow_test_calibration").value))
        self._matrix, self._distortion, self._camera_info_ns = None, None, None
        self._last_input_ns, self._last_valid_ns = 0, 0
        self._last_reason = None
        self._max_age_ns = int(float(self.get_parameter("max_age_ms").value) * 1e6)
        if self._max_age_ns <= 0:
            raise ValueError("max_age_ms must be positive")
        self._publisher = self.create_publisher(PoseWithCovarianceStamped, "/nexus/vision/platform_pose", 10)
        self._quality_publisher = self.create_publisher(String, "/nexus/observations/quality", 10)
        self._status_publisher = self.create_publisher(String, "/nexus/vision/reference_status", 10)
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._camera_info_callback, 10)
        self.create_subscription(AprilTagDetectionArray, str(self.get_parameter("detections_topic").value), self._detections_callback, 10)
        self.create_timer(.1, self._watchdog)

    def _status(self, reason, stamp_ns=None, valid=False):
        self._last_reason = reason
        self._status_publisher.publish(String(data=json.dumps({
            "schema_version": 1, "subject": "platform", "valid": valid, "reason": reason,
            "sample_timestamp_ns": stamp_ns, "last_valid_sample_timestamp_ns": self._last_valid_ns,
            "calibration_id": self.calibration["calibration_id"],
        }, allow_nan=False)))

    def _camera_info_callback(self, message):
        try:
            if message.header.frame_id != self.calibration["camera_frame"]:
                raise ValueError("reference_camera_frame_mismatch")
            if self.calibration["image_rectified"]:
                # Detection pixels belong to the rectified optical camera.
                matrix = np.array(message.p).reshape(3, 4)[:, :3]
                distortion = np.zeros(5)
            else:
                matrix = np.array(message.k).reshape(3, 3)
                distortion = np.array(message.d)
                if distortion.size == 0:
                    distortion = np.zeros(5)
            self._matrix, self._distortion = validate_camera_parameters(matrix, distortion)
            if not np.all(np.isfinite(self._distortion)):
                raise ValueError("nonfinite_reference_camera_distortion")
            self._camera_info_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        except (ValueError, TypeError) as error:
            self._matrix, self._distortion = None, None
            self._status(str(error))

    def _detections_callback(self, message):
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        try:
            now = self.get_clock().now().nanoseconds
            if stamp <= self._last_input_ns or stamp <= 0 or not 0 <= now - stamp <= self._max_age_ns:
                raise ValueError("reference_sample_time_invalid_or_stale")
            self._last_input_ns = stamp
            if message.header.frame_id != self.calibration["camera_frame"]:
                raise ValueError("reference_detection_frame_mismatch")
            if self._matrix is None:
                raise ValueError("reference_camera_info_missing")
            if self._camera_info_ns and not 0 <= stamp - self._camera_info_ns <= self._max_age_ns:
                raise ValueError("reference_camera_info_stale")
            detections = [(int(tag.id), np.array([[point.x, point.y] for point in tag.corners]))
                          for tag in message.detections if tag.family == self.calibration["tag_family"]]
            board = solve_tag_board(detections, self.tags, self._matrix, self._distortion,
                                    pixel_sigma_px=self.calibration["pixel_sigma_px"])
            pose, covariance = board_to_map_body(board, self.calibration["transform_body_camera"],
                                                 self.calibration["extrinsic_covariance"], self.calibration["reference_covariance"])
            if self.get_clock().now().nanoseconds - stamp > self._max_age_ns:
                raise ValueError("reference_solution_stale")
        except (ValueError, TypeError, cv2.error, np.linalg.LinAlgError) as error:
            self._status(str(error), stamp)
            return
        output = PoseWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = "map"
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = pose[:3, 3].tolist()
        quaternion = rotation_matrix_to_quaternion(pose[:3, :3]).tolist()
        output.pose.pose.orientation.x, output.pose.pose.orientation.y, output.pose.pose.orientation.z, output.pose.pose.orientation.w = quaternion
        output.pose.covariance = covariance.reshape(-1).tolist()
        area = sum(abs(cv2.contourArea(points.astype(np.float32))) for identifier, points in detections if identifier in board.tag_ids)
        self._quality_publisher.publish(String(data=json.dumps({
            "schema_version": 1, "subject": "platform", "source": "fixed_tag", "sample_timestamp_ns": stamp,
            "features": {"tag_area_px": area, "visible_points": board.inlier_count,
                         "inlier_ratio": board.inlier_ratio, "reprojection_error_px": board.reprojection_error_px},
        }, allow_nan=False)))
        self._publisher.publish(output)
        self._last_valid_ns = stamp
        self._status("", stamp, True)

    def _watchdog(self):
        if self._last_valid_ns and self.get_clock().now().nanoseconds - self._last_valid_ns > self._max_age_ns:
            if self._last_reason != "reference_lost":
                self._status("reference_lost", self._last_valid_ns)


def main(args=None):
    rclpy.init(args=args)
    node = FixedReferenceNode()
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
