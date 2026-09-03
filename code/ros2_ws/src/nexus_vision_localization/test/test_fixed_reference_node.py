"""Real tag-detection messages through the surveyed reference ROS2 node."""

import json
import time

import cv2
import numpy as np
import pytest
import rclpy
import yaml
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from nexus_vision_localization.fixed_reference_node import FixedReferenceNode, load_reference_calibration
from nexus_vision_localization.reference_geometry import SurveyedTag


def calibration(tmp_path):
    tags = []
    for index in range(3):
        pose = np.eye(4)
        pose[:3, 3] = [.4 * (index - 1), .15 * (index % 2), 0.]
        tags.append({"id": index + 2, "size_m": .2, "transform_map_tag": pose.tolist()})
    body_camera = np.diag([1., -1., -1., 1.])
    body_camera[:3, 3] = [.05, 0., -.03]
    data = {"schema_version": 1, "calibration_id": "synthetic_reference_node_test", "calibration_status": "synthetic_test",
            "reference_frame": "map", "camera_frame": "camera_rect", "tag_family": "36h11", "image_rectified": True,
            "transform_body_camera": body_camera.tolist(), "extrinsic_covariance": (np.eye(6) * 1e-6).tolist(),
            "reference_covariance": (np.eye(6) * 1e-6).tolist(), "pixel_sigma_px": 1., "tags": tags}
    path = tmp_path / "reference_test.yaml"
    path.write_text(yaml.safe_dump(data))
    return path, data


def test_reference_node_requires_surveyed_map_or_explicit_synthetic_opt_in(tmp_path):
    path, data = calibration(tmp_path)
    with pytest.raises(ValueError, match="measured"):
        load_reference_calibration(path)
    assert len(load_reference_calibration(path, True)[1]) == 3
    data["tags"].append(data["tags"][0])
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="uniquely"):
        load_reference_calibration(path, True)


def test_fixed_tag_dds_uses_rectified_intrinsics_and_keeps_sample_time(tmp_path):
    path, data = calibration(tmp_path)
    rclpy.init(args=["--ros-args", "-p", f"calibration_file:={path}", "-p", "allow_test_calibration:=true",
                     "-p", "max_age_ms:=1000.0"])
    node = FixedReferenceNode()
    driver = Node("fixed_reference_regression_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    poses, quality, status = [], [], []
    driver.create_subscription(PoseWithCovarianceStamped, "/nexus/vision/platform_pose", poses.append, 10)
    driver.create_subscription(String, "/nexus/observations/quality", lambda msg: quality.append(json.loads(msg.data)), 10)
    driver.create_subscription(String, "/nexus/vision/reference_status", lambda msg: status.append(json.loads(msg.data)), 10)
    camera_pub = driver.create_publisher(CameraInfo, "/camera/camera_info", 10)
    detection_pub = driver.create_publisher(AprilTagDetectionArray, "/apriltag/detections", 10)

    def spin_until(condition, timeout=3.):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert condition(), "reference DDS condition timed out"

    matrix = np.array([[700., 0., 320.], [0., 700., 240.], [0., 0., 1.]])
    camera_map = np.eye(4)
    camera_map[:3, :3] = cv2.Rodrigues(np.array([np.pi - .15, .1, .05]))[0]
    camera_map[:3, 3] = [.1, .1, 2.]
    expected_body = np.linalg.inv(camera_map) @ np.linalg.inv(np.array(data["transform_body_camera"]))
    try:
        spin_until(lambda: camera_pub.get_subscription_count() and detection_pub.get_subscription_count()
                   and node._publisher.get_subscription_count() and node._quality_publisher.get_subscription_count()
                   and node._status_publisher.get_subscription_count())
        info = CameraInfo()
        info.header.frame_id = "camera_rect"
        info.width, info.height = 640, 480
        info.k = [500., 0., 320., 0., 510., 240., 0., 0., 1.]
        info.p = np.column_stack((matrix, np.zeros(3))).reshape(-1).tolist()
        info.d = [.2, -.1, .001, .001, 0.]
        camera_pub.publish(info)
        spin_until(lambda: node._matrix is not None)
        message = AprilTagDetectionArray()
        stamp = driver.get_clock().now().nanoseconds
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id = "camera_rect"
        for specification in data["tags"]:
            tag = SurveyedTag(specification["id"], specification["size_m"], specification["transform_map_tag"])
            pixels = cv2.projectPoints(tag.corners(), cv2.Rodrigues(camera_map[:3, :3])[0], camera_map[:3, 3], matrix, np.zeros(5))[0].reshape(4, 2)
            detection = AprilTagDetection()
            detection.id, detection.family = tag.marker_id, "36h11"
            for point, pixel in zip(detection.corners, pixels):
                point.x, point.y = pixel.tolist()
            message.detections.append(detection)
        detection_pub.publish(message)
        spin_until(lambda: poses and quality)
        output = poses[-1]
        assert output.header.frame_id == "map"
        assert output.header.stamp.sec * 1_000_000_000 + output.header.stamp.nanosec == stamp
        position = output.pose.pose.position
        np.testing.assert_allclose([position.x, position.y, position.z], expected_body[:3, 3], atol=1e-5)
        assert np.all(np.linalg.eigvalsh(np.array(output.pose.covariance).reshape(6, 6)) > 0)
        assert quality[-1]["subject"] == "platform" and quality[-1]["source"] == "fixed_tag"
        assert quality[-1]["sample_timestamp_ns"] == stamp
        assert quality[-1]["features"]["visible_points"] == 12
        unknown = AprilTagDetectionArray()
        unknown.header.frame_id = "camera_rect"
        unknown.header.stamp = driver.get_clock().now().to_msg()
        detection = AprilTagDetection()
        detection.id, detection.family = 999, "36h11"
        unknown.detections.append(detection)
        detection_pub.publish(unknown)
        spin_until(lambda: any(s["reason"] == "no_surveyed_reference_visible" for s in status))
        assert len(poses) == 1
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
