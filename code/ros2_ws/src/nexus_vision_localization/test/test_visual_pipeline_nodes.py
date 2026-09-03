"""Real DDS image -> detection/mask -> geometry -> platform factor wiring.

Network boundaries explicitly return synthetic detections/features. This tests
ROS orchestration and the real numerical back end, not pretrained inference.
"""

import json
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml

from nexus_fusion_localization.platform_node import PlatformLocalizationNode
from nexus_fusion_localization.platform_state import BodyState
from nexus_vision_localization import detection_node, superpoint_node
from nexus_vision_localization.object_detector import Detection2D
from nexus_vision_localization.superpoint_node import load_motion_calibration
from test_visual_motion import CAMERA, scene


def calibration(tmp_path):
    data = {
        "schema_version": 1, "calibration_id": "synthetic_visual_pipeline", "calibration_status": "synthetic_test",
        "camera_frame": "camera_optical_frame", "image_rectified": True, "transform_body_camera": np.eye(4).tolist(),
        "map_frame": "map", "body_frame": "base_link",
        "imu": {"frame_id": "imu_link", "rotation_body_imu": np.eye(3).tolist(), "at_body_origin_or_compensated": True,
                "noise": {"accel_density": .08, "gyro_density": .004, "accel_bias_walk": .001,
                          "gyro_bias_walk": .0001, "maximum_gap_s": .05}},
        "uwb": {"tag_body_m": [0., 0., 0.], "anchors_map_m": {}},
        "initial_velocity_sigma_mps": .2, "initial_accel_bias_sigma_mps2": .1, "initial_gyro_bias_sigma_rps": .01,
        "window": {"window_size": 6, "prediction_horizon_s": 5.},
    }
    path = tmp_path / "synthetic_pipeline.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_motion_node_rejects_unmeasured_calibration(tmp_path):
    path = calibration(tmp_path)
    with pytest.raises(ValueError, match="measured"):
        load_motion_calibration(path)
    assert load_motion_calibration(path, allow_test=True)["image_rectified"]


def test_image_detection_motion_platform_pipeline_with_synthetic_network_boundaries(tmp_path, monkeypatch):
    first, second, true_rotation, true_translation = scene(noise=0.)
    selected_masks = []

    class SyntheticDetector:
        manifest = {"class_names": ["moving_test_region"]}

        def __init__(self, _):
            pass

        def infer(self, image, **_):
            if image[0, 0, 0] == 3:
                raise ValueError("synthetic_detector_failure")
            return [Detection2D(0, "moving_test_region", .9, (0., 0., 640., 240.))]

    class SyntheticDense:
        def __init__(self, features):
            self.value = features

        def features(self, allowed, **_):
            selected_masks.append(allowed.copy())
            pixels = np.floor(self.value.points_px + .5).astype(int)
            return self.value.select(allowed[pixels[:, 1], pixels[:, 0]])

    class SyntheticEncoder:
        def __init__(self, _):
            pass

        def infer(self, image):
            return SyntheticDense(first if image[0, 0, 0] == 1 else second)

    monkeypatch.setattr(detection_node, "ObjectDetectorOnnx", SyntheticDetector)
    monkeypatch.setattr(superpoint_node, "SuperPointOnnx", SyntheticEncoder)
    path = calibration(tmp_path)
    rclpy.init(args=["--ros-args", "-p", f"calibration_file:={path}", "-p", "allow_test_calibration:=true",
                     "-p", "model_manifest:=synthetic_network_boundary", "-p", "max_age_ms:=3000.0",
                     "-p", "max_observation_age_ms:=3000.0", "-p", "reorder_delay_ms:=0.0"])
    detector, motion, platform = detection_node.ObjectDetectionNode(), superpoint_node.SuperPointMotionNode(), PlatformLocalizationNode()
    driver = Node("visual_pipeline_regression_driver")
    executor = SingleThreadedExecutor()
    for node in (detector, motion, platform, driver):
        executor.add_node(node)
    observations, qualities, statuses, odometry, detections = [], [], [], [], []
    driver.create_subscription(String, "/nexus/vision/body_motion", lambda msg: observations.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/observations/quality", lambda msg: qualities.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/vision/motion_status", lambda msg: statuses.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/vision/detections", lambda msg: detections.append(json.loads(msg.data)), 20)
    driver.create_subscription(Odometry, "/nexus/platform/odom", odometry.append, 20)
    images = driver.create_publisher(Image, "/camera/image_rect", 10)
    cameras = driver.create_publisher(CameraInfo, "/camera/camera_info", 10)
    imus = driver.create_publisher(Imu, "/nexus/fcu/imu", 100)
    bridge = CvBridge()

    def spin_until(predicate, timeout=8.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate(), "visual pipeline DDS condition timed out"

    def stamp(message, value, frame):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(value, 1_000_000_000)
        message.header.frame_id = frame
        return message

    def image_message(marker, value):
        array = np.zeros((480, 640, 3), np.uint8)
        array[0, 0, 0] = marker
        return stamp(bridge.cv2_to_imgmsg(array, encoding="bgr8"), value, "camera_optical_frame")

    try:
        spin_until(lambda: images.get_subscription_count() == 2 and cameras.get_subscription_count() > 0
                   and imus.get_subscription_count() > 0 and motion._motion_pub.get_subscription_count() >= 2
                   and detector._publisher.get_subscription_count() >= 2 and platform._odom_publisher.get_subscription_count() > 0)
        start = driver.get_clock().now().nanoseconds - 150_000_000
        platform.estimator.initialize(BodyState(start, np.zeros(3), true_translation / .1, np.eye(3)), np.eye(15) * .001)
        camera = stamp(CameraInfo(), start, "camera_optical_frame")
        camera.width, camera.height = 640, 480
        camera.p = np.column_stack((CAMERA, np.zeros(3))).reshape(-1).tolist()
        camera.k = np.eye(3).reshape(-1).tolist()  # rectified input must use P
        cameras.publish(camera)
        spin_until(lambda: motion._camera is not None)
        images.publish(image_message(1, start))
        spin_until(lambda: any(s["reason"] == "reference_initialized" for s in statuses))
        assert not observations
        omega = cv2.Rodrigues(true_rotation)[0].reshape(3) / .1
        for index in range(21):
            rotation = cv2.Rodrigues(omega * index * .005)[0]
            force = rotation.T @ np.array([0., 0., 9.80665])
            message = stamp(Imu(), start + index * 5_000_000, "imu_link")
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = force.tolist()
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = omega.tolist()
            imus.publish(message)
        spin_until(lambda: platform._last_imu_ns == start + 100_000_000)
        images.publish(image_message(2, start + 100_000_000))
        spin_until(lambda: observations and odometry and any(s["valid"] for s in statuses))
        packet = observations[-1]
        assert packet["previous_stamp_ns"] == start and packet["sample_timestamp_ns"] == start + 100_000_000
        assert packet["metric_translation"] is False
        assert len(selected_masks) == 2 and not selected_masks[-1][:240].any() and selected_masks[-1][300:].all()
        assert any(q["source"] == "superpoint" and q["sample_timestamp_ns"] == packet["sample_timestamp_ns"] for q in qualities)
        assert platform.estimator.last_source_stamps["superpoint"] == packet["sample_timestamp_ns"]
        output = odometry[-1]
        assert output.header.stamp.sec * 1_000_000_000 + output.header.stamp.nanosec == packet["sample_timestamp_ns"]
        actual = [output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z]
        np.testing.assert_allclose(actual, true_translation, atol=.02)
        assert output.header.frame_id == "map" and output.child_frame_id == "base_link"
        images.publish(image_message(3, start + 120_000_000))
        spin_until(lambda: any(s["reason"] == "dynamic_detection_failed" for s in statuses))
        assert len(observations) == 1 and len(selected_masks) == 2
        assert any(not d["valid"] and d["reason"] == "synthetic_detector_failure" for d in detections)
        # An unmatched later image cannot reuse an old frame's mask.
        unmatched = image_message(4, start + 130_000_000)
        motion._image_callback(unmatched)
        motion._drain()
        assert len(selected_masks) == 2
    finally:
        executor.shutdown()
        for node in (driver, platform, motion, detector):
            node.destroy_node()
        rclpy.shutdown()
