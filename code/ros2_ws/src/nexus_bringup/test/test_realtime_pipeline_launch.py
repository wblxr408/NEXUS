import time
import unittest
from pathlib import Path

import launch
import launch.actions
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from nexus_msgs.msg import TargetObservation
from sensor_msgs.msg import CameraInfo
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    main_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("nexus_bringup"))
                / "launch" / "target_localization.launch.py")),
        launch_arguments={
            "hardware_required": "false",
            "use_rviz": "false",
            "marker_size_m": "0.2",
            "position_variance_m2": "0.01",
            "max_pair_delta_ms": "50.0",
        }.items(),
    )
    return launch.LaunchDescription([
        main_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRealtimePipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("realtime_pipeline_test_client")
        cls.input_publisher = cls.node.create_publisher(
            TargetObservation, "/nexus/vision/target_observation", 10)
        cls.map_vision_publisher = cls.node.create_publisher(
            TargetObservation, "/nexus/vision/map_target_observation", 10)
        cls.uwb_publisher = cls.node.create_publisher(
            TargetObservation, "/nexus/uwb/target_observation", 10)
        cls.camera_info_publisher = cls.node.create_publisher(
            CameraInfo, "/camera/camera_info", 10)
        cls.detection_publisher = cls.node.create_publisher(
            AprilTagDetectionArray, "/apriltag/detections", 10)
        cls.outputs = []
        cls.map_outputs = []
        cls.node.create_subscription(
            TargetObservation, "/nexus/target/pose",
            lambda message: cls.outputs.append(message), 10)
        cls.node.create_subscription(
            TargetObservation, "/nexus/vision/map_target_observation",
            lambda message: cls.map_outputs.append(message), 10)
        cls.tf_broadcaster = StaticTransformBroadcaster(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_camera_observation_reaches_map_fusion_output(self):
        self._send_camera_transform()

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            now = self.node.get_clock().now()
            sample_ns = now.nanoseconds
            message = TargetObservation()
            message.header.stamp = now.to_msg()
            message.header.frame_id = "camera_test"
            message.receive_timestamp_ns = sample_ns
            message.target_id = "test_target"
            message.source_mode = TargetObservation.SOURCE_DIRECT_VISION
            message.validity = TargetObservation.VALIDITY_VALID
            message.last_valid_sample_timestamp_ns = sample_ns
            message.unit = "m"
            message.pose.position.x = 1.0
            message.pose.position.y = 2.0
            message.pose.position.z = 3.0
            message.pose.orientation.w = 1.0
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = covariance[14] = 0.01
            message.covariance = covariance
            message.confidence = 0.8
            self.input_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            valid = [item for item in self.outputs
                     if item.validity == TargetObservation.VALIDITY_VALID]
            if valid:
                output = valid[-1]
                self.assertEqual(output.header.frame_id, "map")
                self.assertEqual(output.target_id, "test_target")
                self.assertEqual(output.unit, "m")
                self.assertAlmostEqual(output.pose.position.x, 11.0, places=5)
                self.assertAlmostEqual(output.pose.position.y, 2.0, places=5)
                self.assertAlmostEqual(output.pose.position.z, 3.0, places=5)
                self.assertGreaterEqual(output.receive_timestamp_ns,
                                        output.last_valid_sample_timestamp_ns)
                return
        self.fail("camera observation did not reach /nexus/target/pose in map frame")

    def _send_camera_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_test"
        transform.transform.translation.x = 10.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def _map_observation(self, target_id, source_mode, stamp_ns, x):
        message = TargetObservation()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = "map"
        message.receive_timestamp_ns = stamp_ns
        message.target_id = target_id
        message.source_mode = source_mode
        message.validity = TargetObservation.VALIDITY_VALID
        message.last_valid_sample_timestamp_ns = stamp_ns
        message.unit = "m"
        message.pose.position.x = x
        message.pose.orientation.w = 1.0
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = covariance[14] = 0.01
        message.covariance = covariance
        message.confidence = 0.8
        return message

    def test_unsynchronized_sources_degrade_to_newest_source(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            now_ns = self.node.get_clock().now().nanoseconds
            vision = self._map_observation(
                "pair_target", TargetObservation.SOURCE_DIRECT_VISION,
                now_ns - 100_000_000, 1.0)
            uwb = self._map_observation(
                "pair_target", TargetObservation.SOURCE_DIRECT_UWB,
                now_ns, 2.0)
            self.map_vision_publisher.publish(vision)
            self.uwb_publisher.publish(uwb)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            candidates = [
                item for item in self.outputs
                if item.target_id == "pair_target"
                and item.validity == TargetObservation.VALIDITY_VALID
                and item.source_mode == TargetObservation.SOURCE_DIRECT_UWB
            ]
            if candidates:
                self.assertAlmostEqual(candidates[-1].pose.position.x, 2.0)
                return
        self.fail("unsynchronized pair did not degrade to newest UWB observation")

    def test_vision_pnp_node_reaches_map_output(self):
        self._send_camera_transform()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            now = self.node.get_clock().now().to_msg()
            camera = CameraInfo()
            camera.header.stamp = now
            camera.header.frame_id = "camera_test"
            camera.width = 640
            camera.height = 480
            camera.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
            camera.d = [0.0] * 5
            detections = AprilTagDetectionArray()
            detections.header = camera.header
            detection = AprilTagDetection()
            detection.family = "36h11"
            detection.id = 0
            for corner, coordinates in zip(
                    detection.corners,
                    ((270.0, 290.0), (370.0, 290.0),
                     (370.0, 190.0), (270.0, 190.0))):
                corner.x, corner.y = coordinates
            detections.detections = [detection]
            self.camera_info_publisher.publish(camera)
            self.detection_publisher.publish(detections)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            candidates = [
                item for item in self.outputs
                if item.target_id == "target_0"
                and item.validity == TargetObservation.VALIDITY_VALID
            ]
            if candidates:
                output = candidates[-1]
                self.assertEqual(output.header.frame_id, "map")
                self.assertAlmostEqual(output.pose.position.x, 10.0, places=3)
                self.assertAlmostEqual(output.pose.position.y, 0.0, places=3)
                self.assertAlmostEqual(output.pose.position.z, 1.0, places=3)
                return
        self.fail("AprilTag detection did not reach the map-frame fusion output")

    def test_missing_transform_produces_explicit_invalid_state(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            now_ns = self.node.get_clock().now().nanoseconds
            message = self._map_observation(
                "missing_tf_target", TargetObservation.SOURCE_DIRECT_VISION,
                now_ns, 1.0)
            message.header.frame_id = "unknown_camera"
            self.input_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            candidates = [
                item for item in self.map_outputs
                if item.target_id == "missing_tf_target"
            ]
            if candidates:
                output = candidates[-1]
                self.assertEqual(output.validity, TargetObservation.VALIDITY_INVALID)
                self.assertEqual(output.invalid_reason, "transform_unavailable")
                return
        self.fail("missing transform did not produce an explicit invalid state")

    def test_fusion_watchdog_reports_stale_after_input_stops(self):
        now_ns = self.node.get_clock().now().nanoseconds
        message = self._map_observation(
            "stale_target", TargetObservation.SOURCE_DIRECT_UWB, now_ns, 3.0)
        self.uwb_publisher.publish(message)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            candidates = [
                item for item in self.outputs
                if item.target_id == "stale_target"
                and item.validity == TargetObservation.VALIDITY_INVALID
                and item.invalid_reason == "stale"
            ]
            if candidates:
                self.assertEqual(candidates[-1].last_valid_sample_timestamp_ns, now_ns)
                return
        self.fail("fusion watchdog did not publish stale invalid status")


@launch_testing.post_shutdown_test()
class TestProcessesExit(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
