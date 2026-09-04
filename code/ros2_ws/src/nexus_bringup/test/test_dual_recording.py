import json
import os
import signal
import subprocess
import time

from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

from nexus_bringup.dual_recording import prepare_recording, recording_topics
from nexus_fusion_localization.target_kinematic_node import TargetKinematicFusionNode
from nexus_msgs.msg import TargetKinematicState
from nexus_viz_dashboard.localization_dashboard_node import LocalizationDashboardNode


def test_recording_requires_explicit_run_and_refuses_overwrite(tmp_path):
    config = tmp_path / "calibration.yaml"
    config.write_text("synthetic: true\n")
    with pytest.raises(ValueError):
        prepare_recording(str(tmp_path / "output"), "UNREGISTERED", "LIVE", {}, recording_topics())
    command = prepare_recording(str(tmp_path / "output"), "synthetic_test", "SIMULATION", {"calibration": config}, recording_topics())
    assert command[:3] == ["ros2", "bag", "record"]
    before = (tmp_path / "output/run.json").read_bytes()
    with pytest.raises(FileExistsError):
        prepare_recording(str(tmp_path / "output"), "synthetic_test", "SIMULATION", {"calibration": config}, recording_topics())
    assert (tmp_path / "output/run.json").read_bytes() == before
    assert (tmp_path / "output/configuration/calibration.yaml").read_bytes() == config.read_bytes()


def test_real_rosbag_roundtrip_preserves_sensor_stamps_dual_outputs_and_history(tmp_path):
    topics = recording_topics()
    command = prepare_recording(str(tmp_path / "run"), "synthetic_record_roundtrip", "SIMULATION", {}, topics)
    rclpy.init(args=["--ros-args", "-p", "input_mode:=SIMULATION", "-p", "run_id:=synthetic_record_roundtrip"])
    driver, fusion, display = Node("recording_test_driver"), TargetKinematicFusionNode(), LocalizationDashboardNode()
    executor = SingleThreadedExecutor()
    for node in (driver, fusion, display):
        executor.add_node(node)
    specs = {"/camera/image_rect": Image, "/camera/camera_info": CameraInfo, "/nexus/fcu/imu": Imu,
             "/nexus/uwb/ranges": String, "/nexus/platform/odom": Odometry,
             "/nexus/vision/target_kinematics": TargetKinematicState, "/nexus/observations/quality": String}
    publishers = {topic: driver.create_publisher(kind, topic, qos_profile_sensor_data if kind in (Image, CameraInfo, Imu) else 20)
                  for topic, kind in specs.items()}
    log = (tmp_path / "recorder.log").open("w")
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stamp = 0

    def spin(predicate, timeout=6.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), (tmp_path / "recorder.log").read_text()

    try:
        spin(lambda: all(pub.get_subscription_count() >= (2 if topic in {"/nexus/vision/target_kinematics", "/nexus/platform/odom", "/nexus/observations/quality"} else 1)
                         for topic, pub in publishers.items()) and fusion._publisher.get_subscription_count() >= 2
             and display.snapshot_publisher.get_subscription_count() >= 1)
        stamp = driver.get_clock().now().nanoseconds
        messages = {topic: kind() for topic, kind in specs.items()}
        for msg in messages.values():
            if hasattr(msg, "header"):
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                msg.header.frame_id = "camera_optical_frame"
        image = messages["/camera/image_rect"]
        image.height, image.width, image.step, image.encoding, image.data = 2, 2, 2, "mono8", [0, 1, 2, 3]
        imu = messages["/nexus/fcu/imu"]
        imu.header.frame_id, imu.linear_acceleration.z = "imu_link", 9.80665
        odom = messages["/nexus/platform/odom"]
        odom.header.frame_id, odom.child_frame_id, odom.pose.pose.orientation.w = "map", "base_link", 1.
        odom.pose.pose.position.z, odom.twist.twist.linear.x = 1.8, .73
        ranges = {"schema_version": 1, "frame_id": "map", "unit": "m", "sample_timestamp_ns": stamp,
                  "anchor_ids": ["a", "b", "c", "d"], "ranges_m": [2., 3., 4., 5.], "variances_m2": [.01] * 4}
        messages["/nexus/uwb/ranges"].data = json.dumps(ranges)
        target = messages["/nexus/vision/target_kinematics"]
        target.header.frame_id, target.unit, target.target_id = "map", "m", "static_test_target"
        target.valid = target.position_observed = True
        target.state, target.source, target.position_reference = "confirmed", "superpoint_multiview_static", "reference_feature:test:0"
        target.receive_timestamp_ns = target.last_valid_sample_timestamp_ns = stamp
        target.pose.position.x, target.pose.position.y, target.pose.position.z = .4, .2, .1
        target.pose.orientation.x = target.pose.orientation.y = target.pose.orientation.z = target.pose.orientation.w = float("nan")
        target.velocity.x = target.velocity.y = target.velocity.z = float("nan")
        covariance = np.full((9, 9), np.nan)
        covariance[:3, :3] = np.eye(3) * .01
        target.covariance, target.confidence = covariance.reshape(-1).tolist(), 1.
        messages["/nexus/observations/quality"].data = json.dumps({
            "schema_version": 1, "subject": "target", "target_id": target.target_id,
            "source": target.source, "position_reference": target.position_reference,
            "sample_timestamp_ns": stamp, "features": {"inlier_ratio": .9}})
        for topic, message in messages.items():
            publishers[topic].publish(message)
        spin(lambda: target.target_id in fusion.estimator.tracks)
        until = time.monotonic() + .7
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            log.close()
            executor.shutdown()
            for node in (driver, fusion, display):
                node.destroy_node()
            rclpy.shutdown()
    assert process.returncode == 0, (tmp_path / "recorder.log").read_text()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path / "run/bag"), storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {entry.name: get_message(entry.type) for entry in reader.get_all_topics_and_types()}
    recorded = {}
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        recorded.setdefault(topic, []).append(deserialize_message(payload, types[topic]))
    assert set(specs) <= set(recorded)
    for topic, kind in specs.items():
        message = recorded[topic][0]
        if hasattr(message, "header"):
            assert message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec == stamp
        else:
            assert json.loads(message.data)["sample_timestamp_ns"] == stamp
    outputs = recorded["/nexus/target/kinematics"]
    assert any(msg.valid and msg.last_valid_sample_timestamp_ns == stamp for msg in outputs)
    assert any(msg.historical and not msg.valid and msg.last_valid_sample_timestamp_ns == stamp for msg in outputs)
    assert all(not msg.velocity_observed and np.isnan(msg.velocity.x) for msg in outputs)
    states = [json.loads(msg.data) for msg in recorded["/nexus/viz/localization_state"]]
    assert any(state["targets"] and state["targets"][0]["historical_position"] == [.4, .2, .1] for state in states)
    assert "/nexus/target/kinematic_fusion_status" in recorded
