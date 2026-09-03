"""ROS2 platform ingest/output integration in a dedicated localhost domain."""

import json
import time

import numpy as np
import pytest
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from nexus_fusion_localization.platform_node import PlatformLocalizationNode, load_platform_calibration


def calibration_file(tmp_path):
    document = {
        "schema_version": 1, "calibration_id": "synthetic_platform_node_test",
        "calibration_status": "synthetic_test", "map_frame": "map", "body_frame": "base_link",
        "imu": {"frame_id": "imu_link", "rotation_body_imu": np.diag([1., -1., -1.]).tolist(),
                "at_body_origin_or_compensated": True,
                "noise": {"accel_density": .08, "gyro_density": .004,
                          "accel_bias_walk": .001, "gyro_bias_walk": .0001, "maximum_gap_s": .05}},
        "uwb": {"tag_body_m": [0., 0., 0.], "anchors_map_m": {}},
        "initial_velocity_sigma_mps": .1, "initial_accel_bias_sigma_mps2": .1,
        "initial_gyro_bias_sigma_rps": .01,
        "window": {"window_size": 3, "prediction_horizon_s": .25, "kernel": "linear", "irls_iterations": 1},
    }
    path = tmp_path / "platform_test.yaml"
    path.write_text(yaml.safe_dump(document))
    return path, document


def test_live_node_refuses_unverified_calibration_and_missing_noise(tmp_path):
    path, document = calibration_file(tmp_path)
    with pytest.raises(ValueError, match="measured"):
        load_platform_calibration(path)
    assert load_platform_calibration(path, allow_test=True)["calibration_id"] == document["calibration_id"]
    del document["imu"]["noise"]["gyro_density"]
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="noise"):
        load_platform_calibration(path, allow_test=True)


def test_platform_ros_imu_axes_position_update_outlier_and_loss(tmp_path):
    path, _ = calibration_file(tmp_path)
    rclpy.init(args=["--ros-args", "-p", f"calibration_file:={path}", "-p", "allow_test_calibration:=true",
                     "-p", "max_observation_age_ms:=1000.0", "-p", "reorder_delay_ms:=0.0"])
    node = PlatformLocalizationNode()
    driver = Node("platform_regression_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    poses, statuses = [], []
    driver.create_subscription(Odometry, "/nexus/platform/odom", poses.append, 20)
    driver.create_subscription(String, "/nexus/platform/status", lambda msg: statuses.append(json.loads(msg.data)), 50)
    imu_pub = driver.create_publisher(Imu, "/nexus/fcu/imu", 100)
    seed_pub = driver.create_publisher(PoseWithCovarianceStamped, "/nexus/platform/initial_pose", 10)
    uwb_pub = driver.create_publisher(Odometry, "/nexus/fcu/odom", 10)
    range_pub = driver.create_publisher(String, "/nexus/uwb/ranges", 10)

    def spin_until(condition, timeout=3.):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert condition(), "platform DDS condition timed out"

    def header(message, stamp, frame):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id = frame
        return message

    try:
        spin_until(lambda: all(pub.get_subscription_count() for pub in (imu_pub, seed_pub, uwb_pub, range_pub))
                   and node._odom_publisher.get_subscription_count() > 0 and node._status_publisher.get_subscription_count() > 0)
        start = driver.get_clock().now().nanoseconds - 120_000_000
        seed = header(PoseWithCovarianceStamped(), start, "map")
        seed.pose.pose.position.z, seed.pose.pose.orientation.w = 1., 1.
        seed.pose.covariance = (np.eye(6) * .001).reshape(-1).tolist()
        seed_pub.publish(seed)
        spin_until(lambda: len(poses) == 1)
        for i in range(25):
            message = header(Imu(), start + i * 5_000_000, "imu_link")
            message.linear_acceleration.z = -9.80665  # declared test IMU has reversed y/z axes
            imu_pub.publish(message)
        spin_until(lambda: node._last_imu_ns == start + 120_000_000)
        observed_stamp = start + 100_000_000
        observation = header(Odometry(), observed_stamp, "map")
        observation.child_frame_id = "base_link"
        observation.pose.pose.position.z = 1.
        observation.pose.covariance = (np.eye(6) * .0025).reshape(-1).tolist()
        uwb_pub.publish(observation)
        spin_until(lambda: len(poses) >= 2)
        output = poses[-1]
        assert output.child_frame_id == "base_link" and output.header.frame_id == "map"
        assert output.header.stamp.sec * 1_000_000_000 + output.header.stamp.nanosec == observed_stamp
        assert abs(output.pose.pose.position.z - 1.) < .001
        assert abs(output.twist.twist.linear.z) < .001
        assert output.pose.pose.orientation.w == pytest.approx(1., abs=1e-5)
        assert np.isfinite(output.twist.twist.angular.z)
        malformed = header(PoseWithCovarianceStamped(), driver.get_clock().now().nanoseconds, "map")
        malformed.pose.pose.orientation.w = 2.
        malformed.pose.covariance = (np.eye(6) * .001).reshape(-1).tolist()
        with pytest.raises(ValueError, match="unit quaternion"):
            node._pose_measurement(malformed)
        unavailable = header(Imu(), driver.get_clock().now().nanoseconds, "imu_link")
        unavailable.linear_acceleration_covariance[0] = -1.
        count = len(node.estimator.imu.samples)
        imu_pub.publish(unavailable)
        spin_until(lambda: any("marked unavailable" in s["reason"] for s in statuses))
        assert len(node.estimator.imu.samples) == count
        # Even a usable IMU stream must not turn a UWB jump into platform motion.
        jump = header(Odometry(), start + 110_000_000, "map")
        jump.child_frame_id = "base_link"
        jump.pose.pose.position.x, jump.pose.pose.position.z = 100., 1.
        jump.pose.covariance = (np.eye(6) * .0025).reshape(-1).tolist()
        uwb_pub.publish(jump)
        spin_until(lambda: any(any(d.get("reason") == "innovation_outlier" for d in s.get("diagnostics", [])) for s in statuses))
        assert len(poses) == 2
        range_pub.publish(String(data=json.dumps({"schema_version": 1, "unit": "cm", "frame_id": "map"})))
        spin_until(lambda: any(s["reason"].startswith("ranges_rejected:") for s in statuses))
        spin_until(lambda: any(s["reason"] == "external_constraints_lost" for s in statuses))
        assert len(poses) == 2
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
