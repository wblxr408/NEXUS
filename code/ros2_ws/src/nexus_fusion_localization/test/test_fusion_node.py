"""Real ROS2 publisher/subscriber test, run in an isolated localhost domain."""

import json
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from nexus_msgs.msg import TargetObservation
from nexus_fusion_localization.fusion_node import FusionLocalizationNode
from nexus_fusion_localization.reliability import ReliabilityMLP, covariance_scales


def test_ros_observation_outlier_diagnostics_and_stale_watchdog():
    rclpy.init(args=["--ros-args", "-p", "max_age_ms:=150.0"])
    estimator = FusionLocalizationNode()
    driver = Node("fusion_regression_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(estimator)
    executor.add_node(driver)
    results, diagnostics = [], []
    output_subscription = driver.create_subscription(
        TargetObservation, "/nexus/target/pose", results.append, 10)
    status_subscription = driver.create_subscription(
        String, "/nexus/target/fusion_status", lambda msg: diagnostics.append(json.loads(msg.data)), 10)
    publisher = driver.create_publisher(TargetObservation, "/nexus/vision/map_target_observation", 10)

    def spin_until(predicate, timeout=3.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert predicate(), "ROS2 integration condition timed out"

    def send(x):
        msg = TargetObservation()
        stamp = driver.get_clock().now().nanoseconds
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id, msg.target_id, msg.unit = "map", "test_target", "m"
        msg.receive_timestamp_ns = stamp
        msg.source_mode, msg.validity, msg.confidence = 2, 1, .95
        msg.pose.position.x = float(x)
        msg.pose.orientation.w = 1.
        msg.covariance = (np.eye(6) * .001).reshape(-1).tolist()
        publisher.publish(msg)
        return stamp

    try:
        spin_until(lambda: publisher.get_subscription_count() > 0
                   and estimator._publisher.get_subscription_count() > 0
                   and estimator._quality_publisher.get_subscription_count() > 0)
        first_stamp = send(1.)
        spin_until(lambda: results and diagnostics)
        assert results[-1].validity == TargetObservation.VALIDITY_VALID
        assert results[-1].header.stamp.sec * 1_000_000_000 + results[-1].header.stamp.nanosec == first_stamp
        assert results[-1].pose.position.x == 1.
        assert diagnostics[-1]["decision"] == "accepted"
        outlier_stamp = send(100.)
        spin_until(lambda: len(diagnostics) >= 2 and any(msg.invalid_reason == "innovation_outlier" for msg in results))
        rejected = next(msg for msg in results if msg.invalid_reason == "innovation_outlier")
        assert rejected.validity == TargetObservation.VALIDITY_INVALID
        assert np.isnan(rejected.pose.position.x)
        assert rejected.last_valid_sample_timestamp_ns == first_stamp
        assert diagnostics[-1]["sample_timestamp_ns"] == outlier_stamp
        send(1.01)
        spin_until(lambda: len(diagnostics) >= 3 and results[-1].validity == TargetObservation.VALIDITY_VALID)
        assert abs(results[-1].pose.position.x - 1.) < .02
        spin_until(lambda: results[-1].invalid_reason == "stale")
        assert np.isnan(results[-1].pose.position.x)
    finally:
        driver.destroy_subscription(output_subscription)
        driver.destroy_subscription(status_subscription)
        executor.shutdown()
        driver.destroy_node()
        estimator.destroy_node()
        rclpy.shutdown()


def test_node_loads_model_and_uses_only_exact_sample_quality(tmp_path, monkeypatch):
    rng = np.random.default_rng(2)
    features = rng.normal(size=(30, 20))
    labels = rng.uniform(.2, .8, size=(30, 4))
    model = ReliabilityMLP(seed=2)
    model.fit(features, labels, [f"synthetic_{i // 10}" for i in range(30)],
              epochs=3, label_provenance="synthetic node wiring test")
    path = tmp_path / "model.npz"
    model.save(path)
    rclpy.init(args=["--ros-args", "-p", "reliability_mode:=learned",
                     "-p", f"reliability_model:={path}"])
    estimator = FusionLocalizationNode()
    try:
        for payload in ("null", "[]", '{"schema_version":1}'):
            estimator._quality_callback(String(data=payload))
        warnings = []
        with monkeypatch.context() as patch:
            patch.setattr(estimator, "get_logger", lambda: SimpleNamespace(warning=warnings.append))
            estimator._quality_callback(String(data=json.dumps({
                "schema_version": 1, "subject": "platform", "source": "fixed_tag",
                "sample_timestamp_ns": estimator.get_clock().now().nanoseconds,
                "features": {"inlier_ratio": .9},
            })))
        assert not warnings and not estimator._quality_features
        msg = TargetObservation()
        stamp = estimator.get_clock().now().nanoseconds
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id, msg.target_id, msg.unit = "map", "model_target", "m"
        msg.receive_timestamp_ns = stamp
        msg.source_mode, msg.validity, msg.confidence = 2, 1, .9
        msg.pose.orientation.w = 1.
        msg.covariance = (np.eye(6) * .01).reshape(-1).tolist()
        quality = {"schema_version": 1, "target_id": msg.target_id, "source_mode": 2,
                   "sample_timestamp_ns": stamp, "features": {"inlier_ratio": .8}}
        estimator._quality_callback(String(data=json.dumps(quality)))
        estimator._callback(msg)
        expected = covariance_scales(model.predict({"inlier_ratio": .8, "network_latency_s": 0.}))[0]
        np.testing.assert_allclose(estimator._filter.tracks[msg.target_id].covariance[:3, :3],
                                   np.eye(3) * .01 * expected)
        assert not estimator._quality_features
        assert estimator._filter.last_diagnostic["decision"] == "accepted"
    finally:
        estimator.destroy_node()
        rclpy.shutdown()
