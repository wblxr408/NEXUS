from copy import deepcopy
import json
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from nexus_msgs.msg import TargetKinematicState
from nexus_viz_dashboard.localization_dashboard_node import LocalizationDashboardNode


def target(stamp, identifier="fixed", historical=False, observed=None):
    msg = TargetKinematicState()
    msg.header.frame_id, msg.unit, msg.target_id = "map", "m", identifier
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.receive_timestamp_ns = stamp
    msg.last_valid_sample_timestamp_ns = stamp if observed is None else observed
    msg.valid = msg.position_observed = not historical
    msg.historical = historical
    msg.state, msg.reason = ("lost", "target_not_observed") if historical else ("confirmed", "")
    msg.source, msg.position_reference = "correlated_target_fusion", "reference_feature:test:0"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 1., 2., .5
    msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = msg.pose.orientation.w = float("nan")
    msg.velocity.x = msg.velocity.y = msg.velocity.z = float("nan")
    covariance = np.full((9, 9), np.nan)
    covariance[:3, :3] = np.diag([.01, .04, .09])
    msg.covariance = covariance.reshape(-1).tolist()
    return msg


def platform(stamp):
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = "map", "base_link"
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.pose.pose.position.x, msg.pose.pose.position.z = -2., 3.
    msg.pose.pose.orientation.w = 1.
    msg.twist.twist.linear.x, msg.twist.twist.linear.z = 3., 4.
    return msg


def test_dual_display_dds_outputs_observed_history_and_removes_stale_platform():
    rclpy.init(args=[])
    display, driver = LocalizationDashboardNode(), Node("dual_display_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(display)
    executor.add_node(driver)
    states, markers = [], []
    driver.create_subscription(String, "/nexus/viz/localization_state", lambda msg: states.append(json.loads(msg.data)), 20)
    driver.create_subscription(MarkerArray, "/nexus/viz/localization_markers", markers.append, 20)
    target_pub = driver.create_publisher(TargetKinematicState, "/nexus/target/kinematics", 20)
    platform_pub = driver.create_publisher(Odometry, "/nexus/platform/odom", 20)

    def spin(predicate):
        until = time.monotonic() + 4.
        while not predicate() and time.monotonic() < until:
            executor.spin_once(timeout_sec=.005)
        assert predicate(), states[-2:]

    try:
        spin(lambda: target_pub.get_subscription_count() and platform_pub.get_subscription_count()
             and display.snapshot_publisher.get_subscription_count() and display.marker_publisher.get_subscription_count())
        stamp = driver.get_clock().now().nanoseconds
        target_pub.publish(target(stamp))
        platform_pub.publish(platform(stamp))
        spin(lambda: any(s["targets"] and s["targets"][0]["position"] and s["platform"]["position"] for s in states))
        current = next(s for s in states if s["targets"] and s["targets"][0]["position"] and s["platform"]["position"])
        assert current["platform"]["position"] == [-2., 0., 3.]
        assert current["platform"]["speed_mps"] == 5.
        assert current["targets"][0]["position"] == [1., 2., .5]
        assert current["targets"][0]["orientation"] is None
        np.testing.assert_allclose(current["targets"][0]["sigma_m"], [.1, .2, .3])
        spin(lambda: states[-1]["targets"] and states[-1]["targets"][0]["display_state"] == "historical")
        old = states[-1]["targets"][0]
        assert old["position"] is None and old["historical_position"] == [1., 2., .5]
        assert old["last_valid_sample_timestamp_ns"] == str(stamp)
        assert states[-1]["platform"]["position"] is None
        spin(lambda: markers and any("HISTORY" in m.text for m in markers[-1].markers))
        assert markers[-1].markers[0].action == Marker.DELETEALL
        assert not any(m.ns == "platform" and m.action == Marker.ADD for m in markers[-1].markers)
    finally:
        executor.shutdown()
        display.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()


def test_observation_epoch_orders_status_preserves_reference_and_clock_reset(monkeypatch):
    rclpy.init(args=[])
    node = LocalizationDashboardNode()
    now = [1_780_000_000_500_000_000]
    monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])))
    try:
        first = now[0] - 200_000_000
        node.on_target(target(first))
        node.on_target(target(now[0], historical=True, observed=first))
        newer = target(first + 100_000_000)
        newer.state = "re-associated"
        node.on_target(newer)
        assert node.snapshot(now[0])["targets"][0]["state"] == "re-associated"
        node.on_target(target(now[0], historical=True, observed=first))
        assert node.snapshot(now[0])["targets"][0]["state"] == "re-associated"
        invalid = deepcopy(newer)
        invalid.header.frame_id = "base_link"
        invalid.target_id = "wrong_frame"
        node.on_target(invalid)
        assert "wrong_frame" not in node.targets
        future = target(now[0] + 1, "future")
        node.on_target(future)
        assert "future" not in node.targets
        # History arriving first after a display restart is still visibly old.
        node.on_target(target(now[0], "history_only", True, first))
        assert node.snapshot(now[0])["targets"][1]["historical_position"] == [1., 2., .5]
        node.publish_state()
        session = node.session_id
        now[0] -= 1_000_000_000
        node.publish_state()
        assert not node.targets and node.platform is None and node.session_id != session
    finally:
        node.destroy_node()
        rclpy.shutdown()
