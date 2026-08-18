#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from nexus_msgs.msg import TargetObservation


class DashboardNode(Node):
    def __init__(self):
        super().__init__("nexus_viz_dashboard")
        self._last_target = None
        self._last_platform = None
        self._last_observation = None
        self._observation_marker_pub = self.create_publisher(Marker, "/nexus/viz/target_observation", 10)
        self._target_marker_pub = self.create_publisher(Marker, "/nexus/viz/target_pose", 10)
        self.create_subscription(Odometry, "/nexus/fcu/odom", self._platform_callback, 10)
        self.create_subscription(TargetObservation, "/nexus/vision/target_observation", self._observation_callback, 10)
        self.create_subscription(TargetObservation, "/nexus/target/pose", self._callback, 10)
        self.create_timer(1.0, self._report)

    def _callback(self, message):
        self._last_target = message

    def _platform_callback(self, message):
        self._last_platform = message

    def _observation_callback(self, message):
        self._last_observation = message
        self._publish_marker(message, self._observation_marker_pub, (1.0, 0.7, 0.0))

    def _publish_marker(self, message, publisher, color):
        marker = Marker()
        marker.header = message.header
        marker.ns = "nexus_target"
        marker.id = abs(hash(message.target_id)) % 2**31
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = message.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        publisher.publish(marker)

    def _report(self):
        if self._last_target is None:
            self.get_logger().info("no target pose received; waiting for real input")
            return
        message = self._last_target
        self._publish_marker(message, self._target_marker_pub, (0.0, 1.0, 0.2))
        self.get_logger().info(
            f"target={message.target_id} frame={message.header.frame_id} "
            f"source={message.source_mode} confidence={message.confidence:.3f} "
            f"position=({message.pose.position.x:.3f}, {message.pose.position.y:.3f}, {message.pose.position.z:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
