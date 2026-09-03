#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from nexus_msgs.msg import TargetObservation

from nexus_coord_transform.geometry import matrix_to_quaternion

# Sandbox deck from simulation/sandbox_scene.yaml (unit mm): 4000 x 4700 envelope,
# deck_height 50, origin at the corner. Drawn as the F3 support-plane constraint.
SUPPORT_PLANE_X_M = 4.0
SUPPORT_PLANE_Y_M = 4.7
SUPPORT_PLANE_HEIGHT_M = 0.05
# Platform positions kept to draw the multi-view intersection bundle.
VIEW_RAY_HISTORY = 12


class DashboardNode(Node):
    def __init__(self):
        super().__init__("nexus_viz_dashboard")
        self._last_target = None
        self._last_platform = None
        self._last_observation = None
        self._platform_history = []
        self._observation_marker_pub = self.create_publisher(Marker, "/nexus/viz/target_observation", 10)
        self._target_marker_pub = self.create_publisher(Marker, "/nexus/viz/target_pose", 10)
        self.create_subscription(Odometry, "/nexus/fcu/odom", self._platform_callback, 10)
        # Gazebo publishes the platform on its own odometry topic; both feed the view rays.
        self.create_subscription(Odometry, "/nexus/gazebo/uav/odom", self._platform_callback, 10)
        self.create_subscription(TargetObservation, "/nexus/vision/map_target_observation", self._observation_callback, 10)
        self.create_subscription(TargetObservation, "/nexus/target/pose", self._callback, 10)
        self.create_timer(1.0, self._report)

    def _callback(self, message):
        self._last_target = message

    def _platform_callback(self, message):
        self._last_platform = message
        self._platform_history.append(message.pose.pose.position)
        self._platform_history = self._platform_history[-VIEW_RAY_HISTORY:]

    def _observation_callback(self, message):
        self._last_observation = message
        if message.validity != TargetObservation.VALIDITY_VALID:
            self.get_logger().warning(
                f"vision target invalid reason={message.invalid_reason} "
                f"last_valid_sample_ns={message.last_valid_sample_timestamp_ns}")
            return
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

    def _publish_covariance_ellipsoid(self, message):
        # Position block of the row-major 6x6 covariance (indices 0,1,2,6,7,8,12,13,14);
        # the full block is used, so anisotropic and correlated covariances stay visible.
        block = np.asarray(message.covariance, dtype=float).reshape(6, 6)[:3, :3]
        if not np.all(np.isfinite(block)):
            self.get_logger().warning(
                f"target={message.target_id} position covariance is not finite; no ellipsoid published")
            return
        eigenvalues, eigenvectors = np.linalg.eigh(block)
        if eigenvalues.min() <= 0.0:
            self.get_logger().warning(
                f"target={message.target_id} position covariance is not positive definite "
                f"(min eigenvalue={eigenvalues.min():.3e}); no ellipsoid published")
            return
        if np.linalg.det(eigenvectors) < 0.0:
            eigenvectors[:, 0] = -eigenvectors[:, 0]
        quaternion = matrix_to_quaternion(eigenvectors)
        marker = Marker()
        marker.header = message.header
        marker.ns = "nexus_target_covariance"
        marker.id = abs(hash(message.target_id)) % 2**31
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = message.pose.position
        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])
        # Marker scale is the full axis length, so 1-sigma semi-axes need 2 sigma.
        marker.scale.x, marker.scale.y, marker.scale.z = (
            2.0 * float(np.sqrt(value)) for value in eigenvalues)
        marker.color.r, marker.color.g, marker.color.b = (0.9, 0.2, 0.8)
        marker.color.a = 0.35
        self._target_marker_pub.publish(marker)

    def _publish_view_rays(self, message):
        # One line per kept platform position: depth comes from multi-view intersection,
        # not from a single ray.
        if not self._platform_history:
            return
        marker = Marker()
        marker.header = message.header
        marker.ns = "nexus_view_rays"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color.r, marker.color.g, marker.color.b = (0.2, 0.6, 1.0)
        marker.color.a = 0.7
        for platform in self._platform_history:
            marker.points.append(platform)
            marker.points.append(message.pose.position)
        self._target_marker_pub.publish(marker)

    def _publish_support_plane(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nexus_support_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = SUPPORT_PLANE_X_M / 2.0
        marker.pose.position.y = SUPPORT_PLANE_Y_M / 2.0
        marker.pose.position.z = SUPPORT_PLANE_HEIGHT_M
        marker.pose.orientation.w = 1.0
        marker.scale.x = SUPPORT_PLANE_X_M
        marker.scale.y = SUPPORT_PLANE_Y_M
        marker.scale.z = 0.005
        marker.color.r, marker.color.g, marker.color.b = (0.6, 0.6, 0.65)
        marker.color.a = 0.25
        self._target_marker_pub.publish(marker)

    def _report(self):
        self._publish_support_plane()
        if self._last_target is None:
            self.get_logger().info("no target pose received; waiting for real input")
            return
        message = self._last_target
        if message.validity != TargetObservation.VALIDITY_VALID:
            self.get_logger().warning(
                f"target pose invalid reason={message.invalid_reason} "
                f"last_valid_sample_ns={message.last_valid_sample_timestamp_ns}")
            return
        self._publish_marker(message, self._target_marker_pub, (0.0, 1.0, 0.2))
        self._publish_covariance_ellipsoid(message)
        self._publish_view_rays(message)
        variances = np.asarray(message.covariance, dtype=float).reshape(6, 6).diagonal()[:3]
        sigma = (f"({np.sqrt(variances[0]):.3f}, {np.sqrt(variances[1]):.3f}, {np.sqrt(variances[2]):.3f})"
                 if np.all(np.isfinite(variances)) and np.all(variances >= 0.0) else "unavailable")
        self.get_logger().info(
            f"target={message.target_id} frame={message.header.frame_id} "
            f"source={message.source_mode} confidence={message.confidence:.3f} "
            f"unit={message.unit} receive_timestamp_ns={message.receive_timestamp_ns} "
            f"position=({message.pose.position.x:.3f}, {message.pose.position.y:.3f}, {message.pose.position.z:.3f}) "
            f"sigma_xyz_m={sigma}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
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
