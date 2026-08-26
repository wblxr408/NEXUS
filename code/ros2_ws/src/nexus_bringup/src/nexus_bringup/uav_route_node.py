#!/usr/bin/env python3
"""Continuously move the Gazebo display UAV around fixed sandbox waypoints."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


WAYPOINTS = (
    (0.70, 0.70),
    (3.30, 0.70),
    (3.30, 4.00),
    (0.70, 4.00),
)


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class UavRouteNode(Node):
    def __init__(self):
        super().__init__("uav_route_node")
        self.position = None
        self.yaw = 0.0
        self.waypoint_index = 0
        self.publisher = self.create_publisher(Twist, "/nexus/gazebo/uav/cmd_vel", 10)
        self.create_subscription(Odometry, "/nexus/gazebo/uav/odom", self.on_odometry, 10)
        self.create_timer(0.05, self.command_next_waypoint)

    def on_odometry(self, message):
        pose = message.pose.pose
        self.position = (pose.position.x, pose.position.y)
        self.yaw = yaw_from_quaternion(pose.orientation)

    def command_next_waypoint(self):
        if self.position is None:
            return
        target_x, target_y = WAYPOINTS[self.waypoint_index]
        delta_x = target_x - self.position[0]
        delta_y = target_y - self.position[1]
        distance = math.hypot(delta_x, delta_y)
        if distance < 0.10:
            self.waypoint_index = (self.waypoint_index + 1) % len(WAYPOINTS)
            return
        desired_yaw = math.atan2(delta_y, delta_x)
        yaw_error = wrap_angle(desired_yaw - self.yaw)
        command = Twist()
        command.angular.z = max(-1.2, min(1.2, 2.0 * yaw_error))
        if abs(yaw_error) < 0.25:
            command.linear.x = min(0.55, 0.7 * distance)
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = UavRouteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
