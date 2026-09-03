#!/usr/bin/env python3
"""ROS2 UDP consumer for the read-only MAVLink JSON distribution stream.

This node never opens the flight-controller serial/TCP endpoint.  A single
upstream process owns that resource and emits JSON datagrams; this consumer
turns only the platform IMU/odometry fields into standard ROS2 messages.
"""

from __future__ import annotations

import json
import math
import socket
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _stamp(stamp_ns: int):
    return stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000


class TelemetryUdpNode(Node):
    def __init__(self):
        super().__init__("nexus_telemetry_udp")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 14551)
        self.declare_parameter("platform_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("position_variance_m2", 0.25)
        self.declare_parameter("poll_period_ms", 2.0)
        self.declare_parameter("max_datagrams_per_poll", 100)
        self._platform_frame = str(self.get_parameter("platform_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._max_datagrams_per_poll = max(1, int(self.get_parameter("max_datagrams_per_poll").value))
        self._position_variance_m2 = float(self.get_parameter("position_variance_m2").value)
        if not math.isfinite(self._position_variance_m2) or self._position_variance_m2 <= 0.0:
            raise ValueError("position_variance_m2 must be finite and positive")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((str(self.get_parameter("listen_host").value), int(self.get_parameter("listen_port").value)))
        self._socket.setblocking(False)
        self._imu_pub = self.create_publisher(Imu, "/nexus/fcu/imu", 20)
        self._odom_pub = self.create_publisher(Odometry, "/nexus/fcu/odom", 10)
        period = max(float(self.get_parameter("poll_period_ms").value) / 1000.0, 0.001)
        self.create_timer(period, self._poll)

    def destroy_node(self):
        self._socket.close()
        return super().destroy_node()

    def _poll(self):
        for _ in range(self._max_datagrams_per_poll):
            try:
                payload, _ = self._socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().error(f"UDP telemetry receive failed: {exc}")
                return
            try:
                state = json.loads(payload.decode("utf-8"))
                if not isinstance(state, dict):
                    continue
                self._publish_imu(state)
                self._publish_odom(state)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                self.get_logger().warning(f"ignoring malformed telemetry datagram: {exc}")

    def _publish_imu(self, state: dict[str, Any]):
        imu = state.get("imu") or {}
        acceleration = imu.get("acceleration_mps2") or {}
        angular = imu.get("angular_velocity_rps") or {}
        values = [_finite(acceleration.get(axis)) for axis in "xyz"] + [_finite(angular.get(axis)) for axis in "xyz"]
        if any(value is None for value in values):
            return
        stamp_us = _finite(imu.get("sample_timestamp_us"))
        if stamp_us is None or stamp_us <= 0:
            return
        if imu.get("sample_timestamp_domain") != "ros_unix_ns":
            self.get_logger().warning("refusing IMU sample with an unverified timestamp domain")
            return
        message = Imu()
        seconds, nanoseconds = _stamp(int(stamp_us * 1000.0))
        message.header.stamp.sec, message.header.stamp.nanosec = seconds, nanoseconds
        message.header.frame_id = "imu_link"
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = values[:3]
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = values[3:]
        message.orientation_covariance[0] = -1.0  # orientation is not supplied by the vendor stream
        self._imu_pub.publish(message)

    def _publish_odom(self, state: dict[str, Any]):
        # A cumulative snapshot is not a measurement.  Do not republish stale
        # or disconnected platform state with a fresh wall-clock timestamp.
        if state.get("online") is not True:
            return
        position = state.get("position") or {}
        values = [_finite(position.get(f"{axis}_m")) for axis in "xyz"]
        if any(value is None for value in values):
            return
        source_frame = str(position.get("frame_id") or "")
        if source_frame != self._map_frame:
            self.get_logger().warning(
                f"refusing platform odometry in unverified frame '{source_frame}'; expected '{self._map_frame}'")
            return
        if position.get("sample_timestamp_domain") != "ros_unix_ns":
            self.get_logger().warning("refusing platform odometry with an unverified timestamp domain")
            return
        sample_timestamp_ns = _finite(position.get("sample_timestamp_ns"))
        if sample_timestamp_ns is None or sample_timestamp_ns <= 0:
            return
        message = Odometry()
        seconds, fraction = _stamp(int(sample_timestamp_ns))
        message.header.stamp.sec, message.header.stamp.nanosec = seconds, fraction
        message.header.frame_id = source_frame
        message.child_frame_id = self._platform_frame
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = values
        message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[14] = self._position_variance_m2
        self._odom_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryUdpNode()
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
