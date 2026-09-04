#!/usr/bin/env python3
"""Receive v2 read-only Pi JSON and publish ROS2 IMU/UWB observations."""

import json
import socket
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from nexus_pi_readonly_ingress.telemetry_codec import TelemetryCodec


class TelemetryIngressNode(Node):
    def __init__(self):
        super().__init__("nexus_pi_readonly_telemetry_ingress")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 14551)
        self.declare_parameter("transport", "tcp_client")
        self.declare_parameter("remote_host", "192.168.1.143")
        self.declare_parameter("hardware_config", "")
        path = str(self.get_parameter("hardware_config").value)
        if not path:
            raise ValueError("hardware_config is required")
        with open(path, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        uwb = config["uwb"]
        self.codec = TelemetryCodec(uwb["tag_id"], uwb["range_slot_anchor_ids"], uwb["range_variance_m2"])
        self.transport = str(self.get_parameter("transport").value)
        if self.transport not in {"udp", "tcp_client"}:
            raise ValueError("transport must be udp or tcp_client")
        self.socket = None
        self.tcp_buffer = b""
        self.next_connect_ns = 0
        if self.transport == "udp":
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((str(self.get_parameter("listen_host").value), int(self.get_parameter("listen_port").value)))
            self.socket.setblocking(False)
        self.imu_pub = self.create_publisher(Imu, "/nexus/fcu/imu", 100)
        self.uwb_pub = self.create_publisher(String, "/nexus/uwb/ranges", 20)
        self.health_pub = self.create_publisher(String, "/nexus/pi/telemetry_health", 10)
        self.received = self.invalid = self.imu_count = self.uwb_count = 0
        self.started_ns = time.time_ns()
        self.last_imu_latency_ms = None
        self.last_receive_ns = 0
        self.create_timer(0.002, self.poll)
        self.create_timer(1.0, self.publish_health)

    def destroy_node(self):
        if self.socket is not None:
            self.socket.close()
        return super().destroy_node()

    def poll(self):
        if self.transport == "tcp_client":
            self.poll_tcp()
            return
        for _ in range(100):
            try:
                payload, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            self.consume(payload, time.time_ns())

    def connect_tcp(self):
        now = time.time_ns()
        if self.socket is not None or now < self.next_connect_ns:
            return
        try:
            self.socket = socket.create_connection(
                (str(self.get_parameter("remote_host").value), int(self.get_parameter("listen_port").value)),
                timeout=0.1)
            self.socket.setblocking(False)
            self.tcp_buffer = b""
        except OSError:
            self.socket = None
            self.next_connect_ns = now + 1_000_000_000

    def poll_tcp(self):
        self.connect_tcp()
        if self.socket is None:
            return
        try:
            while True:
                chunk = self.socket.recv(65535)
                if not chunk:
                    raise ConnectionResetError("telemetry stream closed")
                self.tcp_buffer += chunk
                while b"\n" in self.tcp_buffer:
                    payload, self.tcp_buffer = self.tcp_buffer.split(b"\n", 1)
                    self.consume(payload, time.time_ns())
        except BlockingIOError:
            return
        except OSError:
            self.socket.close()
            self.socket = None
            self.next_connect_ns = time.time_ns() + 1_000_000_000

    def consume(self, payload, arrival):
        try:
            state = json.loads(payload.decode("utf-8"))
            if not isinstance(state, dict):
                raise ValueError("root must be object")
            self.received += 1
            self.last_receive_ns = arrival
            self.publish_imu(self.codec.imu_packet(state, arrival))
            packet = self.codec.uwb_packet(state, arrival)
            if packet:
                self.uwb_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
                self.uwb_count += 1
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError, KeyError):
            self.invalid += 1

    def publish_imu(self, packet):
        if not packet:
            return
        message = Imu()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(packet["stamp_ns"], 1_000_000_000)
        message.header.frame_id = "imu_link"
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = packet["acceleration"]
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = packet["angular_velocity"]
        message.orientation_covariance[0] = -1.0
        for index in (0, 4, 8):
            message.linear_acceleration_covariance[index] = 0.04
            message.angular_velocity_covariance[index] = 0.0004
        self.imu_pub.publish(message)
        self.imu_count += 1
        self.last_imu_latency_ms = packet["latency_ms"]

    def publish_health(self):
        now = time.time_ns()
        elapsed = max((now - self.started_ns) / 1e9, 1e-6)
        data = {"schema_version": 1, "valid": bool(self.last_receive_ns and now - self.last_receive_ns < 3_000_000_000),
                "received_datagrams": self.received, "invalid_datagrams": self.invalid,
                "published_imu": self.imu_count, "published_uwb": self.uwb_count,
                "boot_clock_resets": self.codec.aligner.reset_count,
                "pi_clock_resets": self.codec.uwb_clock_aligner.reset_count,
                "input_hz": self.received / elapsed, "imu_hz": self.imu_count / elapsed,
                "uwb_hz": self.uwb_count / elapsed,
                "estimated_missing_imu_samples": self.codec.estimated_missing_imu_samples,
                "nonmonotonic_imu_samples": self.codec.nonmonotonic_imu_samples,
                "last_imu_latency_ms": self.last_imu_latency_ms,
                "last_receive_age_ms": None if not self.last_receive_ns else (now - self.last_receive_ns) / 1e6}
        self.health_pub.publish(String(data=json.dumps(data, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryIngressNode()
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
