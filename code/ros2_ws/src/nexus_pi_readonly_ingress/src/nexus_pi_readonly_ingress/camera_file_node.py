#!/usr/bin/env python3
"""Publish atomically written IMX219 v2 files as ROS Image/CameraInfo."""

import io
import json
import socket
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


class CameraFileNode(Node):
    def __init__(self):
        super().__init__("nexus_pi_camera_file_ingress")
        self.declare_parameter("camera_dir", "")
        self.declare_parameter("poll_period_ms", 5.0)
        self.declare_parameter("transport", "tcp_client")
        self.declare_parameter("remote_host", "192.168.1.143")
        self.declare_parameter("remote_port", 14552)
        self.camera_dir = Path(str(self.get_parameter("camera_dir").value))
        self.transport = str(self.get_parameter("transport").value)
        if self.transport not in {"file", "tcp_client"}:
            raise ValueError("transport must be file or tcp_client")
        self.last_stamp = 0
        self.socket = None
        self.buffer = b""
        self.next_connect_ns = 0
        self.frames = self.invalid = 0
        self.started_ns = time.time_ns()
        self.last_health_publish_ns = 0
        self.raw_pub = self.create_publisher(Image, "/nexus/camera/imx219/image_raw", 5)
        self.compressed_pub = self.create_publisher(CompressedImage, "/nexus/camera/imx219/image_raw/compressed", 5)
        self.info_pub = self.create_publisher(CameraInfo, "/nexus/camera/imx219/camera_info", 5)
        self.health_pub = self.create_publisher(String, "/nexus/pi/camera_health", 5)
        self.metadata_pub = self.create_publisher(String, "/nexus/camera/imx219/metadata", 5)
        self.create_timer(max(0.001, float(self.get_parameter("poll_period_ms").value) / 1000), self.poll)
        self.create_timer(1.0, self.publish_health)

    def destroy_node(self):
        if self.socket is not None:
            self.socket.close()
        return super().destroy_node()

    def poll(self):
        if self.transport == "tcp_client":
            self.poll_tcp()
            return
        try:
            metadata = json.loads((self.camera_dir / "latest.json").read_text(encoding="utf-8"))
            if metadata.get("sample_timestamp_domain") != "ros_unix_ns_receive":
                return
            stamp = int(metadata["captured_unix_ns"])
            if stamp <= self.last_stamp:
                return
            self.publish_frame(metadata, (self.camera_dir / metadata["image"]).read_bytes())
        except (FileNotFoundError, json.JSONDecodeError, KeyError, OSError, ValueError):
            return

    def connect_tcp(self):
        now = time.time_ns()
        if self.socket is not None or now < self.next_connect_ns:
            return
        try:
            self.socket = socket.create_connection((str(self.get_parameter("remote_host").value),
                                                    int(self.get_parameter("remote_port").value)), timeout=0.2)
            self.socket.setblocking(False)
            self.buffer = b""
        except OSError:
            self.socket = None
            self.next_connect_ns = now + 1_000_000_000

    def poll_tcp(self):
        self.connect_tcp()
        if self.socket is None:
            return
        try:
            for _ in range(4):
                chunk = self.socket.recv(1_048_576)
                if not chunk:
                    raise ConnectionResetError("camera stream closed")
                self.buffer += chunk
                while len(self.buffer) >= 8:
                    metadata_size, image_size = struct.unpack("!II", self.buffer[:8])
                    if metadata_size > 65536 or image_size > 16_000_000:
                        raise ValueError("invalid camera frame sizes")
                    total = 8 + metadata_size + image_size
                    if len(self.buffer) < total:
                        break
                    metadata = json.loads(self.buffer[8:8 + metadata_size].decode("utf-8"))
                    image = self.buffer[8 + metadata_size:total]
                    self.buffer = self.buffer[total:]
                    self.publish_frame(metadata, image)
            return
        except BlockingIOError:
            return
        except (OSError, ValueError, json.JSONDecodeError, UnicodeDecodeError):
            self.invalid += 1
            self.socket.close()
            self.socket = None
            self.next_connect_ns = time.time_ns() + 1_000_000_000

    def publish_frame(self, metadata, encoded):
        if metadata.get("sample_timestamp_domain") != "ros_unix_ns_receive":
            raise ValueError("unverified camera timestamp")
        source_stamp = int(metadata["captured_unix_ns"])
        if source_stamp <= self.last_stamp:
            return
        from PIL import Image as PilImage
        rgb = PilImage.open(io.BytesIO(encoded)).convert("RGB")
        width, height = rgb.size
        if width != int(metadata["width"]) or height != int(metadata["height"]):
            raise ValueError("camera metadata/image size mismatch")
        stamp = time.time_ns() if self.transport == "tcp_client" else source_stamp
        self.last_stamp = source_stamp
        sec, nanosec = divmod(stamp, 1_000_000_000)
        raw = Image()
        raw.header.stamp.sec, raw.header.stamp.nanosec = sec, nanosec
        raw.header.frame_id = "camera_optical_frame"
        raw.height, raw.width, raw.encoding, raw.is_bigendian = height, width, "rgb8", 0
        raw.step, raw.data = width * 3, rgb.tobytes()
        compressed = CompressedImage()
        compressed.header = raw.header
        compressed.format, compressed.data = "jpeg", encoded
        info = CameraInfo()
        info.header = raw.header
        info.height, info.width = height, width
        self.raw_pub.publish(raw)
        self.compressed_pub.publish(compressed)
        self.info_pub.publish(info)
        self.metadata_pub.publish(String(data=json.dumps({
            "schema_version": 1,
            "ros_sample_timestamp_ns": stamp,
            "ros_timestamp_domain": "ros_unix_ns_edge_receive" if self.transport == "tcp_client"
                                    else "ros_unix_ns_receive",
            "pi_receive_timestamp_ns": source_stamp,
            "sensor_timestamp_ns": metadata.get("sensor_timestamp_ns"),
            "sensor_timestamp_domain": "libcamera_sensor_monotonic_unaligned",
            "width": width, "height": height,
        }, allow_nan=False)))
        self.frames += 1
        if time.time_ns() - self.last_health_publish_ns >= 1_000_000_000:
            self.publish_health()

    def publish_health(self):
        now = time.time_ns()
        self.last_health_publish_ns = now
        elapsed = max((now - self.started_ns) / 1e9, 1e-6)
        self.health_pub.publish(String(data=json.dumps({
            "schema_version": 1, "valid": self.frames > 0,
            "frames": self.frames, "invalid_frames": self.invalid,
            "average_fps": self.frames / elapsed,
            "last_source_timestamp_ns": self.last_stamp or None,
            "transport": self.transport,
        }, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CameraFileNode()
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
