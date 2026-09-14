#!/usr/bin/env python3
"""Publish atomically written IMX219 v2 files as ROS Image/CameraInfo."""

from array import array
import io
import json
import socket
import struct
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Int16, String

from nexus_pi_readonly_ingress.time_alignment import BootTimeAligner
from nexus_pi_readonly_ingress.camera_clock import CameraClockClient
from nexus_pi_readonly_ingress.camera_intrinsics import CameraIntrinsics


class CameraFileNode(Node):
    def __init__(self):
        super().__init__("nexus_pi_camera_file_ingress")
        self.declare_parameter("camera_dir", "")
        self.declare_parameter("poll_period_ms", 5.0)
        self.declare_parameter("transport", "tcp_client")
        self.declare_parameter("remote_host", "192.168.1.143")
        self.declare_parameter("remote_port", 14552)
        self.declare_parameter("clock_sync", False)
        self.declare_parameter("clock_port", 14553)
        self.declare_parameter("activation_command_topic", "")
        self.declare_parameter("start_command", 3)
        self.declare_parameter("stop_commands", [2, 4])
        self.clock = CameraClockClient(str(self.get_parameter("remote_host").value),
                                       int(self.get_parameter("clock_port").value))
        self.clock_sync = bool(self.get_parameter("clock_sync").value)
        if self.clock_sync:
            self.create_timer(1.0, self.probe_clock)
        self.camera_dir = Path(str(self.get_parameter("camera_dir").value))
        self.transport = str(self.get_parameter("transport").value)
        if self.transport not in {"file", "tcp_client"}:
            raise ValueError("transport must be file or tcp_client")
        self.declare_parameter("intrinsics_file", "")
        intrinsics_file = str(self.get_parameter("intrinsics_file").value)
        self.intrinsics = CameraIntrinsics(intrinsics_file) if intrinsics_file else None
        self.last_sensor_key = None
        self.last_stamp = 0
        self.socket = None
        self.buffer = b""
        self.next_connect_ns = 0
        activation_topic = str(self.get_parameter("activation_command_topic").value)
        self.start_command = int(self.get_parameter("start_command").value)
        self.stop_commands = {int(value) for value in self.get_parameter("stop_commands").value}
        self.capture_enabled = not bool(activation_topic)
        self.command_sub = None
        if activation_topic:
            self.command_sub = self.create_subscription(
                Int16, activation_topic, self.command_callback, 10)
        self.frames = self.invalid = 0
        self.last_sequence = None
        self.estimated_missing_frames = 0
        self.sequence_resets = 0
        self.last_receive_ns = 0
        self.last_transport_latency_ms = None
        self.pi_clock_aligner = BootTimeAligner(reset_threshold_ns=1_000_000_000)
        self.started_ns = time.time_ns()
        self.last_health_publish_ns = 0
        self.raw_pub = self.create_publisher(Image, "/nexus/camera/imx219/image_raw", 5)
        self.compressed_pub = self.create_publisher(CompressedImage, "/nexus/camera/imx219/image_raw/compressed", 5)
        self.rect_pub = self.create_publisher(Image, "/camera/image_rect", 5)
        self.rect_info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 5)
        self.info_pub = self.create_publisher(CameraInfo, "/nexus/camera/imx219/camera_info", 5)
        self.health_pub = self.create_publisher(String, "/nexus/pi/camera_health", 5)
        self.metadata_pub = self.create_publisher(String, "/nexus/camera/imx219/metadata", 5)
        self.create_timer(max(0.001, float(self.get_parameter("poll_period_ms").value) / 1000), self.poll)
        self.create_timer(1.0, self.publish_health)

    def command_callback(self, message):
        command = int(message.data)
        if command == self.start_command:
            if not self.capture_enabled:
                self.capture_enabled = True
                self.next_connect_ns = 0
                self.get_logger().info("收到起飞命令，开始接收 IMX219 视频流")
            return
        if command in self.stop_commands:
            if self.capture_enabled:
                self.capture_enabled = False
                self.buffer = b""
                if self.socket is not None:
                    self.socket.close()
                    self.socket = None
                self.get_logger().info("收到降落/上锁命令，停止接收 IMX219 视频流")

    def probe_clock(self):
        try:
            self.clock.probe()
        except (OSError, ValueError, KeyError):
            return

    def destroy_node(self):
        if self.socket is not None:
            self.socket.close()
        return super().destroy_node()

    def poll(self):
        if not self.capture_enabled:
            return
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
        latest = None
        try:
            for _ in range(16):
                try:
                    chunk = self.socket.recv(1_048_576)
                except BlockingIOError:
                    break
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
                    latest = (metadata, image)
            # Decode at most one image per timer tick so the clock probe and ROS
            # executor can run even when the camera produces frames faster.
            if latest is not None:
                self.publish_frame(*latest)
        except (OSError, ValueError, KeyError, json.JSONDecodeError, UnicodeDecodeError):
            self.invalid += 1
            self.socket.close()
            self.socket = None
            self.next_connect_ns = time.time_ns() + 1_000_000_000

    def publish_frame(self, metadata, encoded):
        if metadata.get("sample_timestamp_domain") != "ros_unix_ns_receive":
            raise ValueError("unverified camera timestamp")
        source_stamp = int(metadata["captured_unix_ns"])
        sensor_key = (metadata.get("boot_id"), metadata.get("sensor_timestamp_ns"))
        if self.clock_sync:
            if self.last_sensor_key == sensor_key:
                return
        elif source_stamp <= self.last_stamp:
            return
        sequence = metadata.get("frame_sequence")
        if sequence is not None:
            sequence = int(sequence)
            if sequence < 0:
                raise ValueError("negative camera frame sequence")
            if self.last_sequence is not None:
                if sequence <= self.last_sequence:
                    self.sequence_resets += 1
                elif sequence > self.last_sequence + 1:
                    self.estimated_missing_frames += sequence - self.last_sequence - 1
            self.last_sequence = sequence
        from PIL import Image as PilImage
        rgb = PilImage.open(io.BytesIO(encoded)).convert("RGB")
        width, height = rgb.size
        if width != int(metadata["width"]) or height != int(metadata["height"]):
            raise ValueError("camera metadata/image size mismatch")
        arrival = time.time_ns()
        clock_metadata = None
        if self.clock_sync:
            if metadata.get("sensor_timestamp_domain") != "CLOCK_BOOTTIME":
                raise ValueError("camera does not supply the verified sensor clock")
            try:
                stamp, clock_metadata = self.clock.align(metadata.get("sensor_timestamp_ns"), metadata.get("boot_id"))
            except ValueError:
                self.invalid += 1
                return
            aligned_source_stamp = None
            self.last_transport_latency_ms = (arrival - stamp) / 1e6
        elif self.transport == "tcp_client":
            aligned_source_stamp = self.pi_clock_aligner.align_ns(source_stamp, arrival)
            stamp = arrival
            self.last_transport_latency_ms = max(0.0, (arrival - aligned_source_stamp) / 1e6)
        else:
            aligned_source_stamp = source_stamp
            stamp = source_stamp
            self.last_transport_latency_ms = 0.0
        self.last_receive_ns = arrival
        self.last_stamp = source_stamp
        self.last_sensor_key = sensor_key
        sec, nanosec = divmod(stamp, 1_000_000_000)
        raw = Image()
        raw.header.stamp.sec, raw.header.stamp.nanosec = sec, nanosec
        raw.header.frame_id = "camera_optical_frame"
        raw.height, raw.width, raw.encoding, raw.is_bigendian = height, width, "rgb8", 0
        raw.step, raw.data = width * 3, array("B", rgb.tobytes())
        compressed = CompressedImage()
        compressed.header = raw.header
        compressed.format, compressed.data = "jpeg", array("B", encoded)
        info = CameraInfo()
        info.header = raw.header
        info.height, info.width = height, width
        if self.intrinsics:
            import numpy as np
            self.intrinsics.check_mode(width, height, metadata.get("rotation_deg"))
            info.distortion_model = "plumb_bob"
            info.k = self.intrinsics.matrix.reshape(-1).tolist()
            info.d = self.intrinsics.distortion.tolist()
            info.r = np.eye(3).reshape(-1).tolist()
            info.p = np.column_stack((self.intrinsics.matrix, np.zeros(3))).reshape(-1).tolist()
            rectified = self.intrinsics.crop_rectified(self.intrinsics.rectify(np.asarray(rgb)))
            rect = Image()
            rect.header = raw.header
            rect.height, rect.width, rect.encoding, rect.is_bigendian = (*rectified.shape[:2], "rgb8", 0)
            rect.step = rect.width * 3
            rect.data = array("B", rectified.tobytes())
            rect_info = CameraInfo()
            rect_info.header = raw.header
            rect_info.height, rect_info.width = rect.height, rect.width
            rect_info.distortion_model = "plumb_bob"
            rect_info.k = self.intrinsics.rectified_matrix.reshape(-1).tolist()
            rect_info.r = np.eye(3).reshape(-1).tolist()
            rect_info.p = np.column_stack((self.intrinsics.rectified_matrix,
                                           np.zeros(3))).reshape(-1).tolist()
            rect_info.d = [0.0] * 5
            self.rect_pub.publish(rect)
            self.rect_info_pub.publish(rect_info)
        self.raw_pub.publish(raw)
        self.compressed_pub.publish(compressed)
        self.info_pub.publish(info)
        self.metadata_pub.publish(String(data=json.dumps({
            "schema_version": 1,
            "ros_sample_timestamp_ns": stamp,
            "ros_timestamp_domain": ("ros_unix_ns_aligned_sensor" if self.clock_sync else
                                     ("ros_unix_ns_edge_receive" if self.transport == "tcp_client"
                                      else "ros_unix_ns_receive")),
            "clock_alignment": clock_metadata,
            "pi_receive_timestamp_ns": source_stamp,
            "aligned_pi_receive_timestamp_ns": aligned_source_stamp,
            "sensor_timestamp_ns": metadata.get("sensor_timestamp_ns"),
            "sensor_timestamp_domain": metadata.get("sensor_timestamp_domain", "libcamera_sensor_monotonic_unaligned"),
            # Preserve per-frame exposure/readout timing for downstream EIS.
            # The ingress never fabricates a readout value when libcamera did
            # not provide one.
            "exposure_time_us": metadata.get("exposure_time_us"),
            "rolling_shutter_readout_us": metadata.get("rolling_shutter_readout_us"),
            "frame_sequence": sequence,
            "estimated_missing_frames": self.estimated_missing_frames,
            "transport_latency_ms": self.last_transport_latency_ms,
            "width": width, "height": height,
            "valid_roi_full_image_px": (list(self.intrinsics.roi_xyxy)
                                         if self.intrinsics else None),
        }, allow_nan=False)))
        self.frames += 1
        if time.time_ns() - self.last_health_publish_ns >= 1_000_000_000:
            self.publish_health()

    def publish_health(self):
        now = time.time_ns()
        self.last_health_publish_ns = now
        elapsed = max((now - self.started_ns) / 1e9, 1e-6)
        age_ms = None if not self.last_receive_ns else (now - self.last_receive_ns) / 1e6
        self.health_pub.publish(String(data=json.dumps({
            "schema_version": 1,
            "valid": bool(self.frames > 0 and age_ms is not None and age_ms < 3000.0),
            "frames": self.frames, "invalid_frames": self.invalid,
            "average_fps": self.frames / elapsed,
            "estimated_missing_frames": self.estimated_missing_frames,
            "sequence_resets": self.sequence_resets,
            "last_receive_age_ms": age_ms,
            "last_transport_latency_ms": self.last_transport_latency_ms,
            "pi_clock_resets": self.pi_clock_aligner.reset_count,
            "last_source_timestamp_ns": self.last_stamp or None,
            "transport": self.transport,
            "capture_enabled": self.capture_enabled,
        }, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CameraFileNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
