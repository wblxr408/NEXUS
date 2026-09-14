#!/usr/bin/env python3
"""Map Pi ROS Unix stamps to ground Unix using measured four-timestamp exchange."""
import copy
import json
import socket
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nexus_pi_readonly_ingress.camera_clock import CameraClockClient


class FcuClockIngressNode(Node):
    def __init__(self):
        super().__init__('nexus_fcu_clock_ingress')
        self.declare_parameter('remote_host', '192.168.1.143')
        self.declare_parameter('transport', 'dds')
        self.transport = self.get_parameter('transport').value
        if self.transport not in {'dds', 'tcp'}:
            raise ValueError('unsupported FC ingress transport')
        self.socket = None
        self.buffer = b''
        self.next_connect = 0
        self.received_imu = self.published_imu = self.received_ranges = self.published_ranges = 0
        self.last_input_error = None
        self.applied_offset = None
        self.offset_boot_id = None
        self.offset_update_ns = 0
        self.clock = CameraClockClient(self.get_parameter('remote_host').value, clock='CLOCK_REALTIME')
        self.raw_imu_pub = self.create_publisher(Imu, '/nexus/pi/imu_raw', 100)
        self.raw_range_pub = self.create_publisher(String, '/nexus/pi/ranges_raw', 20)
        self.raw_odom_pub = self.create_publisher(Odometry, '/nexus/pi/odom_raw', 100)
        self.odom_pub = self.create_publisher(Odometry, '/nexus/pi/odom_aligned', 100)
        self.imu_pub = self.create_publisher(Imu, '/nexus/pi/imu_aligned', 100)
        self.range_pub = self.create_publisher(String, '/nexus/pi/ranges_aligned', 20)
        self.status_pub = self.create_publisher(String, '/nexus/pi/fcu_clock_status', 10)
        if self.transport == 'dds':
            self.create_subscription(Odometry, '/odom_global_001', self.odom, 100)
            self.create_subscription(Imu, '/imu_global_001', self.imu, qos_profile_sensor_data)
            self.create_subscription(String, '/nexus/uwb/ranges', self.ranges, 20)
        else:
            self.create_timer(.005, self.poll_tcp)
        self.create_timer(1., self.probe)

    def poll_tcp(self):
        try:
            if self.socket is None:
                if time.monotonic() < self.next_connect:
                    return
                self.socket = socket.create_connection((self.clock.address[0], 14554), timeout=.15)
                self.socket.setblocking(False)
                self.buffer = b''
            for _ in range(8):
                try:
                    data = self.socket.recv(65536)
                except BlockingIOError:
                    break
                if not data:
                    raise ConnectionError('FC sensor stream disconnected')
                self.buffer += data
            while len(self.buffer) >= 4:
                size = struct.unpack('!I', self.buffer[:4])[0]
                if not 1 < size < 1_000_000:
                    raise ValueError('invalid FC sensor frame length')
                if len(self.buffer) < size + 4:
                    break
                kind, payload = self.buffer[4], self.buffer[5:4+size]
                self.buffer = self.buffer[4+size:]
                if kind == 1:
                    self.imu(deserialize_message(payload, Imu))
                elif kind == 2:
                    self.ranges(deserialize_message(payload, String))
                elif kind == 3:
                    self.odom(deserialize_message(payload, Odometry))
                else:
                    raise ValueError('unknown FC sensor frame type')
        except (OSError, ValueError, RuntimeError) as error:
            if self.socket is not None:
                self.socket.close()
            self.socket = None
            self.buffer = b''
            self.next_connect = time.monotonic() + 1
            self.status_pub.publish(String(data=json.dumps(dict(valid=False, reason=str(error)))))

    def probe(self):
        try:
            sample = self.clock.probe()
            self.status_pub.publish(String(data=json.dumps(dict(valid=True, received_imu=self.received_imu, published_imu=self.published_imu,
                received_ranges=self.received_ranges, published_ranges=self.published_ranges,
                last_input_error=self.last_input_error, applied_offset_ns=self.applied_offset, **sample))))
        except (OSError, ValueError, KeyError) as error:
            self.status_pub.publish(String(data=json.dumps(dict(valid=False, reason=str(error)))))

    def aligned(self, stamp):
        value, metadata = self.clock.align(stamp, self.clock.boot_id)
        if metadata['rtt_ns'] > 100_000_000:
            raise ValueError('clock exchange uncertainty exceeds 50 ms')
        now = time.monotonic_ns()
        target = metadata['offset_ns']
        if self.applied_offset is None or self.offset_boot_id != self.clock.boot_id:
            self.applied_offset = target
            self.offset_boot_id = self.clock.boot_id
        else:
            # Smooth network-asymmetry jitter; never create backwards IMU time
            # by applying a new clock probe as an instantaneous epoch step.
            change = target - self.applied_offset
            if abs(change) > 100_000_000:
                self.applied_offset = None
                raise ValueError('Pi clock epoch changed; resynchronizing')
            limit = max(1, (now - self.offset_update_ns) // 1000)
            self.applied_offset += max(-limit, min(limit, change))
        self.offset_update_ns = now
        metadata['applied_offset_ns'] = self.applied_offset
        return stamp + self.applied_offset, metadata

    def imu(self, msg):
        self.received_imu += 1
        self.raw_imu_pub.publish(msg)
        try:
            source = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            stamp, _ = self.aligned(source)
            result = copy.deepcopy(msg)
            result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 10**9)
            self.imu_pub.publish(result)
            self.published_imu += 1
        except (ValueError, TypeError, KeyError) as error:
            self.last_input_error = str(error)
            return

    def odom(self, msg):
        self.raw_odom_pub.publish(msg)
        try:
            source = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            stamp, _ = self.aligned(source)
            result = copy.deepcopy(msg)
            result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 10**9)
            self.odom_pub.publish(result)
        except (ValueError, TypeError, KeyError) as error:
            self.last_input_error = str(error)

    def ranges(self, msg):
        self.received_ranges += 1
        self.raw_range_pub.publish(msg)
        try:
            packet = json.loads(msg.data)
            source = packet['sample_timestamp_ns']
            stamp, metadata = self.aligned(source)
            packet.update(sample_timestamp_ns=stamp, source_sample_timestamp_ns=source,
                          timestamp_domain='ground_unix_ns_aligned_pi_ros', clock_alignment=metadata)
            self.range_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
            self.published_ranges += 1
        except (ValueError, TypeError, KeyError) as error:
            self.last_input_error = str(error)
            return


def main():
    rclpy.init()
    node = FcuClockIngressNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
