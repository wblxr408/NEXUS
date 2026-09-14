"""Measured epoch conversion preserves sensor values and rejects stale synchronization."""
import json
import time
import pytest
import rclpy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nexus_pi_readonly_ingress.fcu_clock_ingress_node import FcuClockIngressNode


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_epoch_alignment_preserves_values_and_source_stamp():
    rclpy.init()
    node = FcuClockIngressNode()
    try:
        node.imu_pub, node.range_pub = Sink(), Sink()
        now = time.time_ns()
        node.clock.boot_id = 'test_boot'
        node.clock.samples = [dict(offset_ns=-6_000_000_000, rtt_ns=2_000_000,
            edge_unix_ns=now, monotonic_ns=time.monotonic_ns(), boot_id='test_boot')]
        imu = Imu()
        source = now + 6_000_000_000
        imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(source, 10**9)
        imu.header.frame_id = 'scaled_imu'
        imu.linear_acceleration.z = 9.81
        imu.angular_velocity.x = .03
        node.imu(imu)
        result = node.imu_pub.messages[0]
        assert result.header.stamp.sec*10**9+result.header.stamp.nanosec == now
        assert imu.header.stamp.sec*10**9+imu.header.stamp.nanosec == source
        assert result.linear_acceleration == imu.linear_acceleration
        assert result.angular_velocity == imu.angular_velocity
        node.odom_pub = Sink()
        odom = Odometry()
        odom.header = imu.header
        odom.pose.pose.orientation.z = .5
        odom.pose.pose.orientation.w = .8660254038
        node.odom(odom)
        aligned_odom = node.odom_pub.messages[0]
        assert aligned_odom.header.stamp.sec*10**9+aligned_odom.header.stamp.nanosec == now
        assert aligned_odom.pose == odom.pose
        assert odom.header.stamp.sec*10**9+odom.header.stamp.nanosec == source
        node.ranges(String(data=json.dumps(dict(sample_timestamp_ns=source, ranges_m=[1,2,3,4]))))
        packet = json.loads(node.range_pub.messages[0].data)
        assert packet['sample_timestamp_ns'] == now
        assert packet['source_sample_timestamp_ns'] == source
        assert packet['ranges_m'] == [1,2,3,4]
        node.clock.samples[0]['offset_ns'] -= 10_000_000
        next_stamp, metadata = node.aligned(source + 5_000_000)
        assert next_stamp > now
        assert metadata['applied_offset_ns'] > -6_010_000_000
        node.clock.samples[0]['monotonic_ns'] -= 6_000_000_000
        node.imu(imu)
        assert len(node.imu_pub.messages) == 1
        node.clock.samples = []
        with pytest.raises(ValueError):
            node.aligned(source)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_tcp_split_frames_preserve_ros_payloads():
    import struct
    from rclpy.serialization import serialize_message
    class Connection:
        def __init__(self, chunks):
            self.chunks = iter(chunks)
            self.closed = False
        def recv(self, size):
            item = next(self.chunks, None)
            if item is None:
                raise BlockingIOError()
            return item
        def close(self):
            self.closed = True
    rclpy.init()
    node = FcuClockIngressNode()
    try:
        received = []
        node.imu = lambda m: received.append(m)
        node.ranges = lambda m: received.append(m)
        imu = Imu()
        imu.linear_acceleration.z = 9.81
        ranges = String(data='{"ranges_m":[1,2,3,4]}')
        def frame(kind, m):
            payload = bytes([kind]) + serialize_message(m)
            return struct.pack('!I',len(payload)) + payload
        stream = frame(1,imu) + frame(2,ranges)
        node.socket = Connection([stream[:2], None, stream[2:]])
        node.poll_tcp()
        assert not received
        node.poll_tcp()
        assert received == [imu,ranges]
        node.socket = Connection([struct.pack('!I',1_000_001)])
        broken = node.socket
        node.poll_tcp()
        assert broken.closed and node.socket is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
