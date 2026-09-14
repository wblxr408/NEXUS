#!/usr/bin/env python3
"""Read-only CDR forwarding of existing ROS IMU/ranges; never opens FC serial."""
import queue
import socket
import struct
import threading
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('nexus_fcu_ros_stream')
    outgoing = queue.Queue(maxsize=400)
    stop = threading.Event()
    connected = threading.Event()
    overflow = threading.Event()
    def enqueue(kind):
        def receive(message):
            if not connected.is_set():
                return
            payload = bytes([kind]) + serialize_message(message)
            try:
                outgoing.put_nowait(struct.pack('!I', len(payload)) + payload)
            except queue.Full:
                overflow.set()
        return receive
    node.create_subscription(Imu, '/imu_global_001', enqueue(1), QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(String, '/nexus/uwb/ranges', enqueue(2), QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Odometry, '/odom_global_001', enqueue(3), QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    def serve():
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', 14554))
            server.listen(1)
            server.settimeout(.2)
            while not stop.is_set():
                try:
                    client, _ = server.accept()
                except socket.timeout:
                    continue
                with client:
                    client.settimeout(.5)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    while not outgoing.empty():
                        outgoing.get_nowait()
                    overflow.clear()
                    connected.set()
                    try:
                        while not stop.is_set() and not overflow.is_set():
                            try:
                                packet = outgoing.get(timeout=.2)
                            except queue.Empty:
                                continue
                            client.sendall(packet)
                    except OSError:
                        pass
                    finally:
                        connected.clear()
    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    finally:
        stop.set()
        thread.join(timeout=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
