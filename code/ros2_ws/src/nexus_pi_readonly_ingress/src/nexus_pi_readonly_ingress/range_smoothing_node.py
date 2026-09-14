#!/usr/bin/env python3
"""Publish rolling robust UWB means alongside the original live stream."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nexus_pi_readonly_ingress.robust_range_window import RobustRangeWindow


class RangeSmoothingNode(Node):
    def __init__(self):
        super().__init__('nexus_uwb_range_smoothing')
        self.declare_parameter('input_topic', '/nexus/uwb/ranges')
        self.declare_parameter('output_topic', '/nexus/uwb/ranges_smoothed')
        self.declare_parameter('window_size', 200)
        self.declare_parameter('stale_timeout_s', 5.0)
        source = self.get_parameter('input_topic').value
        target = self.get_parameter('output_topic').value
        if source == target:
            raise ValueError('input and output topics must differ')
        self.window = RobustRangeWindow(self.get_parameter('window_size').value,
                                        self.get_parameter('stale_timeout_s').value)
        self.publisher = self.create_publisher(String, target, 20)
        self.create_subscription(String, source, self.receive, 20)
        self.last_result = None
        self.create_timer(1.0, self.check_stale)

    def receive(self, message):
        try:
            result = self.window.push(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f'Invalid UWB packet: {error}')
            return
        if result is not None:
            self.last_result = result
            self.publish(result)

    def publish(self, result):
        self.publisher.publish(String(data=json.dumps(result, allow_nan=False)))

    def check_stale(self):
        if (self.last_result is not None and not self.last_result['stale'] and
                time.monotonic() - self.window.last_arrival > self.window.stale_s):
            self.last_result = dict(self.last_result, stale=True, ready=False,
                                    ranges_m=[None] * len(self.last_result['ranges_m']))
            self.publish(self.last_result)


def main():
    rclpy.init()
    node = RangeSmoothingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
