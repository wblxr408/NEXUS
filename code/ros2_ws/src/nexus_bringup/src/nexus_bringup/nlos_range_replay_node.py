#!/usr/bin/env python3
"""Replay labelled UWB range JSONL with a fresh ROS-time base; no flight control."""

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NlosRangeReplayNode(Node):
    def __init__(self):
        super().__init__("nexus_nlos_range_replay")
        self.declare_parameter("packet_file", "")
        self.declare_parameter("topic", "/nexus/uwb/ranges")
        self.declare_parameter("speed", 1.)
        path = Path(str(self.get_parameter("packet_file").value))
        speed = float(self.get_parameter("speed").value)
        if not path.is_file() or not speed > 0:
            raise ValueError("packet_file must exist and speed must be positive")
        self._packets = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if not self._packets:
            raise ValueError("packet file is empty")
        stamps = [self._validate(packet) for packet in self._packets]
        if any(right <= left for left, right in zip(stamps, stamps[1:])):
            raise ValueError("packet timestamps must strictly increase")
        self._publisher = self.create_publisher(String, str(self.get_parameter("topic").value), 20)
        self._speed, self._index = speed, 0
        self._source_start_ns = stamps[0]
        self._replay_start_ns = self.get_clock().now().nanoseconds + 50_000_000
        self._timer = self.create_timer(.002, self._tick)

    @staticmethod
    def _validate(packet):
        if (not isinstance(packet, dict) or packet.get("schema_version") != 1 or packet.get("frame_id") != "map"
                or packet.get("unit") != "m" or not isinstance(packet.get("sample_timestamp_ns"), int)
                or len(packet.get("anchor_ids", [])) != 4 or len(packet.get("ranges_m", [])) != 4
                or len(packet.get("variances_m2", [])) != 4):
            raise ValueError("invalid replay range packet")
        return packet["sample_timestamp_ns"]

    def _tick(self):
        if self._index >= len(self._packets):
            self.get_logger().info("NLOS range replay complete")
            self._timer.cancel()
            return
        packet = dict(self._packets[self._index])
        due = self._replay_start_ns + int((packet["sample_timestamp_ns"] - self._source_start_ns) / self._speed)
        if self.get_clock().now().nanoseconds < due:
            return
        packet["sample_timestamp_ns"] = due
        self._publisher.publish(String(data=json.dumps(packet, allow_nan=False)))
        self._index += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NlosRangeReplayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
