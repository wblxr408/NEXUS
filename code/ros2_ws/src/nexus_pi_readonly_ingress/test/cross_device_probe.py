#!/usr/bin/env python3
"""Observe the three telemetry topics during a Pi-to-edge check."""

import json
import time

import rclpy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("nexus_pi_cross_device_probe")
    counts = {"imu": 0, "uwb": 0, "health": 0}
    samples = {}

    def imu(message):
        counts["imu"] += 1
        samples["imu_z_mps2"] = message.linear_acceleration.z
        samples["imu_stamp_ns"] = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def uwb(message):
        counts["uwb"] += 1
        samples["uwb"] = json.loads(message.data)

    def health(message):
        counts["health"] += 1
        samples["health"] = json.loads(message.data)

    node.create_subscription(Imu, "/nexus/fcu/imu", imu, 100)
    node.create_subscription(String, "/nexus/uwb/ranges", uwb, 20)
    node.create_subscription(String, "/nexus/pi/telemetry_health", health, 10)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and not all(counts.values()):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({"counts": counts, "samples": samples}, allow_nan=False))
    valid = (all(counts.values()) and samples.get("uwb", {}).get("tag_id") == 2
             and samples.get("imu_z_mps2") is not None)
    raise SystemExit(0 if valid else 1)


if __name__ == "__main__":
    main()
