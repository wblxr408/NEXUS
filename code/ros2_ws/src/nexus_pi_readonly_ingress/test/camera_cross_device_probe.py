#!/usr/bin/env python3
import json
import time

import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("nexus_pi_camera_cross_device_probe")
    counts = {"raw": 0, "compressed": 0, "info": 0, "metadata": 0, "health": 0}
    sample, raw_stamps, info_stamps = {}, set(), set()

    def raw(message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        counts["raw"] += 1
        sample.update(width=message.width, height=message.height, encoding=message.encoding,
                      stamp_ns=stamp)
        raw_stamps.add(stamp)

    node.create_subscription(Image, "/nexus/camera/imx219/image_raw", raw, 5)
    node.create_subscription(CompressedImage, "/nexus/camera/imx219/image_raw/compressed",
                             lambda _: counts.__setitem__("compressed", counts["compressed"] + 1), 5)
    def camera_info(message):
        counts["info"] += 1
        info_stamps.add(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)
    node.create_subscription(CameraInfo, "/nexus/camera/imx219/camera_info", camera_info, 5)

    def metadata(message):
        counts["metadata"] += 1
        sample["metadata"] = json.loads(message.data)
    node.create_subscription(String, "/nexus/camera/imx219/metadata", metadata, 5)
    node.create_subscription(String, "/nexus/pi/camera_health",
                             lambda m: (counts.__setitem__("health", counts["health"] + 1),
                                        sample.update(health=json.loads(m.data))), 5)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and (not all(counts.values()) or not (raw_stamps & info_stamps)):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({"counts": counts, "sample": sample}, allow_nan=False))
    valid = (all(counts.values()) and sample.get("width") == 1640
             and sample.get("height") == 1232 and sample.get("encoding") == "rgb8"
             and sample.get("stamp_ns", 0) > 0
             and sample.get("metadata", {}).get("sensor_timestamp_ns") is not None
             and sample.get("metadata", {}).get("frame_sequence") is not None
             and sample.get("metadata", {}).get("transport_latency_ms") is not None
             and sample.get("health", {}).get("valid") is True
             and sample.get("health", {}).get("last_receive_age_ms") is not None
             and bool(raw_stamps & info_stamps))
    raise SystemExit(0 if valid else 1)


if __name__ == "__main__":
    main()
