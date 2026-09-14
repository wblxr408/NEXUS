"""Bounded live check of sensor timestamps in ROS; saves metadata only."""

import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String

from nexus_pi_readonly_ingress.camera_file_node import CameraFileNode


def main():
    rclpy.init(args=["--ros-args", "-p", "clock_sync:=true"])
    node = CameraFileNode()
    assert node.get_parameter("clock_sync").type_ == Parameter.Type.BOOL
    received = []
    node.create_subscription(String, "/nexus/camera/imx219/metadata",
                             lambda message: received.append(json.loads(message.data)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + 60
    try:
        while time.monotonic() < deadline and len(received) < 30:
            executor.spin_once(timeout_sec=.1)
    finally:
        print(f"frames={node.frames} invalid={node.invalid} missing={node.estimated_missing_frames}", flush=True)
        with Path(sys.argv[1]).open("x") as stream:
            json.dump(received, stream, indent=2)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    assert len(received) >= 20, len(received)
    for sample in received:
        assert sample["ros_timestamp_domain"] == "ros_unix_ns_aligned_sensor"
        assert sample["sensor_timestamp_domain"] == "CLOCK_BOOTTIME"
        assert sample["ros_sample_timestamp_ns"] == sample["sensor_timestamp_ns"] + sample["clock_alignment"]["offset_ns"]
        assert 0 < sample["transport_latency_ms"] < 2000
    stamps = [s["ros_sample_timestamp_ns"] for s in received]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    print(f"LIVE_SENSOR_CLOCK_ROS_PASS frames={len(received)}")


if __name__ == "__main__":
    main()
