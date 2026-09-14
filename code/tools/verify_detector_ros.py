"""Replay a held-out image through the actual ROS detector using the trained CUDA model."""

import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from std_msgs.msg import String

from nexus_vision_localization.detection_node import ObjectDetectionNode


def main():
    root = Path(sys.argv[1]).resolve()
    cv2.setNumThreads(4)
    rclpy.init(args=["--ros-args", "-p", "model_manifest:=" + str(root / "detector/detector_manifest.json")])
    node = ObjectDetectionNode()
    received = []
    node.create_subscription(String, "/nexus/vision/detections",
                             lambda msg: received.append(json.loads(msg.data)), 20)
    reference = Path(sys.argv[2]) if len(sys.argv) > 2 else root / "parity/reference_00.npz"
    image = np.load(reference)["image"]
    assert node.model.network.device.type == "cuda"
    node.model.infer(image)
    try:
        for _ in range(3):
            message = node._bridge.cv2_to_imgmsg(image, encoding="bgr8")
            message.header.frame_id = "camera_optical_frame"
            message.header.stamp = node.get_clock().now().to_msg()
            node._image_callback(message)
            node._drain()
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
        (root / "detector_ros.json").write_text(json.dumps(received, indent=2) + "\n")
        assert received
        assert all(x["valid"] and len(x["detections"]) == 3 for x in received), received
        print(f"TRAINED_DETECTOR_ROS_GPU_PASS packets={len(received)} gpu={node.model.network.device_name}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
