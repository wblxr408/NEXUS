#!/usr/bin/env python3
"""Same-sample detection packets for platform masks and external target discovery."""

import json
from time import perf_counter

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from nexus_vision_localization.object_detector import ObjectDetectorOnnx


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("nexus_object_detection")
        for name, default in {"model_manifest": "", "image_topic": "/camera/image_rect", "max_age_ms": 300.,
                              "confidence_threshold": .35, "iou_threshold": .5, "maximum_detections": 100,
                              "static_class_ids": ""}.items():
            self.declare_parameter(name, default)
        self.model = ObjectDetectorOnnx(self.get_parameter("model_manifest").value)
        self._max_age_ns = int(self.get_parameter("max_age_ms").value * 1e6)
        if self._max_age_ns <= 0:
            raise ValueError("detection max_age_ms must be positive")
        self._settings = {name: self.get_parameter(name).value
                          for name in ("confidence_threshold", "iou_threshold", "maximum_detections")}
        self._static_classes = {int(value.strip()) for value in self.get_parameter("static_class_ids").value.split(",") if value.strip()}
        if any(value < 0 or value >= len(self.model.manifest["class_names"]) for value in self._static_classes):
            raise ValueError("static_class_ids must refer to declared model classes")
        self._bridge = CvBridge()
        self._pending, self._last_seen_ns, self._dropped = None, 0, 0
        self._publisher = self.create_publisher(String, "/nexus/vision/detections", 20)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._image_callback, qos_profile_sensor_data)
        self.create_timer(.01, self._drain)

    @staticmethod
    def _stamp(message):
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _publish(self, message, detections=(), reason="", runtime_ms=0.):
        data = {"schema_version": 1, "sample_timestamp_ns": self._stamp(message),
                "frame_id": message.header.frame_id, "image_size_wh": [int(message.width), int(message.height)],
                "valid": not bool(reason), "reason": reason, "runtime_ms": runtime_ms, "dropped_frames": self._dropped,
                "detections": [item.as_dict() for item in detections],
                "dynamic_boxes": [list(item.bbox_xywh_px) for item in detections if item.class_id not in self._static_classes]}
        self._publisher.publish(String(data=json.dumps(data, allow_nan=False)))

    def _image_callback(self, message):
        stamp = self._stamp(message)
        age = self.get_clock().now().nanoseconds - stamp
        if not message.header.frame_id or stamp <= self._last_seen_ns or not 0 <= age <= self._max_age_ns:
            self._publish(message, reason="invalid_detection_sample_time_or_frame")
            return
        self._last_seen_ns = stamp
        if self._pending is not None:
            self._dropped += 1
        self._pending = message

    def _drain(self):
        if self._pending is None:
            return
        message, self._pending = self._pending, None
        started = perf_counter()
        try:
            if self.get_clock().now().nanoseconds - self._stamp(message) > self._max_age_ns:
                raise ValueError("queued_detection_stale")
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            detections = self.model.infer(image, **self._settings)
            if self.get_clock().now().nanoseconds - self._stamp(message) > self._max_age_ns:
                raise ValueError("detection_inference_stale")
            self._publish(message, detections, runtime_ms=(perf_counter() - started) * 1000)
        except (TypeError, ValueError, cv2.error, CvBridgeError) as error:
            self._publish(message, reason=str(error), runtime_ms=(perf_counter() - started) * 1000)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectDetectionNode()
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
