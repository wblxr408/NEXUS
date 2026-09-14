#!/usr/bin/env python3
"""Same-sample detection packets for platform masks and external target discovery."""

import json
from time import perf_counter

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from nexus_vision_localization.object_detector import ObjectDetectorOnnx


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("nexus_object_detection")
        for name, default in {"model_manifest": "", "lite_model_manifest": "", "robust_model_manifest": "", "image_topic": "/camera/image_rect", "max_age_ms": 300.,
                              "confidence_threshold": .35, "iou_threshold": .5, "maximum_detections": 100,
                              "static_class_ids": "", "compute_plan_topic": "/nexus/optimization/compute_plan",
                              "default_detector_period_s": .5, "mask_prediction_horizon_s": .5}.items():
            self.declare_parameter(name, default)
        self.model = ObjectDetectorOnnx(self.get_parameter("model_manifest").value)
        lite_manifest = self.get_parameter("lite_model_manifest").value
        self._lite_model = ObjectDetectorOnnx(lite_manifest) if lite_manifest else None
        robust_manifest = self.get_parameter("robust_model_manifest").value
        self._robust_model = ObjectDetectorOnnx(robust_manifest) if robust_manifest else None
        for name, candidate in (("lite", self._lite_model), ("robust", self._robust_model)):
            if candidate is not None and candidate.manifest["class_names"] != self.model.manifest["class_names"]:
                raise ValueError(f"{name} detector class order must match the full detector")
        self._active_model = self.model
        self._active_model_name = "full"
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
        self._last_dynamic_mask, self._last_dynamic_mask_ns = [], 0
        self._next_inference_ns = 0
        self._detector_period_ns = int(float(self.get_parameter("default_detector_period_s").value) * 1e9)
        self._image_scale = 1.
        if self._detector_period_ns <= 0:
            raise ValueError("default_detector_period_s must be positive")
        self._mask_prediction_horizon_ns = int(float(self.get_parameter("mask_prediction_horizon_s").value) * 1e9)
        if self._mask_prediction_horizon_ns <= 0:
            raise ValueError("mask_prediction_horizon_s must be positive")
        self._publisher = self.create_publisher(String, "/nexus/vision/detections", 20)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._image_callback, qos_profile_sensor_data)
        self.create_subscription(String, self.get_parameter("compute_plan_topic").value, self._plan_callback, 10)
        self.create_timer(.01, self._drain)

    @staticmethod
    def _stamp(message):
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _publish(self, message, detections=(), reason="", runtime_ms=0., mode="detector", predicted_mask_boxes=()):
        data = {"schema_version": 1, "sample_timestamp_ns": self._stamp(message),
                "frame_id": message.header.frame_id, "image_size_wh": [int(message.width), int(message.height)],
                "valid": not bool(reason), "reason": reason, "runtime_ms": runtime_ms, "dropped_frames": self._dropped,
                "mode": mode,
                "model_profile": self._active_model_name, "image_scale": self._image_scale,
                "detections": [item.as_dict() for item in detections],
                "dynamic_boxes": [list(item.bbox_xywh_px) for item in detections if item.class_id not in self._static_classes],
                # These boxes are deliberately separate from detections: they
                # protect platform geometry only and cannot create an instance
                # observation in a skipped detector frame.
                "predicted_mask_boxes": [list(box) for box in predicted_mask_boxes]}
        self._publisher.publish(String(data=json.dumps(data, allow_nan=False)))

    def _plan_callback(self, message):
        try:
            plan = json.loads(message.data)
            now = self.get_clock().now().nanoseconds
            period = float(plan["detector_period_s"])
            image_scale = float(plan["image_scale"])
            high_accuracy = plan["high_accuracy"]
            robust_model = plan.get("robust_model", False)
            if (plan.get("schema_version") != 1 or plan.get("source") != "adaptive_observation"
                    or not np.isfinite(period) or not .05 <= period <= 10. or not .25 <= image_scale <= 1.
                    or not isinstance(high_accuracy, bool) or not isinstance(robust_model, bool)
                    or not isinstance(plan.get("valid_until_ns"), int) or plan["valid_until_ns"] < now):
                raise ValueError("invalid or expired compute plan")
            self._detector_period_ns = int(period * 1e9)
            self._image_scale = image_scale
            if robust_model and self._robust_model is not None:
                self._active_model = self._robust_model
                self._active_model_name = "robust"
            elif high_accuracy or self._lite_model is None:
                self._active_model = self.model
                self._active_model_name = "full"
            else:
                self._active_model = self._lite_model
                self._active_model_name = "lite"
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return

    def _image_callback(self, message):
        stamp = self._stamp(message)
        age = self.get_clock().now().nanoseconds - stamp
        if not message.header.frame_id or stamp <= self._last_seen_ns or not 0 <= age <= self._max_age_ns:
            self._publish(message, reason="invalid_detection_sample_time_or_frame")
            return
        self._last_seen_ns = stamp
        if stamp < self._next_inference_ns:
            # A tracking-only packet remains same-sample and explicitly never
            # reuses a stale detector box as if it were a fresh observation.
            predicted = (self._last_dynamic_mask if 0 <= stamp - self._last_dynamic_mask_ns <= self._mask_prediction_horizon_ns else [])
            self._publish(message, mode="tracking_only", predicted_mask_boxes=predicted)
            return
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
            detections = self._active_model.infer(image, image_scale=self._image_scale, **self._settings)
            if self.get_clock().now().nanoseconds - self._stamp(message) > self._max_age_ns:
                raise ValueError("detection_inference_stale")
            self._publish(message, detections, runtime_ms=(perf_counter() - started) * 1000)
            self._last_dynamic_mask = [tuple(item.bbox_xywh_px) for item in detections if item.class_id not in self._static_classes]
            self._last_dynamic_mask_ns = self._stamp(message)
            self._next_inference_ns = self._stamp(message) + self._detector_period_ns
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
