#!/usr/bin/env python3
"""Timestamp-aligned images and dynamic regions -> static visual motion."""

import json
from copy import copy
from pathlib import Path
from time import perf_counter

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import yaml

from nexus_vision_localization.detector import validate_camera_parameters
from nexus_vision_localization.reference_geometry import rigid_transform
from nexus_vision_localization.static_tracker import StaticMotionTracker
from nexus_vision_localization.superpoint_frontend import SuperPointOnnx, static_background_mask, roi_mask
from nexus_vision_localization.target_frontend import TargetVisualFrontend
from nexus_vision_localization.visual_motion import MotionConfig


def load_motion_calibration(path, allow_test=False):
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get("schema_version") != 1 or not data.get("calibration_id"):
        raise ValueError("visual calibration requires schema_version=1 and calibration_id")
    if data.get("calibration_status") != "measured" and not (allow_test and data.get("calibration_status") == "synthetic_test"):
        raise ValueError("visual motion requires measured calibration or explicit synthetic-test opt-in")
    if not data.get("camera_frame") or not isinstance(data.get("image_rectified"), bool):
        raise ValueError("visual calibration must declare optical frame and rectification")
    rigid_transform(data["transform_body_camera"])
    MotionConfig(**data.get("geometry", {}))
    return data


class SuperPointMotionNode(Node):
    def __init__(self):
        super().__init__("nexus_superpoint_motion")
        for name, default in {
            "model_manifest": "", "calibration_file": "", "allow_test_calibration": False,
            "image_topic": "/camera/image_rect", "camera_info_topic": "/camera/camera_info",
            "detections_topic": "/nexus/vision/detections", "mask_topic": "/nexus/vision/dynamic_mask",
            "mask_input": "boxes", "mask_margin_px": 4, "max_age_ms": 300., "queue_size": 4,
            "maximum_reference_age_s": .5, "maximum_points": 1024, "score_threshold": .015,
            "nms_radius": 4, "border_px": 4,
            "enable_target_tracking": False, "reference_timeout_s": 10.,
        }.items():
            self.declare_parameter(name, default)

        def value(name):
            return self.get_parameter(name).value

        self.calibration = load_motion_calibration(value("calibration_file"), value("allow_test_calibration"))
        self.model = SuperPointOnnx(value("model_manifest"))
        self.tracker = StaticMotionTracker(MotionConfig(**self.calibration.get("geometry", {})), value("maximum_reference_age_s"))
        self._max_age_ns, self._queue_size = int(value("max_age_ms") * 1e6), int(value("queue_size"))
        self._mask_input, self._mask_margin = value("mask_input"), int(value("mask_margin_px"))
        if self._max_age_ns <= 0 or self._queue_size < 1 or self._mask_margin < 0 or self._mask_input not in {"boxes", "segmentation"}:
            raise ValueError("invalid motion timing/queue/mask parameters")
        self._feature_options = {"maximum_points": int(value("maximum_points")), "threshold": float(value("score_threshold")),
                                 "nms_radius": int(value("nms_radius")), "border_px": int(value("border_px"))}
        self._bridge = CvBridge()
        self._images, self._dynamic = {}, {}
        self._camera = None
        self._last_consumed_ns, self._last_valid_ns = 0, 0
        self._dropped, self._last_reason = 0, None
        self.targets = TargetVisualFrontend() if value("enable_target_tracking") else None
        self._reference_timeout_s = float(value("reference_timeout_s"))
        if not np.isfinite(self._reference_timeout_s) or self._reference_timeout_s <= 0:
            raise ValueError("reference_timeout_s must be positive")
        self._reference_images, self._target_requests, self._request_results = {}, {}, {}
        self._last_target_packet = None
        self._motion_pub = self.create_publisher(String, "/nexus/vision/body_motion", 20)
        self._quality_pub = self.create_publisher(String, "/nexus/observations/quality", 20)
        self._status_pub = self.create_publisher(String, "/nexus/vision/motion_status", 20)
        if self.targets is not None:
            self._tracks_pub = self.create_publisher(String, "/nexus/vision/target_tracks", 20)
            self._requests_pub = self.create_publisher(String, "/nexus/vision/target_request_status", 20)
            self.create_subscription(Image, "/nexus/vision/target_reference_image", self._reference_callback, qos_profile_sensor_data)
            self.create_subscription(String, "/nexus/vision/target_requests", self._request_callback, 20)
        self.create_subscription(Image, value("image_topic"), self._image_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, value("camera_info_topic"), self._camera_callback, qos_profile_sensor_data)
        if self._mask_input == "boxes":
            self.create_subscription(String, value("detections_topic"), self._detections_callback, 20)
        else:
            self.create_subscription(Image, value("mask_topic"), self._mask_callback, qos_profile_sensor_data)
        self.create_timer(.01, self._drain)

    @staticmethod
    def _stamp(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def _check_sample(self, stamp, frame):
        if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= self._last_consumed_ns:
            raise ValueError("visual sample timestamp missing, duplicate or out of order")
        if not 0 <= self.get_clock().now().nanoseconds - stamp <= self._max_age_ns:
            raise ValueError("visual sample timestamp stale or in the future")
        if frame != self.calibration["camera_frame"]:
            raise ValueError("visual sample frame does not match calibrated camera")

    def _status(self, reason, stamp=None, *, valid=False, state="degraded", extra=None):
        self._last_reason = reason
        payload = {"schema_version": 1, "subject": "platform", "source": "superpoint", "valid": valid,
                   "reason": reason, "state": state, "sample_timestamp_ns": stamp,
                   "last_valid_sample_timestamp_ns": self._last_valid_ns, "dropped_frames": self._dropped,
                   "calibration_id": self.calibration["calibration_id"], **(extra or {})}
        self._status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def _put(self, cache, stamp, value):
        cache[stamp] = value
        while len(cache) > self._queue_size:
            del cache[min(cache)]
            self._dropped += int(cache is self._images)

    def _image_callback(self, message):
        stamp = self._stamp(message.header)
        try:
            self._check_sample(stamp, message.header.frame_id)
            if message.width < 8 or message.height < 8:
                raise ValueError("visual image dimensions are too small")
            self._put(self._images, stamp, message)
        except (TypeError, ValueError) as error:
            self._status(str(error), stamp)

    def _camera_callback(self, message):
        try:
            if message.header.frame_id != self.calibration["camera_frame"] or min(message.width, message.height) < 8:
                raise ValueError("invalid visual CameraInfo frame or dimensions")
            if self.calibration["image_rectified"]:
                matrix, distortion = np.array(message.p).reshape(3, 4)[:, :3], np.zeros(5)
            else:
                if message.distortion_model not in {"plumb_bob", "rational_polynomial", ""}:
                    raise ValueError("visual camera distortion model is unsupported")
                matrix, distortion = np.array(message.k).reshape(3, 3), np.array(message.d or [0.] * 5)
            matrix, distortion = validate_camera_parameters(matrix, distortion)
            if not np.all(np.isfinite(distortion)):
                raise ValueError("nonfinite visual camera distortion")
            self._camera = ((int(message.width), int(message.height)), matrix, distortion)
        except (TypeError, ValueError) as error:
            self._camera = None
            self.tracker.reset()
            self._status(str(error))

    def _detections_callback(self, message):
        stamp = None
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or data.get("schema_version") != 1:
                raise ValueError("detection packet requires schema_version=1")
            stamp = data["sample_timestamp_ns"]
            self._check_sample(stamp, data["frame_id"])
            if data.get("valid") is not True:
                self._put(self._dynamic, stamp, (None, "dynamic_detection_failed"))
                return
            size = data["image_size_wh"]
            if not isinstance(size, list) or len(size) != 2 or any(not isinstance(v, int) or v < 8 for v in size):
                raise ValueError("detection packet dimensions invalid")
            boxes = data["dynamic_boxes"]
            if not isinstance(boxes, list):
                raise ValueError("dynamic_boxes must be a list")
            # Validate coordinates before the packet can unblock an image.
            for box in boxes:
                roi_mask(tuple(size), box)
            detections = data.get("detections", [])
            if self.targets is not None:
                if not isinstance(detections, list):
                    raise ValueError("detections must be a list")
                for detection in detections:
                    roi_mask(tuple(size), detection["bbox_xywh_px"])
                    confidence, class_id = detection["confidence"], detection["class_id"]
                    if (not isinstance(class_id, int) or isinstance(class_id, bool) or class_id < 0
                            or not isinstance(confidence, (int, float)) or not 0 < confidence <= 1):
                        raise ValueError("target class/confidence invalid")
            self._put(self._dynamic, stamp, ((tuple(size), boxes, detections), ""))
        except (TypeError, ValueError, KeyError) as error:
            self._status(f"detections_rejected:{error}", stamp)

    def _mask_callback(self, message):
        stamp = self._stamp(message.header)
        try:
            self._check_sample(stamp, message.header.frame_id)
            if message.encoding != "mono8":
                raise ValueError("dynamic mask must be mono8 with nonzero dynamic pixels")
            mask = np.asarray(self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough"))
            if mask.ndim != 2 or mask.dtype != np.uint8:
                raise ValueError("dynamic mask must be an 8-bit scalar image")
            self._put(self._dynamic, stamp, (((int(message.width), int(message.height)), mask != 0, []), ""))
        except (TypeError, ValueError, CvBridgeError) as error:
            self._status(f"mask_rejected:{error}", stamp)

    def _request_status(self, request_id, target_id, state, reason="", *, remember=False):
        packet = {"schema_version": 1, "request_id": request_id, "target_id": target_id,
                  "state": state, "reason": reason}
        self._requests_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
        if remember:
            self._request_results[request_id] = packet
            while len(self._request_results) > 64:
                del self._request_results[next(iter(self._request_results))]

    def _reference_callback(self, message):
        stamp = self._stamp(message.header)
        if stamp <= 0 or message.header.frame_id != self.calibration["camera_frame"] or min(message.width, message.height) < 8:
            self._request_status(None, None, "rejected", "reference image timestamp/frame/dimensions invalid")
            return
        # Reference images can be historical. Their receipt deadline is separate
        # from live-image freshness and they never initialize live motion.
        self._reference_images[stamp] = (message, perf_counter())
        while len(self._reference_images) > self._queue_size:
            del self._reference_images[next(iter(self._reference_images))]

    def _request_callback(self, message):
        request_id, target_id = None, None
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict) or request.get("schema_version") != 1:
                raise ValueError("target request requires schema_version=1")
            request_id, target_id = request["request_id"], request["target_id"]
            if any(not isinstance(value, str) or not value.strip() for value in (request_id, target_id)):
                raise ValueError("request_id and target_id must be nonempty strings")
            if request_id in self._request_results:
                self._requests_pub.publish(String(data=json.dumps(self._request_results[request_id])))
                return
            if request_id in self._target_requests:
                self._request_status(request_id, target_id, "pending")
                return
            stamp = request["sample_timestamp_ns"]
            if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0 or request["frame_id"] != self.calibration["camera_frame"]:
                raise ValueError("reference request timestamp/frame invalid")
            box = np.asarray(request["bbox_xywh_px"], dtype=float)
            if box.shape != (4,) or not np.all(np.isfinite(box)) or np.any(box[2:] <= 0):
                raise ValueError("reference ROI must contain finite xywh with positive size")
            class_id = request.get("class_id")
            if class_id is not None and (not isinstance(class_id, int) or isinstance(class_id, bool) or class_id < 0):
                raise ValueError("reference class_id must be a nonnegative integer")
            existing_id = request.get("existing_track_id")
            if existing_id is not None and (not isinstance(existing_id, str) or not existing_id.strip()):
                raise ValueError("existing_track_id must be a nonempty string")
            if len(self._target_requests) >= self._queue_size:
                raise ValueError("reference request queue is full")
            self._target_requests[request_id] = (request, perf_counter())
            self._request_status(request_id, target_id, "pending")
        except (KeyError, TypeError, ValueError) as error:
            self._request_status(request_id if isinstance(request_id, str) else None,
                                 target_id if isinstance(target_id, str) else None, "rejected", str(error))

    def _drain_references(self):
        now = perf_counter()
        for stamp, (_, receipt) in list(self._reference_images.items()):
            if now - receipt > self._reference_timeout_s:
                del self._reference_images[stamp]
        for request_id, (request, receipt) in list(self._target_requests.items()):
            if now - receipt > self._reference_timeout_s:
                self._request_status(request_id, request["target_id"], "rejected", "reference_pair_timeout", remember=True)
                del self._target_requests[request_id]
                continue
            pair = self._reference_images.get(request["sample_timestamp_ns"])
            if pair is None:
                continue
            del self._target_requests[request_id]
            try:
                image = np.asarray(self._bridge.imgmsg_to_cv2(pair[0], desired_encoding="bgr8"))
                dense = self.model.infer(image)
                if perf_counter() - receipt > self._reference_timeout_s:
                    raise ValueError("reference_processing_timeout")
                pending = copy(self.targets)
                pending.tracker = copy(self.targets.tracker)
                pending.tracker.tracks = dict(self.targets.tracker.tracks)
                pending.register(request["target_id"], dense, image, request["bbox_xywh_px"], self._feature_options,
                                 class_id=request.get("class_id"), existing_track_id=request.get("existing_track_id"),
                                 motion_model=request.get("motion_model", "static"), anchor_reference_px=request.get("anchor_reference_px"))
                if perf_counter() - receipt > self._reference_timeout_s:
                    raise ValueError("reference_processing_timeout")
                self.targets.tracker = pending.tracker
                self._request_status(request_id, request["target_id"], "registered", remember=True)
            except (TypeError, ValueError, cv2.error, CvBridgeError, np.linalg.LinAlgError) as error:
                self._request_status(request_id, request["target_id"], "rejected", str(error), remember=True)
            break  # bounded one-time reference work per live processing cycle

    def _target_watchdog(self, now):
        packet = self._last_target_packet
        if packet is None or not packet["valid"] or now - packet["sample_timestamp_ns"] <= self._max_age_ns:
            return
        packet = {**packet, "valid": False, "diagnostics": [{"reason": "target_observations_stale"}],
                  "tracks": [{**track, "state": "lost", "observed": False, "feature_observations": []} for track in packet["tracks"]]}
        self._tracks_pub.publish(String(data=json.dumps(packet, allow_nan=False)))
        self._last_target_packet = packet
        # Do not advance the numerical tracker using wall time: the next image
        # must still be processed at its actual sample time.

    def _drain(self):
        if self.targets is not None:
            self._drain_references()
        now = self.get_clock().now().nanoseconds
        if self.targets is not None:
            self._target_watchdog(now)
        for cache in (self._images, self._dynamic):
            for stamp in list(cache):
                if now - stamp > self._max_age_ns:
                    del cache[stamp]
                    self._dropped += int(cache is self._images)
                    self._status("unpaired_or_queued_frame_stale", stamp)
        if self._last_valid_ns and now - self._last_valid_ns > self._max_age_ns and self._last_reason != "visual_motion_lost":
            self._status("visual_motion_lost", self._last_valid_ns, state="lost")
        ready = self._images.keys() & self._dynamic.keys()
        if not ready or self._camera is None:
            return
        stamp = max(ready)
        message, (dynamic, error) = self._images[stamp], self._dynamic[stamp]
        for cache in (self._images, self._dynamic):
            for old in list(cache):
                if old <= stamp:
                    del cache[old]
                    self._dropped += int(cache is self._images and old < stamp)
        self._last_consumed_ns = stamp
        if dynamic is None:
            self._status(error, stamp)
            return
        started = perf_counter()
        try:
            size = (int(message.width), int(message.height))
            if dynamic[0] != size or self._camera[0] != size:
                raise ValueError("image, dynamic regions and CameraInfo dimensions differ")
            image = np.asarray(self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"))
            dense = self.model.infer(image)
            if self.get_clock().now().nanoseconds - stamp > self._max_age_ns:
                raise ValueError("superpoint_inference_stale")
            pending_targets, target_observations, exclusions = None, [], []
            if self.targets is not None:
                pending_targets, target_observations, exclusions = self.targets.prepare(
                    dense, image, stamp, dynamic[2], self._feature_options)
            allowed = static_background_mask(size,
                                             dynamic_boxes=(dynamic[1] if self._mask_input == "boxes" else []) + exclusions,
                                             dynamic_mask=dynamic[1] if self._mask_input == "segmentation" else None,
                                             margin_px=self._mask_margin)
            features = dense.features(allowed, **self._feature_options)
            # A slow inference must not install an already stale reference.
            if self.get_clock().now().nanoseconds - stamp > self._max_age_ns:
                raise ValueError("superpoint_frontend_stale")
            pending_motion = copy(self.tracker)
            result = pending_motion.process(features, stamp, self._camera[1], self._camera[2])
            target_packet = (None if pending_targets is None else self.targets.packet(
                stamp, message.header.frame_id, size, target_observations, pending_targets.last_diagnostics, tracker=pending_targets))
            if self.get_clock().now().nanoseconds - stamp > self._max_age_ns:
                raise ValueError("superpoint_geometry_stale")
            self.tracker = pending_motion
            if pending_targets is not None:
                self.targets.tracker = pending_targets
                self._last_target_packet = target_packet
                self._tracks_pub.publish(String(data=json.dumps(target_packet, allow_nan=False)))
            runtime_ms = (perf_counter() - started) * 1000
            if result.motion is None:
                self._status(result.reason, stamp, state=result.state,
                             extra={"static_points": len(features.points_px), "runtime_ms": runtime_ms})
                return
            observation = result.motion.body_observation(result.previous_stamp_ns, stamp, self.calibration["transform_body_camera"])
            quality = dict(result.motion.quality)
            quality["blur_metric"] = float(cv2.Laplacian(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
            quality["dt_s"] = (stamp - result.previous_stamp_ns) / 1e9
            quality["network_latency_s"] = (self.get_clock().now().nanoseconds - stamp) / 1e9
            self._quality_pub.publish(String(data=json.dumps({
                "schema_version": 1, "subject": "platform", "source": "superpoint",
                "sample_timestamp_ns": stamp, "features": quality,
            }, allow_nan=False)))
            self._motion_pub.publish(String(data=json.dumps(observation, allow_nan=False)))
            self._last_valid_ns = stamp
            self._status("", stamp, valid=True, state="tracking", extra={
                "runtime_ms": runtime_ms, "static_points": len(features.points_px),
                "matched_points": len(result.motion.current_indices), "median_parallax_deg": result.motion.median_parallax_deg,
            })
        except (TypeError, ValueError, cv2.error, CvBridgeError, np.linalg.LinAlgError) as failure:
            self._status(str(failure), stamp)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SuperPointMotionNode()
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
