#!/usr/bin/env python3
"""Timestamped target feature tracks and platform odometry -> metric states."""

import json
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
import yaml

from nexus_msgs.msg import TargetKinematicState
from nexus_vision_localization.detector import validate_camera_parameters
from nexus_vision_localization.pose_timeline import PoseTimeline, TimedPose
from nexus_vision_localization.reference_geometry import pose_covariance, rigid_transform
from nexus_vision_localization.target_multiview import BearingObservation, MultiviewConfig, solve_multiview


def load_target_calibration(path, allow_test=False):
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get("schema_version") != 1 or not data.get("calibration_id"):
        raise ValueError("target calibration requires schema_version=1 and calibration_id")
    if data.get("calibration_status") != "measured" and not (allow_test and data.get("calibration_status") == "synthetic_test"):
        raise ValueError("target geometry requires measured calibration or explicit synthetic-test opt-in")
    if not data.get("camera_frame") or not isinstance(data.get("image_rectified"), bool):
        raise ValueError("target calibration requires optical frame and image_rectified")
    rigid_transform(data["transform_body_camera"])
    pose_covariance(data["extrinsic_covariance"])
    if not np.isfinite(data["pixel_sigma_px"]) or data["pixel_sigma_px"] <= 0:
        raise ValueError("target pixel sigma must be positive")
    MultiviewConfig(**data.get("target_geometry", {}))
    return data


class TargetMetricNode(Node):
    def __init__(self):
        super().__init__("nexus_target_metric")
        for name, default in {"calibration_file": "", "allow_test_calibration": False,
                              "camera_info_topic": "/camera/camera_info", "platform_topic": "/nexus/platform/odom",
                              "tracks_topic": "/nexus/vision/target_tracks", "max_age_ms": 300.,
                              "window_size": 12, "window_duration_s": 3., "queue_size": 8,
                              "maximum_tracks": 64}.items():
            self.declare_parameter(name, default)
        self.calibration = load_target_calibration(self.get_parameter("calibration_file").value,
                                                   self.get_parameter("allow_test_calibration").value)
        self.geometry = MultiviewConfig(**self.calibration.get("target_geometry", {}))
        self.timeline = PoseTimeline(**self.calibration.get("target_pose_timeline", {}))
        self._max_age_ns = int(self.get_parameter("max_age_ms").value * 1e6)
        self._duration_ns = int(self.get_parameter("window_duration_s").value * 1e9)
        self._window_size = int(self.get_parameter("window_size").value)
        self._queue_size = int(self.get_parameter("queue_size").value)
        self._maximum_tracks = int(self.get_parameter("maximum_tracks").value)
        if min(self._max_age_ns, self._duration_ns, self._queue_size, self._maximum_tracks) <= 0 or self._window_size < 4:
            raise ValueError("invalid metric target window/queue/timing configuration")
        self._camera = None
        self._pending, self._windows, self._latest = {}, {}, {}
        self._last_input_ns, self._last_processed_ns = 0, 0
        self._publisher = self.create_publisher(TargetKinematicState, "/nexus/vision/target_kinematics", 20)
        self._status_publisher = self.create_publisher(String, "/nexus/vision/target_geometry_status", 20)
        self._quality_publisher = self.create_publisher(String, "/nexus/observations/quality", 20)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self._camera_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter("platform_topic").value, self._platform_callback, 100)
        self.create_subscription(String, self.get_parameter("tracks_topic").value, self._tracks_callback, 20)
        self.create_timer(.01, self._drain)

    @staticmethod
    def _stamp(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def _status(self, reason, stamp=None, target_id=None, **extra):
        self._status_publisher.publish(String(data=json.dumps({"schema_version": 1, "sample_timestamp_ns": stamp,
                                                              "target_id": target_id, "reason": reason, **extra}, allow_nan=False)))

    def _camera_callback(self, message):
        try:
            if message.header.frame_id != self.calibration["camera_frame"] or min(message.width, message.height) < 8:
                raise ValueError("target_camera_frame_or_dimensions_invalid")
            if self.calibration["image_rectified"]:
                matrix, distortion = np.array(message.p).reshape(3, 4)[:, :3], np.zeros(5)
            else:
                if message.distortion_model not in {"", "plumb_bob", "rational_polynomial"}:
                    raise ValueError("target_camera_distortion_unsupported")
                matrix, distortion = np.array(message.k).reshape(3, 3), np.array(message.d or [0.] * 5)
            matrix, distortion = validate_camera_parameters(matrix, distortion)
            if not np.all(np.isfinite(distortion)):
                raise ValueError("target_camera_distortion_nonfinite")
            camera = ((int(message.width), int(message.height)), matrix, distortion)
            if self._camera is not None and any(not np.array_equal(a, b) for a, b in zip(camera, self._camera)):
                self._pending.clear()
                self._windows.clear()
                self._status("target_camera_geometry_changed")
            self._camera = camera
        except (TypeError, ValueError) as error:
            self._camera = None
            self._pending.clear()
            self._windows.clear()
            self._status(str(error))

    def _platform_callback(self, message):
        stamp = self._stamp(message.header)
        try:
            now = self.get_clock().now().nanoseconds
            if message.header.frame_id != "map" or message.child_frame_id != "base_link" or not 0 <= now - stamp <= self.timeline.retention_ns:
                raise ValueError("target_platform_frame_or_timestamp_invalid")
            q = message.pose.pose.orientation
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if not np.all(np.isfinite(quaternion)) or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-5):
                raise ValueError("target_platform_quaternion_invalid")
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
            p = message.pose.pose.position
            transform[:3, 3] = [p.x, p.y, p.z]
            self.timeline.add(TimedPose(stamp, transform, np.array(message.pose.covariance).reshape(6, 6)))
        except (TypeError, ValueError, np.linalg.LinAlgError) as error:
            self._status(str(error), stamp)

    def _tracks_callback(self, message):
        stamp = None
        try:
            packet = json.loads(message.data)
            if not isinstance(packet, dict) or packet.get("schema_version") != 1 or packet.get("unit") != "px":
                raise ValueError("target feature packet requires schema_version=1 and pixel units")
            stamp = packet["sample_timestamp_ns"]
            if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0 or packet["frame_id"] != self.calibration["camera_frame"]:
                raise ValueError("target feature timestamp/frame invalid")
            now = self.get_clock().now().nanoseconds
            if stamp > now:
                raise ValueError("target feature timestamp is in the future")
            if not isinstance(packet["tracks"], list) or len(packet["tracks"]) > self._maximum_tracks:
                raise ValueError("target feature track count invalid")
            ids = [track["track_id"] for track in packet["tracks"]]
            if any(not isinstance(identifier, str) or not identifier for identifier in ids) or len(ids) != len(set(ids)):
                raise ValueError("target feature identities empty or duplicated")
            if packet.get("valid") is not True:
                if stamp >= self._last_input_ns:
                    for track in packet["tracks"]:
                        self._invalid(track["track_id"], stamp, "target_frontend_invalid")
                    self._pending = {key: value for key, value in self._pending.items() if key > stamp}
                return
            if stamp <= self._last_input_ns or not 0 <= now - stamp <= self._max_age_ns:
                raise ValueError("target feature timestamp stale, duplicate or out of order")
            size = packet["image_size_wh"]
            if len(size) != 2 or any(not isinstance(v, int) or v < 8 for v in size):
                raise ValueError("target feature image dimensions invalid")
            self._last_input_ns = stamp
            self._pending[stamp] = packet
            while len(self._pending) > self._queue_size:
                old = min(self._pending)
                evicted = self._pending.pop(old)
                for track in evicted["tracks"]:
                    self._invalid(track["track_id"], old, "target_geometry_queue_full")
        except (TypeError, ValueError, KeyError) as error:
            self._status(str(error), stamp)

    def _empty_message(self, identifier, stamp, reason):
        output = TargetKinematicState()
        output.header.stamp.sec, output.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        output.header.frame_id = "map"
        output.receive_timestamp_ns = max(stamp, self.get_clock().now().nanoseconds)
        output.target_id, output.unit = identifier, "m"
        output.state, output.reason = "degraded", reason
        previous = self._latest.get(identifier)
        output.last_valid_sample_timestamp_ns = previous.last_valid_sample_timestamp_ns if previous is not None else 0
        output.pose.position.x = output.pose.position.y = output.pose.position.z = float("nan")
        output.pose.orientation.x = output.pose.orientation.y = output.pose.orientation.z = output.pose.orientation.w = float("nan")
        output.velocity.x = output.velocity.y = output.velocity.z = float("nan")
        output.covariance = [float("nan")] * 81
        return output

    def _invalid(self, identifier, stamp, reason):
        previous = self._latest.get(identifier)
        if previous is not None and self._stamp(previous.header) > stamp:
            return
        if previous is None and len(self._latest) >= self._maximum_tracks:
            oldest = min(self._latest, key=lambda key: self._stamp(self._latest[key].header))
            del self._latest[oldest]
            self._windows.pop(oldest, None)
        output = self._empty_message(identifier, stamp, reason)
        output.state = "lost" if reason in {"target_frontend_invalid", "target_geometry_stale", "target_not_observed"} else "degraded"
        self._publisher.publish(output)
        self._latest[identifier] = output
        self._status(reason, stamp, identifier, valid=False)

    def _process_track(self, track, stamp, platform):
        identifier = track["track_id"]
        if track.get("observed") is not True or track.get("identity_ambiguous") is not False:
            if track.get("identity_ambiguous"):
                self._windows.pop(identifier, None)
            self._invalid(identifier, stamp, "target_identity_ambiguous" if track.get("identity_ambiguous") else "target_not_observed")
            return
        anchor, reference = track.get("anchor_feature_id"), track.get("reference_id")
        model = track.get("motion_model")
        if not isinstance(anchor, int) or isinstance(anchor, bool) or anchor < 0 or not isinstance(reference, str) or not reference:
            raise ValueError("target_reference_anchor_missing")
        if model != "static":
            raise ValueError("target_motion_model_must_be_static")
        features = track["feature_observations"]
        ids = [feature["feature_id"] for feature in features]
        if len(ids) != len(set(ids)):
            raise ValueError("target_feature_ids_duplicated")
        points = [feature["pixel_px"] for feature in features if feature["feature_id"] == anchor]
        if not points:
            self._invalid(identifier, stamp, "target_reference_anchor_not_visible")
            return
        point = np.asarray(points[0], dtype=float)
        if point.shape != (2,) or not np.all(np.isfinite(point)) or np.any(point < 0) or np.any(point >= self._camera[0]):
            raise ValueError("target_anchor_outside_image")
        confidence = float(track["identity_confidence"])
        if not np.isfinite(confidence) or not 0 < confidence <= 1:
            raise ValueError("target_identity_confidence_invalid")
        key = (reference, anchor, model)
        previous_key, history = self._windows.get(identifier, (None, []))
        history = [item for item in history if stamp - item.platform.stamp_ns <= self._duration_ns] if previous_key == key else []
        observation = BearingObservation(platform, point, self._camera[1], self._camera[2], self.calibration["pixel_sigma_px"])
        pending = (history + [observation])[-self._window_size:]
        started = perf_counter()
        try:
            result = solve_multiview(pending, self.calibration["transform_body_camera"], self.calibration["extrinsic_covariance"],
                                     motion_model=model, config=self.geometry)
        except ValueError as error:
            reason = "target_geometry_stale" if self.get_clock().now().nanoseconds - stamp > self._max_age_ns else str(error)
            if reason in {"insufficient_target_views", "target_motion_geometrically_unobservable", "target_parallax_insufficient"}:
                self._store_window(identifier, key, pending)
            self._invalid(identifier, stamp, reason)
            return
        if self.get_clock().now().nanoseconds - stamp > self._max_age_ns:
            self._invalid(identifier, stamp, "target_geometry_stale")
            return
        self._store_window(identifier, key, [item for item in pending if item.platform.stamp_ns in result.inlier_stamps])
        output = self._empty_message(identifier, stamp, "")
        output.valid = output.position_observed = True
        output.velocity_observed = result.velocity_mps is not None
        output.state = track["state"]
        output.source = "superpoint_multiview_" + model
        output.position_reference = f"reference_feature:{reference}:{anchor}"
        output.pose.position.x, output.pose.position.y, output.pose.position.z = result.position_m.tolist()
        covariance = np.full((9, 9), np.nan)
        dimension = len(result.covariance)
        covariance[:dimension, :dimension] = result.covariance
        output.covariance = covariance.reshape(-1).tolist()
        if result.velocity_mps is not None:
            output.velocity.x, output.velocity.y, output.velocity.z = result.velocity_mps.tolist()
        output.last_valid_sample_timestamp_ns = stamp
        output.confidence = float(confidence * len(result.inlier_stamps) / len(pending))
        quality = {"reprojection_error_px": result.reprojection_rmse_px,
                   "inlier_ratio": len(result.inlier_stamps) / len(pending),
                   "network_latency_s": (self.get_clock().now().nanoseconds - stamp) / 1e9}
        if result.velocity_mps is not None:
            quality["speed_mps"] = float(np.linalg.norm(result.velocity_mps))
        self._quality_publisher.publish(String(data=json.dumps({
            "schema_version": 1, "subject": "target", "target_id": identifier, "source_mode": 3,
            "source": output.source, "position_reference": output.position_reference,
            "sample_timestamp_ns": stamp, "features": quality}, allow_nan=False)))
        self._publisher.publish(output)
        self._latest[identifier] = output
        self._status("", stamp, identifier, valid=True, inlier_stamps=list(result.inlier_stamps),
                     parallax_deg=result.parallax_deg, condition_number=result.condition_number,
                     runtime_ms=(perf_counter() - started) * 1000, interpolated_platform=platform.interpolated)

    def _store_window(self, identifier, key, history):
        if identifier not in self._windows and len(self._windows) >= self._maximum_tracks:
            oldest = min(self._windows, key=lambda name: self._windows[name][1][-1].platform.stamp_ns)
            del self._windows[oldest]
        self._windows[identifier] = (key, history)

    def _drain(self):
        now = self.get_clock().now().nanoseconds
        for identifier, message in list(self._latest.items()):
            if message.valid and now - self._stamp(message.header) > self._max_age_ns:
                self._invalid(identifier, self._stamp(message.header), "target_geometry_stale")
        for stamp, packet in sorted(self._pending.items()):
            if stamp <= self._last_processed_ns:
                del self._pending[stamp]
                continue
            if now - stamp > self._max_age_ns:
                del self._pending[stamp]
                for track in packet["tracks"]:
                    self._invalid(track["track_id"], stamp, "target_geometry_stale")
                continue
            if self._camera is None:
                if packet.get("_waiting_reason") != "target_camera_info_missing":
                    packet["_waiting_reason"] = "target_camera_info_missing"
                    self._status(packet["_waiting_reason"], stamp, valid=False, waiting=True)
                continue
            try:
                platform = self.timeline.at(stamp)
            except ValueError as error:
                if packet.get("_waiting_reason") != str(error):
                    packet["_waiting_reason"] = str(error)
                    self._status(str(error), stamp, valid=False, waiting=True)
                continue  # wait for the next bracketing pose, bounded by age
            del self._pending[stamp]
            self._last_processed_ns = stamp
            if tuple(packet["image_size_wh"]) != self._camera[0]:
                for track in packet["tracks"]:
                    self._invalid(track["track_id"], stamp, "target_camera_dimensions_mismatch")
                continue
            for track in packet["tracks"]:
                try:
                    self._process_track(track, stamp, platform)
                except (TypeError, ValueError, KeyError, cv2.error, np.linalg.LinAlgError) as error:
                    self._invalid(track["track_id"], stamp, str(error))
            now = self.get_clock().now().nanoseconds


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetMetricNode()
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
