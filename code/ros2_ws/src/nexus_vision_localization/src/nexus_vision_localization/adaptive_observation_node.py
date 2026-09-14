#!/usr/bin/env python3
"""Publish compute budgets and safe active-observation recommendations.

This node intentionally has no publisher to a flight controller.  It makes the
planner's decision inspectable on ROS before a separately authorised controller
is ever allowed to consume it.
"""

import json
from collections import deque
from pathlib import Path
from time import perf_counter

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from nexus_msgs.msg import TargetKinematicState
from nexus_vision_localization.adaptive_policy import (
    ActiveObservationPlanner, AdaptiveComputeScheduler, gyro_vibration,
)


def _stamp(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class AdaptiveObservationNode(Node):
    def __init__(self):
        super().__init__("nexus_adaptive_observation")
        defaults = {"imu_topic": "/nexus/fcu/imu", "platform_topic": "/nexus/platform/odom",
                    "target_topic": "/nexus/target/kinematics", "quality_topic": "/nexus/observations/quality",
                    "maximum_imu_samples": 128, "minimum_detector_period_s": .5,
                    "maximum_detector_period_s": 2., "base_keypoints": 256,
                    "policy_profile_file": "",
                    "minimum_orbit_radius_m": 1.5, "maximum_orbit_radius_m": 6.,
                    "desired_parallax_deg": 12., "recommended_altitude_m": float("nan"),
                    "maximum_speed_mps": 3., "maximum_acceleration_mps2": 2., "planning_horizon_s": 3.,
                    "trajectory_steps": 8, "forbidden_circles_json": "[]", "forbidden_spheres_json": "[]"}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        profile_path = str(value("policy_profile_file"))
        profile = None
        if profile_path:
            path = Path(profile_path)
            if not path.is_file():
                raise ValueError("policy_profile_file does not exist")
            profile = json.loads(path.read_text(encoding="utf-8"))
        self.scheduler = AdaptiveComputeScheduler(minimum_detector_period_s=value("minimum_detector_period_s"),
                                                   maximum_detector_period_s=value("maximum_detector_period_s"),
                                                   base_keypoints=value("base_keypoints"), profile=profile)
        self.planner = ActiveObservationPlanner(minimum_radius_m=value("minimum_orbit_radius_m"),
                                                 maximum_radius_m=value("maximum_orbit_radius_m"),
                                                 desired_parallax_deg=value("desired_parallax_deg"),
                                                 maximum_speed_mps=value("maximum_speed_mps"),
                                                 maximum_acceleration_mps2=value("maximum_acceleration_mps2"),
                                                 planning_horizon_s=value("planning_horizon_s"),
                                                 trajectory_steps=value("trajectory_steps"))
        self._imu = deque(maxlen=int(value("maximum_imu_samples")))
        if self._imu.maxlen < 4:
            raise ValueError("maximum_imu_samples must be >= 4")
        self._platform, self._target, self._previous_view = None, None, None
        self._target_state, self._last_recommended_target_stamp = "lost", None
        self._quality = {}
        self._gdop = None
        try:
            self._forbidden_circles = json.loads(value("forbidden_circles_json"))
            self._forbidden_spheres = json.loads(value("forbidden_spheres_json"))
            if not isinstance(self._forbidden_circles, list) or not isinstance(self._forbidden_spheres, list):
                raise ValueError
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError("forbidden regions must be JSON lists") from error
        self._last_plan_ns = 0
        self._compute_pub = self.create_publisher(String, "/nexus/optimization/compute_plan", 10)
        self._observation_pub = self.create_publisher(String, "/nexus/optimization/observation_plan", 10)
        self._vibration_pub = self.create_publisher(String, "/nexus/optimization/vibration_quality", 10)
        self.create_subscription(Imu, value("imu_topic"), self._on_imu, 100)
        self.create_subscription(Odometry, value("platform_topic"), self._on_platform, 30)
        self.create_subscription(TargetKinematicState, value("target_topic"), self._on_target, 30)
        self.create_subscription(String, value("quality_topic"), self._on_quality, 30)
        self.create_timer(.1, self._publish)

    def _on_imu(self, message):
        stamp = _stamp(message.header)
        vector = np.array([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z], dtype=float)
        if stamp > 0 and np.all(np.isfinite(vector)) and (not self._imu or stamp > self._imu[-1][0]):
            self._imu.append((stamp, vector))

    def _on_platform(self, message):
        if message.header.frame_id != "map":
            return
        point = np.array([message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z], dtype=float)
        stamp = _stamp(message.header)
        if stamp > 0 and np.all(np.isfinite(point)):
            self._platform = stamp, point

    def _on_target(self, message):
        if not message.valid or not message.position_observed or message.header.frame_id != "map" or message.unit != "m":
            return
        point = np.array([message.pose.position.x, message.pose.position.y, message.pose.position.z], dtype=float)
        covariance = np.asarray(message.covariance, dtype=float).reshape(9, 9)[:3, :3]
        stamp = _stamp(message.header)
        if stamp > 0 and np.all(np.isfinite(point)) and np.all(np.isfinite(covariance)):
            self._target = (message.target_id, stamp, point, float(np.sqrt(max(0., np.trace(covariance) / 3))),
                            float(message.confidence) if np.isfinite(message.confidence) else None)
            self._target_state = message.state

    def _on_quality(self, message):
        try:
            packet = json.loads(message.data)
            if packet.get("schema_version") != 1:
                return
            features = packet.get("features", {})
            if packet.get("subject") == "platform" and "gdop" in features and np.isfinite(features["gdop"]):
                self._gdop = float(features["gdop"])
                return
            if packet.get("subject") != "target":
                return
            self._quality = {key: float(features[key]) for key in ("inlier_ratio", "reprojection_error_px", "blur_metric", "occlusion_ratio",
                                                                     "outlier_probability", "network_latency_s")
                             if key in features and np.isfinite(features[key])}
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return

    def _vibration(self):
        if len(self._imu) < 4:
            return {"vibration_quality": 0., "gyro_vibration_rad_s": None, "image_rotation_sigma_rad": None}
        stamps = np.array([item[0] for item in self._imu], dtype=float)
        try:
            return gyro_vibration(np.array([item[1] for item in self._imu]), np.diff(stamps) / 1e9)
        except ValueError:
            return {"vibration_quality": 0., "gyro_vibration_rad_s": None, "image_rotation_sigma_rad": None}

    def _publish(self):
        started, now = perf_counter(), self.get_clock().now().nanoseconds
        vibration = self._vibration()
        identity, ambiguous = (None, self._target_state in {"degraded", "lost"})
        if self._target is not None:
            identifier, stamp, _, _, identity = self._target
        plan = self.scheduler.decide(identity_confidence=identity, inlier_ratio=self._quality.get("inlier_ratio"),
                                     reprojection_error_px=self._quality.get("reprojection_error_px"),
                                     blur_metric=self._quality.get("blur_metric"), vibration_quality=vibration["vibration_quality"],
                                     occlusion_ratio=self._quality.get("occlusion_ratio"),
                                     visibility=(None if "occlusion_ratio" not in self._quality
                                                 else 1. - self._quality["occlusion_ratio"]),
                                     outlier_probability=self._quality.get("outlier_probability"),
                                     latency_ms=1000. * self._quality.get("network_latency_s", 0.), identity_ambiguous=ambiguous)
        envelope = {"schema_version": 1, "sample_timestamp_ns": now, "valid_until_ns": now + int(plan.detector_period_s * 2e9),
                    "source": "adaptive_observation", **plan.as_dict()}
        self._compute_pub.publish(String(data=json.dumps(envelope, allow_nan=False)))
        self._vibration_pub.publish(String(data=json.dumps({"schema_version": 1, "sample_timestamp_ns": now,
                                                            "source": "imu_window", **vibration}, allow_nan=False)))
        if self._platform is not None and self._target is not None:
            _, platform = self._platform
            identifier, _, target, sigma, _confidence = self._target
            altitude = self.get_parameter("recommended_altitude_m").value
            # Target confidence is an identity score, not target visibility.
            # Only the occlusion metric supplies visibility to the planner.
            visibility = (None if "occlusion_ratio" not in self._quality
                          else float(np.clip(1. - self._quality["occlusion_ratio"], 0., 1.)))
            recommendation = self.planner.recommend(platform, target, previous_view_map_m=self._previous_view,
                                                     target_sigma_m=sigma, gdop=self._gdop, forbidden_circles=self._forbidden_circles,
                                                     forbidden_spheres=self._forbidden_spheres,
                                                     reprojection_error_px=self._quality.get("reprojection_error_px"),
                                                     visibility=visibility,
                                                     allowed_altitude_m=None if not np.isfinite(altitude) else altitude)
            # A timer can publish several recommendations for one image.  Only
            # a fresh target observation is allowed to advance the baseline
            # used for parallax scoring.
            if self._last_recommended_target_stamp != self._target[1]:
                self._previous_view = platform.copy()
                self._last_recommended_target_stamp = self._target[1]
            self._observation_pub.publish(String(data=json.dumps({"schema_version": 1, "sample_timestamp_ns": now,
                                                                   "target_id": identifier, "frame_id": "map", "unit": "m",
                                                                   "control_command": False, "runtime_ms": (perf_counter() - started) * 1000,
                                                                   **recommendation.as_dict()}, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AdaptiveObservationNode()
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
