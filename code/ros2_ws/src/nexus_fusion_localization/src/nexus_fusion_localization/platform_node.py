#!/usr/bin/env python3
"""ROS2 platform estimator; receives data only and never commands a vehicle."""

from dataclasses import replace
import heapq
import json
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from nexus_fusion_localization.imu_preintegration import ImuNoise, ImuReading
from nexus_fusion_localization.platform_measurements import (
    PoseMeasurement, PositionMeasurement, RangeMeasurement, RelativeMotionMeasurement,
)
from nexus_fusion_localization.platform_state import (
    BodyState, pose_covariance_from_ros, pose_covariance_to_ros, rotation_matrix, skew, vector,
)
from nexus_fusion_localization.platform_window import PlatformConfig, PlatformWindow
from nexus_fusion_localization.reliability import ReliabilityMLP, covariance_scales, feature_row


def load_platform_calibration(path, allow_test=False):
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get("schema_version") != 1 or not data.get("calibration_id"):
        raise ValueError("platform calibration requires schema_version=1 and calibration_id")
    status = data.get("calibration_status")
    if status != "measured" and not (allow_test and status == "synthetic_test"):
        raise ValueError("platform requires measured calibration; test calibration needs explicit opt-in")
    if data.get("map_frame") != "map" or data.get("body_frame") != "base_link":
        raise ValueError("platform calibration must name map and base_link")
    imu = data["imu"]
    if not imu.get("frame_id") or imu.get("at_body_origin_or_compensated") is not True:
        raise ValueError("IMU frame and origin/lever-arm compensation must be confirmed")
    rotation_matrix(imu["rotation_body_imu"])
    required_noise = set(ImuNoise.__dataclass_fields__)
    if set(imu["noise"]) != required_noise:
        raise ValueError(f"calibration must specify all IMU noise fields: {sorted(required_noise)}")
    ImuNoise(**imu["noise"])
    for name in ("initial_velocity_sigma_mps", "initial_accel_bias_sigma_mps2", "initial_gyro_bias_sigma_rps"):
        if not np.isfinite(data[name]) or data[name] <= 0:
            raise ValueError(f"{name} must be positive")
    vector(data["uwb"]["tag_body_m"], 3, "UWB lever arm")
    if "vision" in data:
        vector(data["vision"]["camera_body_m"], 3, "visual sensor lever arm")
    for anchor in data["uwb"].get("anchors_map_m", {}).values():
        vector(anchor, 3, "surveyed anchor")
    return data


class PlatformLocalizationNode(Node):
    def __init__(self):
        super().__init__("nexus_platform_localization")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("allow_test_calibration", False)
        self.declare_parameter("max_observation_age_ms", 300.0)
        self.declare_parameter("reorder_delay_ms", 30.0)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("reliability_mode", "fixed")
        self.declare_parameter("reliability_model", "")
        self.declare_parameter("imu_buffer_s", 5.0)
        self.calibration = load_platform_calibration(
            str(self.get_parameter("calibration_file").value), bool(self.get_parameter("allow_test_calibration").value))
        self._rotation_body_imu = rotation_matrix(self.calibration["imu"]["rotation_body_imu"])
        options = dict(self.calibration.get("window", {}))
        self.estimator = PlatformWindow(PlatformConfig(
            **options, imu_noise=ImuNoise(**self.calibration["imu"]["noise"])))
        self._max_age_ns = int(float(self.get_parameter("max_observation_age_ms").value) * 1e6)
        self._reorder_ns = int(float(self.get_parameter("reorder_delay_ms").value) * 1e6)
        self._imu_buffer_ns = int(float(self.get_parameter("imu_buffer_s").value) * 1e9)
        if not 0 <= self._reorder_ns < self._max_age_ns:
            raise ValueError("reorder delay must be nonnegative and below maximum observation age")
        if self._imu_buffer_ns <= self._max_age_ns:
            raise ValueError("IMU cache must exceed maximum observation age")
        self._reliability_mode = str(self.get_parameter("reliability_mode").value)
        if self._reliability_mode not in {"fixed", "rule", "learned"}:
            raise ValueError("invalid platform reliability_mode")
        self._model = (ReliabilityMLP.load(str(self.get_parameter("reliability_model").value))
                       if self._reliability_mode == "learned" else None)
        self._quality = {}
        self._pending, self._sequence = [], 0
        self._last_imu_ns, self._last_status_reason = 0, None
        self._odom_publisher = self.create_publisher(Odometry, "/nexus/platform/odom", 20)
        self._status_publisher = self.create_publisher(String, "/nexus/platform/status", 20)
        self._tf = TransformBroadcaster(self) if self.get_parameter("publish_tf").value else None
        self.create_subscription(Imu, "/nexus/fcu/imu", self._imu_callback, 100)
        self.create_subscription(Odometry, "/nexus/fcu/odom", self._position_callback, 20)
        self.create_subscription(PoseWithCovarianceStamped, "/nexus/platform/initial_pose", self._initial_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/nexus/vision/platform_pose", self._reference_callback, 20)
        self.create_subscription(String, "/nexus/uwb/ranges", self._ranges_callback, 20)
        self.create_subscription(String, "/nexus/vision/body_motion", self._motion_callback, 20)
        self.create_subscription(String, "/nexus/observations/quality", self._quality_callback, 50)
        self.create_timer(.01, self._drain)

    @staticmethod
    def _stamp(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def _check_time(self, stamp):
        now = self.get_clock().now().nanoseconds
        if stamp <= 0 or not 0 <= now - stamp <= self._max_age_ns:
            raise ValueError("platform observation has stale/future/missing sample timestamp")

    def _status(self, reason, *, valid=False, stamp_ns=None, extra=None):
        self._last_status_reason = reason
        data = {"schema_version": 1, "subject": "platform", "valid": bool(valid), "reason": reason,
                "sample_timestamp_ns": stamp_ns, "calibration_id": self.calibration["calibration_id"],
                "reliability_mode": self._reliability_mode,
                "source_last_sample_ns": self.estimator.last_source_stamps, **(extra or {})}
        self._status_publisher.publish(String(data=json.dumps(data, allow_nan=False)))

    def _quality_callback(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or data.get("subject") != "platform":
                return  # shared topic also carries external-target quality
            if data.get("schema_version") != 1 or data.get("source") not in {
                    "uwb_position", "uwb_ranges", "fixed_tag", "superpoint", "imu"}:
                raise ValueError("invalid platform quality schema/source")
            stamp = int(data["sample_timestamp_ns"])
            self._check_time(stamp)
            if not isinstance(data["features"], dict):
                raise ValueError("quality features must be an object")
            feature_row(data["features"])
            now = self.get_clock().now().nanoseconds
            self._quality = {key: value for key, value in self._quality.items() if now - key[1] <= self._max_age_ns}
            self._quality[(data["source"], stamp)] = dict(data["features"])
        except (TypeError, ValueError, KeyError, OverflowError) as error:
            self._status(f"quality_rejected:{error}")

    def _scale(self, source, stamp):
        features = self._quality.pop((source, stamp), {})
        if self._reliability_mode == "fixed":
            return 1.
        if self._reliability_mode == "rule":
            if source.startswith("uwb"):
                return float(np.clip(1. + max(0., float(features.get("gdop", 1.)) - 1.) ** 2, 1., 100.))
            ratio = float(features.get("inlier_ratio", 1.))
            if not np.isfinite(ratio) or not 0 <= ratio <= 1:
                raise ValueError("invalid inlier_ratio for rule reliability")
            return 1. + 99. * (1. - ratio) ** 2
        features["network_latency_s"] = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if self.estimator.states:
            previous = next(reversed(self.estimator.states.values()))
            features["dt_s"] = (stamp - previous.stamp_ns) / 1e9
            features["speed_mps"] = float(np.linalg.norm(previous.velocity_mps))
        scales = covariance_scales(self._model.predict(features))
        return float(scales[2] if source == "imu" else scales[1] if source.startswith("uwb") else scales[0])

    def _enqueue(self, measurement):
        self._check_time(measurement.stamp_ns)
        self._sequence += 1
        heapq.heappush(self._pending, (measurement.stamp_ns, self._sequence, measurement))

    def _imu_callback(self, message):
        try:
            stamp = self._stamp(message.header)
            self._check_time(stamp)
            if message.header.frame_id != self.calibration["imu"]["frame_id"]:
                raise ValueError("IMU frame does not match calibration")
            if message.linear_acceleration_covariance[0] == -1 or message.angular_velocity_covariance[0] == -1:
                raise ValueError("IMU acceleration or angular velocity is marked unavailable")
            accel = vector([message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z], 3, "IMU acceleration")
            gyro = vector([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z], 3, "IMU angular velocity")
            self.estimator.add_imu(ImuReading(stamp, self._rotation_body_imu @ accel, self._rotation_body_imu @ gyro))
            self._last_imu_ns = stamp
            if not self.estimator.states:
                self.estimator.imu.discard_before(stamp - self._max_age_ns)
            else:
                self.estimator.imu.discard_before(stamp - self._imu_buffer_ns)
        except (ValueError, TypeError, OverflowError) as error:
            self._status(f"imu_rejected:{error}")

    def _pose_measurement(self, message):
        stamp = self._stamp(message.header)
        self._check_time(stamp)
        if message.header.frame_id != "map":
            raise ValueError("platform reference must be in map")
        pose = message.pose.pose
        quaternion = vector([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], 4, "platform quaternion")
        if abs(np.linalg.norm(quaternion) - 1.) > 1e-3:
            raise ValueError("platform orientation requires a unit quaternion")
        rotation = Rotation.from_quat(quaternion).as_matrix()
        covariance = pose_covariance_from_ros(rotation, np.array(message.pose.covariance).reshape(6, 6))
        return PoseMeasurement(stamp, [pose.position.x, pose.position.y, pose.position.z], rotation, covariance)

    def _initialize(self, measurement, source):
        body = BodyState(measurement.stamp_ns, measurement.position_m, np.zeros(3), measurement.rotation_map_body)
        indices = [0, 1, 2, 6, 7, 8]
        covariance = np.zeros((15, 15))
        covariance[np.ix_(indices, indices)] = measurement.covariance
        covariance[3:6, 3:6] = np.eye(3) * self.calibration["initial_velocity_sigma_mps"] ** 2
        covariance[9:12, 9:12] = np.eye(3) * self.calibration["initial_accel_bias_sigma_mps2"] ** 2
        covariance[12:15, 12:15] = np.eye(3) * self.calibration["initial_gyro_bias_sigma_rps"] ** 2
        self.estimator.initialize(body, covariance, source)
        self._pending = [entry for entry in self._pending if entry[0] > measurement.stamp_ns]
        heapq.heapify(self._pending)
        self._status("initialized", valid=True, stamp_ns=measurement.stamp_ns)
        self._publish_odom(body, covariance)

    def _initial_callback(self, message):
        try:
            self._initialize(self._pose_measurement(message), "initial_pose")
        except (TypeError, ValueError) as error:
            self._status(f"initial_pose_rejected:{error}")

    def _reference_callback(self, message):
        try:
            measurement = self._pose_measurement(message)
            if not self.estimator.states:
                self._initialize(measurement, "fixed_tag")
            elif (self.estimator.imu.samples
                  and next(reversed(self.estimator.states)) < self.estimator.imu.samples[0].stamp_ns
                  and measurement.stamp_ns > next(reversed(self.estimator.states))):
                # After a long outage there is no complete IMU bridge to the
                # old state. A new surveyed full pose can explicitly re-seed;
                # UWB position alone cannot fabricate the missing orientation.
                self._initialize(measurement, "fixed_tag_reinitialization")
            else:
                self._enqueue(measurement)
        except (TypeError, ValueError) as error:
            self._status(f"reference_rejected:{error}")

    def _position_callback(self, message):
        try:
            if message.header.frame_id != "map" or message.child_frame_id != "base_link":
                raise ValueError("UWB odometry must identify map and base_link")
            position = message.pose.pose.position
            matrix = np.array(message.pose.covariance).reshape(6, 6)[:3, :3]
            self._enqueue(PositionMeasurement(self._stamp(message.header), [position.x, position.y, position.z],
                                              matrix, self.calibration["uwb"]["tag_body_m"]))
        except (TypeError, ValueError) as error:
            self._status(f"position_rejected:{error}")

    def _ranges_callback(self, message):
        try:
            data = json.loads(message.data)
            if (not isinstance(data, dict) or data.get("schema_version") != 1
                    or data.get("frame_id") != "map" or data.get("unit") != "m"):
                raise ValueError("range packet requires schema=1, map, metres")
            ids = [str(value) for value in data["anchor_ids"]]
            if len(set(ids)) != len(ids):
                raise ValueError("duplicate range anchor ID")
            surveyed = self.calibration["uwb"]["anchors_map_m"]
            anchors = [surveyed[identifier] for identifier in ids]
            variances = vector(data["variances_m2"], len(ids), "range variances")
            self._enqueue(RangeMeasurement(int(data["sample_timestamp_ns"]), anchors, data["ranges_m"],
                                           np.diag(variances), self.calibration["uwb"]["tag_body_m"]))
        except (TypeError, ValueError, KeyError, OverflowError) as error:
            self._status(f"ranges_rejected:{error}")

    def _motion_callback(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or data.get("schema_version") != 1 or data.get("frame_id") != "base_link":
                raise ValueError("relative body motion requires schema=1 and base_link")
            sensor_body = vector(data.get("sensor_body_m", [0., 0., 0.]), 3, "visual sensor lever arm")
            expected = self.calibration.get("vision", {}).get("camera_body_m", [0., 0., 0.])
            if not np.allclose(sensor_body, expected, atol=1e-9):
                raise ValueError("visual sensor lever arm does not match platform calibration")
            self._enqueue(RelativeMotionMeasurement(
                int(data["previous_stamp_ns"]), int(data["sample_timestamp_ns"]),
                data["rotation_i_j"], data["translation_i_j"],
                np.asarray(data["covariance"]).reshape(6, 6), data["metric_translation"], sensor_body_m=sensor_body))
        except (TypeError, ValueError, KeyError, OverflowError) as error:
            self._status(f"motion_rejected:{error}")

    def _drain(self):
        now = self.get_clock().now().nanoseconds
        while self._pending and now - self._pending[0][0] > self._max_age_ns:
            old = heapq.heappop(self._pending)
            self._status("queued_observation_stale", stamp_ns=old[0])
        if self._last_imu_ns and now - self._last_imu_ns > self._max_age_ns:
            if self._last_status_reason != "imu_stale":
                self._status("imu_stale", stamp_ns=self._last_imu_ns)
        if not self.estimator.states:
            return
        if not self._last_imu_ns and self._last_status_reason != "imu_missing":
            self._status("imu_missing")
        while self._pending:
            stamp = self._pending[0][0]
            if now - stamp > self._max_age_ns:
                heapq.heappop(self._pending)
                self._status("queued_observation_stale", stamp_ns=stamp)
                continue
            if stamp > min(now - self._reorder_ns, self._last_imu_ns):
                break
            observations = []
            while self._pending and self._pending[0][0] == stamp:
                observations.append(heapq.heappop(self._pending)[2])
            try:
                observations = [replace(item, covariance_scale=self._scale(item.source, stamp)) for item in observations]
                result = self.estimator.step(stamp, observations, imu_covariance_scale=self._scale("imu", stamp))
            except (TypeError, ValueError) as error:
                self._status(f"quality_inference_failed:{error}", stamp_ns=stamp)
                continue
            now = self.get_clock().now().nanoseconds
            fresh = 0 <= now - result.state.stamp_ns <= self._max_age_ns
            reason = result.reason if fresh else "solution_stale"
            self._status(reason, valid=result.valid and not result.prediction_only and fresh, stamp_ns=stamp,
                         extra={"prediction_only": result.prediction_only, "window_stamps": result.window_stamps,
                                "runtime_ms": result.runtime_ms, "diagnostics": result.diagnostics,
                                "accel_bias_mps2": result.state.accel_bias_mps2.tolist(),
                                "gyro_bias_rps": result.state.gyro_bias_rps.tolist()})
            if result.valid and not result.prediction_only and fresh:
                self._publish_odom(result.state, result.covariance)
        last_constraint = self.estimator.last_external_ns
        if (now - last_constraint) / 1e9 > self.estimator.config.prediction_horizon_s:
            if self._last_status_reason not in {"external_constraints_lost", "imu_stale"}:
                self._status("external_constraints_lost", stamp_ns=last_constraint)

    def _publish_odom(self, state, covariance):
        output = Odometry()
        output.header.stamp.sec, output.header.stamp.nanosec = divmod(state.stamp_ns, 1_000_000_000)
        output.header.frame_id, output.child_frame_id = "map", "base_link"
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = state.position_m.tolist()
        q = Rotation.from_matrix(state.rotation_map_body).as_quat().tolist()
        output.pose.pose.orientation.x, output.pose.pose.orientation.y, output.pose.pose.orientation.z, output.pose.pose.orientation.w = q
        output.pose.covariance = pose_covariance_to_ros(state, covariance).reshape(-1).tolist()
        # nav_msgs/Odometry twist is expressed in child_frame_id, not map.
        velocity = state.rotation_map_body.T @ state.velocity_mps
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = velocity.tolist()
        twist_jacobian = np.zeros((6, 15))
        twist_jacobian[:3, 3:6] = state.rotation_map_body.T
        twist_jacobian[:3, 6:9] = skew(velocity)
        twist_jacobian[3:, 12:15] = -np.eye(3)
        twist_covariance = twist_jacobian @ covariance @ twist_jacobian.T
        samples = self.estimator.imu.samples
        index = int(np.searchsorted([item.stamp_ns for item in samples], state.stamp_ns))
        if len(samples) >= 2 and samples[0].stamp_ns <= state.stamp_ns <= samples[-1].stamp_ns:
            index = min(max(index, 1), len(samples) - 1)
            first, second = samples[index - 1], samples[index]
            fraction = (state.stamp_ns - first.stamp_ns) / (second.stamp_ns - first.stamp_ns)
            gyro = first.angular_velocity_rps * (1 - fraction) + second.angular_velocity_rps * fraction - state.gyro_bias_rps
            output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = gyro.tolist()
            dt = (second.stamp_ns - first.stamp_ns) / 1e9
            gyro_variance = self.estimator.config.imu_noise.gyro_density ** 2 / dt * ((1 - fraction) ** 2 + fraction ** 2)
            twist_covariance[3:, 3:] += np.eye(3) * gyro_variance
        else:
            # Initial pose may precede the first IMU packet; unknown angular
            # rate is explicit, never a fabricated measured zero.
            output.twist.twist.angular.x = output.twist.twist.angular.y = output.twist.twist.angular.z = float("nan")
            twist_covariance[3:, :] = twist_covariance[:, 3:] = float("nan")
        output.twist.covariance = twist_covariance.reshape(-1).tolist()
        self._odom_publisher.publish(output)
        if self._tf is not None:
            transform = TransformStamped()
            transform.header, transform.child_frame_id = output.header, output.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = state.position_m.tolist()
            transform.transform.rotation = output.pose.pose.orientation
            self._tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
