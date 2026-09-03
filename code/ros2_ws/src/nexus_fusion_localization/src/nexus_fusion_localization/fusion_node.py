#!/usr/bin/env python3
import json

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nexus_msgs.msg import TargetObservation

from nexus_fusion_localization.fusion import Observation, fuse_observations, validate_observation
from nexus_fusion_localization.robust_filter import RobustFilterConfig, RobustTargetFilter
from nexus_fusion_localization.reliability import ReliabilityMLP, covariance_scales, feature_row


class FusionLocalizationNode(Node):
    def __init__(self):
        super().__init__("nexus_fusion_localization")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("max_age_ms", 250.0)
        self.declare_parameter("max_pair_delta_ms", 50.0)
        self.declare_parameter("vision_topic", "/nexus/vision/map_target_observation")
        self.declare_parameter("uwb_topic", "/nexus/uwb/target_observation")
        self.declare_parameter("fusion_mode", "robust")
        self.declare_parameter("prediction_horizon_ms", 500.0)
        self.declare_parameter("acceleration_sigma_mps2", 2.0)
        self.declare_parameter("initial_velocity_sigma_mps", 1.0)
        self.declare_parameter("innovation_soft_d2", 7.815)
        self.declare_parameter("innovation_hard_d2", 16.266)
        self.declare_parameter("confirmation_hits", 3)
        self.declare_parameter("maximum_speed_mps", 10.0)
        self.declare_parameter("reliability_mode", "fixed")
        self.declare_parameter("reliability_model", "")
        self.declare_parameter("quality_topic", "/nexus/observations/quality")
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._max_age_ns = int(float(self.get_parameter("max_age_ms").value) * 1e6)
        self._max_pair_delta_ns = int(
            float(self.get_parameter("max_pair_delta_ms").value) * 1e6)
        self._fusion_mode = str(self.get_parameter("fusion_mode").value)
        if self._fusion_mode not in {"robust", "information_baseline"}:
            raise ValueError("fusion_mode must be robust or information_baseline")
        self._filter = RobustTargetFilter(RobustFilterConfig(
            expected_frame=self._target_frame, max_age_ns=self._max_age_ns,
            prediction_horizon_ns=int(float(self.get_parameter("prediction_horizon_ms").value) * 1e6),
            acceleration_sigma_mps2=float(self.get_parameter("acceleration_sigma_mps2").value),
            initial_velocity_sigma_mps=float(self.get_parameter("initial_velocity_sigma_mps").value),
            soft_d2=float(self.get_parameter("innovation_soft_d2").value),
            hard_d2=float(self.get_parameter("innovation_hard_d2").value),
            confirmation_hits=int(self.get_parameter("confirmation_hits").value),
            maximum_speed_mps=float(self.get_parameter("maximum_speed_mps").value),
        ))
        self._reliability_mode = str(self.get_parameter("reliability_mode").value)
        if self._reliability_mode not in {"fixed", "rule", "learned"}:
            raise ValueError("reliability_mode must be fixed, rule, or learned")
        self._model = (ReliabilityMLP.load(str(self.get_parameter("reliability_model").value))
                       if self._reliability_mode == "learned" else None)
        self._quality_features = {}
        self._observations = {}
        self._last_valid_sample_ns = {}
        self._last_output_invalid_reason = {}
        self._publisher = self.create_publisher(TargetObservation, "/nexus/target/pose", 10)
        self._quality_publisher = self.create_publisher(String, "/nexus/target/fusion_status", 10)
        self.create_subscription(String, str(self.get_parameter("quality_topic").value),
                                 self._quality_callback, 10)
        self.create_subscription(
            TargetObservation, str(self.get_parameter("vision_topic").value), self._callback, 10)
        self.create_subscription(TargetObservation, str(self.get_parameter("uwb_topic").value), self._callback, 10)
        self.create_timer(max(float(self.get_parameter("max_age_ms").value) / 2000.0, 0.05),
                          self._stale_watchdog)

    def _quality_callback(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict) or payload.get("schema_version") != 1:
                raise ValueError("quality schema_version must be 1")
            if payload.get("subject") == "platform":
                return  # the shared topic also serves the platform estimator
            key = (str(payload["target_id"]), int(payload["source_mode"]),
                   int(payload["sample_timestamp_ns"]))
            now_ns = self.get_clock().now().nanoseconds
            if not key[0] or key[1] not in {1, 2, 3, 4} or not 0 <= now_ns - key[2] <= self._max_age_ns:
                raise ValueError("invalid or stale quality identity/time")
            values = payload["features"]
            if not isinstance(values, dict):
                raise ValueError("quality features must be an object")
            feature_row(values)
            self._quality_features = {key: values for key, values in self._quality_features.items()
                                      if now_ns - key[2] <= self._max_age_ns}
            self._quality_features[key] = values
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.get_logger().warning(f"rejecting malformed quality observation: {error}")

    def _covariance_scale(self, record):
        if not validate_observation(record, self._target_frame):
            # Malformed measurements must reach the hard rejection path without
            # being interpreted by the model as missing quality features.
            return 1.0
        if self._reliability_mode == "fixed":
            return 1.0
        if self._reliability_mode == "rule":
            return float(1.0 + 99.0 * (1.0 - record.confidence) ** 2)
        values = dict(self._quality_features.pop(
            (record.target_id, record.source_mode, record.stamp_ns), {}))
        values["network_latency_s"] = (record.receive_timestamp_ns - record.stamp_ns) / 1e9
        track = self._filter.tracks.get(record.target_id)
        if track is not None:
            values["dt_s"] = (record.stamp_ns - track.stamp_ns) / 1e9
            values["speed_mps"] = float(np.linalg.norm(track.x[3:]))
            predicted = self._filter.predict(record.target_id, record.stamp_ns)
            if record.source_mode == 1 and predicted is not None:
                values["uwb_innovation_m"] = float(np.linalg.norm(record.position - predicted["state"][:3]))
        scales = covariance_scales(self._model.predict(values))
        return float(scales[1] if record.source_mode == 1 else scales[0])

    def _callback(self, message):
        try:
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            if len(message.covariance) != 36:
                raise ValueError("covariance must have 36 values")
            record = Observation(
                target_id=message.target_id,
                frame_id=message.header.frame_id,
                stamp_ns=stamp_ns,
                position=np.array([message.pose.position.x, message.pose.position.y, message.pose.position.z], dtype=float),
                # geometry_msgs covariance is row-major 6x6 (x/y/z/r/p/yaw). Keep
                # only its positional 3x3 block for the fusion contract.
                covariance=np.array(
                    [message.covariance[index] for index in (0, 1, 2, 6, 7, 8, 12, 13, 14)],
                    dtype=float,
                ),
                confidence=float(message.confidence), source_mode=int(message.source_mode),
                orientation_xyzw=np.array([
                    message.pose.orientation.x, message.pose.orientation.y,
                    message.pose.orientation.z, message.pose.orientation.w,
                ], dtype=float),
                receive_timestamp_ns=int(message.receive_timestamp_ns), validity=int(message.validity),
                invalid_reason=str(message.invalid_reason), unit=str(message.unit),
            )
        except (TypeError, ValueError, OverflowError) as error:
            self.get_logger().warning(f"rejecting malformed target observation: {error}")
            self._publish_invalid(message, "malformed_observation")
            return
        now_ns = self.get_clock().now().nanoseconds
        if self._fusion_mode == "robust":
            scale = self._covariance_scale(record)
            result = self._filter.update(record, now_ns, covariance_scale=scale)
            diagnostic = String()
            diagnostic.data = json.dumps({
                "schema_version": 1, "sample_timestamp_ns": record.stamp_ns,
                "reliability_mode": self._reliability_mode,
                "counts": self._filter.counts, **self._filter.last_diagnostic,
            }, allow_nan=False)
            self._quality_publisher.publish(diagnostic)
            if result is not None:
                self._observations[(record.target_id, record.source_mode)] = record
        else:
            self._observations[(record.target_id, record.source_mode)] = record
            same_target = [item for item in self._observations.values()
                           if item.target_id == record.target_id]
            result = fuse_observations(same_target, self._target_frame, now_ns,
                                       self._max_age_ns, self._max_pair_delta_ns)
        if result is None:
            self.get_logger().warning("no valid target observation available for fusion")
            reason = (self._filter.last_diagnostic["reason"] if self._fusion_mode == "robust"
                      else "no_valid_observation")
            self._publish_invalid(message, reason)
            return
        output = TargetObservation()
        output.header.stamp.sec = result["stamp_ns"] // 1_000_000_000
        output.header.stamp.nanosec = result["stamp_ns"] % 1_000_000_000
        output.header.frame_id = result["frame_id"]
        output.receive_timestamp_ns = max(now_ns, result["stamp_ns"])
        output.target_id = result["target_id"]
        output.source_mode = result["source_mode"]
        output.validity = TargetObservation.VALIDITY_VALID
        output.invalid_reason = ""
        output.last_valid_sample_timestamp_ns = result["stamp_ns"]
        output.unit = "m"
        output.pose.position.x = float(result["position"][0])
        output.pose.position.y = float(result["position"][1])
        output.pose.position.z = float(result["position"][2])
        output.pose.orientation.x = float(result["orientation_xyzw"][0])
        output.pose.orientation.y = float(result["orientation_xyzw"][1])
        output.pose.orientation.z = float(result["orientation_xyzw"][2])
        output.pose.orientation.w = float(result["orientation_xyzw"][3])
        covariance = np.zeros((6, 6))
        covariance[:3, :3] = result["covariance"]
        output.covariance = covariance.reshape(-1).tolist()
        output.confidence = result["confidence"]
        self._publisher.publish(output)
        self._last_valid_sample_ns[result["target_id"]] = result["stamp_ns"]
        self._last_output_invalid_reason[result["target_id"]] = ""
        if result["degraded_reason"]:
            self.get_logger().warning(result["degraded_reason"])

    def _publish_invalid(self, triggering_message, reason):
        output = TargetObservation()
        output.header = triggering_message.header
        output.header.frame_id = self._target_frame
        sample_ns = max(
            int(triggering_message.header.stamp.sec) * 1_000_000_000
            + int(triggering_message.header.stamp.nanosec), 1)
        output.header.stamp.sec = sample_ns // 1_000_000_000
        output.header.stamp.nanosec = sample_ns % 1_000_000_000
        output.receive_timestamp_ns = max(self.get_clock().now().nanoseconds, sample_ns)
        output.target_id = triggering_message.target_id
        output.source_mode = triggering_message.source_mode
        output.validity = TargetObservation.VALIDITY_INVALID
        output.invalid_reason = str(reason)
        output.last_valid_sample_timestamp_ns = self._last_valid_sample_ns.get(
            triggering_message.target_id, 0)
        output.unit = "m"
        output.pose.position.x = float("nan")
        output.pose.position.y = float("nan")
        output.pose.position.z = float("nan")
        output.pose.orientation.x = float("nan")
        output.pose.orientation.y = float("nan")
        output.pose.orientation.z = float("nan")
        output.pose.orientation.w = float("nan")
        output.covariance = [float("nan")] * 36
        output.confidence = 0.0
        self._publisher.publish(output)
        self._last_output_invalid_reason[triggering_message.target_id] = str(reason)

    def _stale_watchdog(self):
        if not self._observations:
            return
        now_ns = self.get_clock().now().nanoseconds
        target_ids = {item.target_id for item in self._observations.values() if item.target_id}
        for target_id in target_ids:
            target_observations = [
                item for item in self._observations.values()
                if item.target_id == target_id
            ]
            if any(
                    item.validity == TargetObservation.VALIDITY_VALID
                    and not (item.stamp_ns <= 0 or now_ns < item.stamp_ns
                             or now_ns - item.stamp_ns > self._max_age_ns)
                    for item in target_observations):
                continue
            if self._last_output_invalid_reason.get(target_id) == "stale":
                continue
            latest = max(target_observations, key=lambda item: item.stamp_ns)
            trigger = TargetObservation()
            trigger.header.stamp.sec = latest.stamp_ns // 1_000_000_000
            trigger.header.stamp.nanosec = latest.stamp_ns % 1_000_000_000
            trigger.header.frame_id = latest.frame_id
            trigger.target_id = latest.target_id
            trigger.source_mode = latest.source_mode
            self._publish_invalid(trigger, "stale")


def main(args=None):
    rclpy.init(args=args)
    node = FusionLocalizationNode()
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
