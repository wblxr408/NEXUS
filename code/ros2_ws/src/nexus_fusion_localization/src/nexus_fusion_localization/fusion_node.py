#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from nexus_msgs.msg import TargetObservation

from nexus_fusion_localization.fusion import Observation, fuse_observations


class FusionLocalizationNode(Node):
    def __init__(self):
        super().__init__("nexus_fusion_localization")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("max_age_ms", 250.0)
        self.declare_parameter("max_pair_delta_ms", 50.0)
        self.declare_parameter("vision_topic", "/nexus/vision/map_target_observation")
        self.declare_parameter("uwb_topic", "/nexus/uwb/target_observation")
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._max_age_ns = int(float(self.get_parameter("max_age_ms").value) * 1e6)
        self._max_pair_delta_ns = int(
            float(self.get_parameter("max_pair_delta_ms").value) * 1e6)
        self._observations = {}
        self._last_valid_sample_ns = {}
        self._last_output_invalid_reason = {}
        self._publisher = self.create_publisher(TargetObservation, "/nexus/target/pose", 10)
        self.create_subscription(
            TargetObservation, str(self.get_parameter("vision_topic").value), self._callback, 10)
        self.create_subscription(TargetObservation, str(self.get_parameter("uwb_topic").value), self._callback, 10)
        self.create_timer(max(float(self.get_parameter("max_age_ms").value) / 2000.0, 0.05),
                          self._stale_watchdog)

    def _callback(self, message):
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
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
            confidence=float(message.confidence),
            source_mode=int(message.source_mode),
            orientation_xyzw=np.array([
                message.pose.orientation.x, message.pose.orientation.y,
                message.pose.orientation.z, message.pose.orientation.w,
            ], dtype=float),
            receive_timestamp_ns=int(message.receive_timestamp_ns),
            validity=int(message.validity),
            invalid_reason=str(message.invalid_reason),
            unit=str(message.unit),
        )
        self._observations[(record.target_id, record.source_mode)] = record
        now_ns = self.get_clock().now().nanoseconds
        same_target = [
            item for item in self._observations.values()
            if item.target_id == record.target_id
        ]
        result = fuse_observations(
            same_target, self._target_frame, now_ns,
            self._max_age_ns, self._max_pair_delta_ns)
        if result is None:
            self.get_logger().warning("no valid target observation available for fusion")
            self._publish_invalid(message, "no_valid_observation")
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
            self.get_logger().warning(
                "dual-source observations exceeded max_pair_delta_ms; publishing newest single source")

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
