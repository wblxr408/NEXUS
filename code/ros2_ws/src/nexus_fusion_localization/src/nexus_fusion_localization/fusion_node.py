#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from nexus_msgs.msg import TargetObservation

from .fusion import Observation, fuse_observations


class FusionLocalizationNode(Node):
    def __init__(self):
        super().__init__("nexus_fusion_localization")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("max_age_ms", 250.0)
        self.declare_parameter("uwb_topic", "/nexus/uwb/target_observation")
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._max_age_ns = int(float(self.get_parameter("max_age_ms").value) * 1e6)
        self._observations = {}
        self._publisher = self.create_publisher(TargetObservation, "/nexus/target/pose", 10)
        self.create_subscription(TargetObservation, "/nexus/vision/target_observation", self._callback, 10)
        self.create_subscription(TargetObservation, str(self.get_parameter("uwb_topic").value), self._callback, 10)

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
        )
        self._observations[(record.target_id, record.source_mode)] = record
        now_ns = self.get_clock().now().nanoseconds
        result = fuse_observations(list(self._observations.values()), self._target_frame, now_ns, self._max_age_ns)
        if result is None:
            self.get_logger().warning("no valid target observation available for fusion")
            return
        output = TargetObservation()
        output.header.stamp.sec = result["stamp_ns"] // 1_000_000_000
        output.header.stamp.nanosec = result["stamp_ns"] % 1_000_000_000
        output.header.frame_id = result["frame_id"]
        output.target_id = result["target_id"]
        output.source_mode = result["source_mode"]
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


def main(args=None):
    rclpy.init(args=args)
    node = FusionLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
