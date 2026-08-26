#!/usr/bin/env python3
"""NEXUS UWB simulation adapter.

Subscribes to Gazebo Ground Truth odometry, generates simulated UWB ranges,
calls the AlgorithmRouter's registered Trilateration runner, and publishes a
TargetObservation for the Dashboard / fusion pipeline.  Ground Truth is used
only for range generation and evaluation; it never enters the algorithm input.
"""

from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node

from nexus_msgs.msg import TargetObservation  # noqa: F401 (ROS2 runtime only)


def _resolve_analysis_root():
    """Return the code/analysis path so the router and algorithms can be imported."""
    candidates = [
        Path(__file__).resolve().parents[6] / "code" / "analysis",
        Path("/home/li/NEXUS/code/analysis"),
    ]
    for candidate in candidates:
        if (candidate / "algorithms" / "router.py").is_file():
            return candidate
    raise RuntimeError("cannot locate code/analysis; set PYTHONPATH or install analysis tools")


import sys  # noqa: E402

_ANALYSIS_ROOT = _resolve_analysis_root()
sys.path.insert(0, str(_ANALYSIS_ROOT))
sys.path.insert(0, str(_ANALYSIS_ROOT.parent.parent / "simulation"))


class UwbSimulationNode(Node):
    def __init__(self):
        super().__init__("nexus_uwb_simulation")
        self.declare_parameter("config_file", "")
        self.declare_parameter("odom_topic", "/nexus/gazebo/uav/odom")
        self.declare_parameter("output_topic", "/nexus/uwb/target_observation")
        self.declare_parameter("algorithm", "uwb.matlab.trilateration")
        self.declare_parameter("sigma_m", 0.0)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("configured_height_m", float("nan"))

        config_file = str(self.get_parameter("config_file").value)
        if not config_file or not Path(config_file).is_file():
            raise RuntimeError(f"UWB config file not found: '{config_file}'")
        with open(config_file, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        noise_config = config.get("range_noise", {})

        self._anchors = [
            (entry["id"], tuple(float(v) for v in entry["position_m"]))
            for entry in config.get("anchors", [])
        ]
        if len(self._anchors) < 3:
            raise ValueError("at least three anchors are required")
        self._sigma = float(noise_config.get("sigma_m", 0.0))
        self._seed = int(noise_config.get("random_seed", 42))
        self._rng = np.random.default_rng(self._seed)
        self._algorithm_name = str(self.get_parameter("algorithm").value)
        self._configured_height = float(self.get_parameter("configured_height_m").value)

        from algorithms.router import AlgorithmRouter, register_default_algorithms
        register_default_algorithms()
        self._router = AlgorithmRouter(self._algorithm_name)

        self._publisher = self.create_publisher(
            TargetObservation, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self.on_ground_truth, 10)
        self.get_logger().info(
            f"UWB simulation started: algorithm={self._algorithm_name} "
            f"anchors={len(self._anchors)} sigma={self._sigma}")

    @property
    def last_result(self):
        return getattr(self, "_last_result", None)

    def on_ground_truth(self, message):
        position = message.pose.pose.position
        gt_xy = np.array([position.x, position.y])
        tag_z = position.z if np.isfinite(position.z) else 0.0

        ranges = []
        for anchor_id, anchor_pos in self._anchors:
            distance_3d = float(np.linalg.norm(np.array(anchor_pos) - [position.x, position.y, tag_z]))
            noise = self._rng.normal(0.0, self._sigma) if self._sigma > 0 else 0.0
            ranges.append(max(0.0, distance_3d + noise))

        from .observation_frame import ObservationFrame as _Frame
        observation_frame = _Frame(ranges, [anchor[1] for anchor in self._anchors])
        result = self._router.run(observation_frame=observation_frame)
        self._last_gt = gt_xy
        self._last_result = result

        output = TargetObservation()
        output.header.stamp = message.header.stamp
        output.header.frame_id = "map"
        output.receive_timestamp_ns = self.get_clock().now().nanoseconds
        output.target_id = "uav_tag"
        output.source_mode = TargetObservation.SOURCE_DIRECT_UWB
        output.validity = (
            TargetObservation.VALIDITY_VALID if result.valid
            else TargetObservation.VALIDITY_INVALID)
        output.unit = "m"
        if result.valid:
            output.pose.position.x = float(result.estimate[0])
            output.pose.position.y = float(result.estimate[1])
            z = self._configured_height if np.isfinite(self._configured_height) else 0.0
            output.pose.position.z = z
            output.pose.orientation.w = 1.0
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = UwbSimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
