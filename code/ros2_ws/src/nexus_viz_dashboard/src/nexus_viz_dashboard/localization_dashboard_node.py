#!/usr/bin/env python3
"""Read-only dual localization display with source-clock freshness and history."""

from copy import deepcopy
import json
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from nexus_msgs.msg import TargetKinematicState
from nexus_coord_transform.geometry import matrix_to_quaternion


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def point_list(point):
    values = [float(point.x), float(point.y), float(point.z)]
    return values if np.all(np.isfinite(values)) else None


def quaternion_list(q):
    values = [float(q.x), float(q.y), float(q.z), float(q.w)]
    return values if np.all(np.isfinite(values)) and np.isclose(np.linalg.norm(values), 1., atol=1e-5) else None


class LocalizationDashboardNode(Node):
    def __init__(self):
        super().__init__("nexus_localization_dashboard")
        for name, default in {"input_mode": "LIVE", "run_id": "UNREGISTERED", "max_age_ms": 300.,
                              "maximum_targets": 64}.items():
            self.declare_parameter(name, default)
        self.mode = self.get_parameter("input_mode").value
        self.run_id = self.get_parameter("run_id").value
        self.max_age_ns = int(self.get_parameter("max_age_ms").value * 1e6)
        self.maximum_targets = int(self.get_parameter("maximum_targets").value)
        if (self.mode not in {"LIVE", "REPLAY", "SIMULATION"} or self.max_age_ns <= 0 or self.maximum_targets <= 0
                or not self.run_id or self.mode == "REPLAY" and self.run_id == "UNREGISTERED"):
            raise ValueError("invalid display mode/run/freshness configuration")
        self.targets, self.platform = {}, None
        self.session_id = str(uuid.uuid4())
        self._last_now = 0
        self.snapshot_publisher = self.create_publisher(String, "/nexus/viz/localization_state", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/nexus/viz/localization_markers", 10)
        self.create_subscription(Odometry, "/nexus/platform/odom", self.on_platform, 30)
        self.create_subscription(TargetKinematicState, "/nexus/target/kinematics", self.on_target, 30)
        self.create_timer(.1, self.publish_state)

    def on_platform(self, message):
        stamp = stamp_ns(message.header)
        now = self.get_clock().now().nanoseconds
        if (message.header.frame_id != "map" or message.child_frame_id != "base_link" or not 0 < stamp <= now
                or self.platform is not None and stamp <= stamp_ns(self.platform.header)
                or point_list(message.pose.pose.position) is None or quaternion_list(message.pose.pose.orientation) is None):
            return
        self.platform = deepcopy(message)

    def on_target(self, message):
        stamp, observed = stamp_ns(message.header), int(message.last_valid_sample_timestamp_ns)
        now = self.get_clock().now().nanoseconds
        if (message.header.frame_id != "map" or message.unit != "m" or not message.target_id
                or not 0 < stamp <= now or not 0 <= observed <= stamp
                or message.valid and (message.historical or not message.position_observed or observed != stamp)
                or message.historical and (message.valid or message.position_observed or message.velocity_observed or message.orientation_observed)):
            return
        if message.valid or message.historical:
            covariance = np.array(message.covariance).reshape(9, 9)[:3, :3]
            if (not observed or not message.position_reference or point_list(message.pose.position) is None
                    or not np.all(np.isfinite(covariance)) or not np.allclose(covariance, covariance.T)
                    or np.linalg.eigvalsh(covariance).min() < 0):
                return
        old = self.targets.get(message.target_id)
        # A fresh observation can precede a previously received status header.
        # Compare the actual observation epoch first, retaining nanosecond ints.
        if old is not None and (observed, stamp, not message.valid) <= (
                int(old.last_valid_sample_timestamp_ns), stamp_ns(old.header), not old.valid):
            return
        if old is None and len(self.targets) >= self.maximum_targets:
            return
        self.targets[message.target_id] = deepcopy(message)

    def snapshot(self, now):
        targets = []
        for identifier, message in sorted(self.targets.items()):
            observed = int(message.last_valid_sample_timestamp_ns)
            age = (now - observed) / 1e9 if 0 < observed <= now else None
            current = bool(message.valid and age is not None and age * 1e9 <= self.max_age_ns)
            available = message.valid or message.historical
            position = point_list(message.pose.position) if available else None
            covariance = np.array(message.covariance).reshape(9, 9)[:3, :3] if available else None
            targets.append({
                "target_id": identifier, "source": message.source, "position_reference": message.position_reference,
                "state": message.state if current else "lost", "reason": message.reason or ("" if current else "target_not_observed"),
                "display_state": "observed" if current else "historical" if position is not None else "lost",
                "sample_timestamp_ns": str(stamp_ns(message.header)), "last_valid_sample_timestamp_ns": str(observed),
                "age_s": age, "position": position if current else None,
                "historical_position": position if not current else None,
                "sigma_m": np.sqrt(np.diag(covariance)).tolist() if current else None,
                "position_covariance": covariance.reshape(-1).tolist() if covariance is not None else None,
                "orientation": quaternion_list(message.pose.orientation) if available else None,
            })
        platform = {"display_state": "no_input", "position": None, "orientation": None,
                    "speed_mps": None, "age_s": None, "sample_timestamp_ns": None}
        if self.platform is not None:
            stamp = stamp_ns(self.platform.header)
            current = 0 <= now - stamp <= self.max_age_ns
            speed = point_list(self.platform.twist.twist.linear)
            platform.update({"display_state": "observed" if current else "stale",
                             "position": point_list(self.platform.pose.pose.position) if current else None,
                             "orientation": quaternion_list(self.platform.pose.pose.orientation) if current else None,
                             "speed_mps": float(np.linalg.norm(speed)) if current and speed is not None else None,
                             "age_s": (now - stamp) / 1e9 if now >= stamp else None, "sample_timestamp_ns": str(stamp)})
        return {"schema_version": 1, "session_id": self.session_id, "generated_timestamp_ns": str(now),
                "input_mode": self.mode if self.platform is not None or targets else "NO_INPUT", "run_id": self.run_id,
                "frame_id": "map", "unit": "m", "covariance_assumption": "conditional_on_platform_and_mount",
                "platform": platform, "targets": targets}

    def markers(self, snapshot, now):
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        def add(namespace, position, color, label, orientation=None, covariance=None):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp.sec, marker.header.stamp.nanosec = divmod(now, 1_000_000_000)
            marker.ns, marker.id, marker.type, marker.action = namespace, 0, Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            marker.pose.orientation.w = 1.  # sphere geometry; not an inferred target attitude
            marker.scale.x = marker.scale.y = marker.scale.z = .12
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            marker.lifetime.nanosec = 500_000_000
            if orientation is not None and namespace == "platform":
                marker.type = Marker.ARROW
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = orientation
                marker.scale.x, marker.scale.y, marker.scale.z = .3, .06, .06
            output.markers.append(marker)
            text = deepcopy(marker)
            text.id, text.type, text.text = 1, Marker.TEXT_VIEW_FACING, label
            text.pose.position.z += .18
            text.pose.orientation.x = text.pose.orientation.y = text.pose.orientation.z = 0.
            text.pose.orientation.w = 1.
            text.scale.z, text.color.a = .1, 1.
            output.markers.append(text)
            if covariance is not None:
                eigenvalues, axes = np.linalg.eigh(np.array(covariance).reshape(3, 3))
                if eigenvalues.min() > 0:
                    if np.linalg.det(axes) < 0:
                        axes[:, 0] *= -1
                    ellipse = deepcopy(marker)
                    ellipse.id, ellipse.type = 2, Marker.SPHERE
                    q = matrix_to_quaternion(axes)
                    ellipse.pose.orientation.x, ellipse.pose.orientation.y, ellipse.pose.orientation.z, ellipse.pose.orientation.w = q
                    ellipse.scale.x, ellipse.scale.y, ellipse.scale.z = (2 * np.sqrt(eigenvalues)).tolist()
                    ellipse.color.a = .18
                    output.markers.append(ellipse)

        platform = snapshot["platform"]
        if platform["position"] is not None:
            add("platform", platform["position"], (.18, .44, .65, 1.), "UAV / observed", platform["orientation"])
        for target in snapshot["targets"]:
            current = target["position"] is not None
            position = target["position"] if current else target["historical_position"]
            if position is not None:
                label = f'{target["target_id"]} / {target["state"] if current else "HISTORY - not observed"}'
                color = (.17, .51, .49, 1.) if current else (.55, .47, .34, .45)
                add("target/" + target["target_id"], position, color, label, covariance=target["position_covariance"] if current else None)
        return output

    def publish_state(self):
        now = self.get_clock().now().nanoseconds
        if now < self._last_now:
            self.targets, self.platform = {}, None
            self.session_id = str(uuid.uuid4())
        self._last_now = now
        snapshot = self.snapshot(now)
        self.snapshot_publisher.publish(String(data=json.dumps(snapshot, allow_nan=False)))
        self.marker_publisher.publish(self.markers(snapshot, now))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
