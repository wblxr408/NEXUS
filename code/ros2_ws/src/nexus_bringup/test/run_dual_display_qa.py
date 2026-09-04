#!/usr/bin/env python3
"""Opt-in synthetic DDS/rosbridge/RViz fixture; never publishes control topics.

Run in a sourced ROS workspace with the same transport as all child nodes.
The browser verifier writes phase.txt; JSONL records actual received messages.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import cv2
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nexus_msgs.msg import TargetKinematicState
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


def executable(package, name):
    return str(Path(get_package_prefix(package)) / "lib" / package / name)


class Fixture(Node):
    def __init__(self, destination):
        super().__init__("nexus_synthetic_display_fixture")
        self.destination = destination
        self.events = (destination / "received.jsonl").open("w")
        self.platform = self.create_publisher(Odometry, "/nexus/platform/odom", 20)
        self.platform_tf = TransformBroadcaster(self)
        self.target = self.create_publisher(TargetKinematicState, "/nexus/vision/target_kinematics", 20)
        self.quality = self.create_publisher(String, "/nexus/observations/quality", 20)
        self.camera = self.create_publisher(CompressedImage, "/nexus/camera/imx219/image_raw/compressed", 10)
        self.create_subscription(TargetKinematicState, "/nexus/target/kinematics", self.on_target, 20)
        self.create_subscription(String, "/nexus/viz/localization_state", self.on_snapshot, 20)
        self.create_subscription(MarkerArray, "/nexus/viz/localization_markers", self.on_markers, 20)
        self.phase, self.started, self.last_target = "no_input", time.monotonic(), 0.
        self.image = np.full((480, 640, 3), 228, dtype=np.uint8)
        cv2.rectangle(self.image, (240, 170), (400, 310), (60, 140, 135), -1)
        cv2.putText(self.image, "SYNTHETIC DISPLAY QA", (65, 65), cv2.FONT_HERSHEY_SIMPLEX, 1., (30, 30, 30), 2)
        self.create_timer(.05, self.tick)

    def record(self, kind, data):
        self.events.write(json.dumps({"kind": kind, "phase": self.phase, "data": data}, allow_nan=False) + "\n")
        self.events.flush()

    def on_target(self, msg):
        self.record("fused", {"valid": msg.valid, "historical": msg.historical, "state": msg.state, "reason": msg.reason,
                              "last_valid_ns": str(msg.last_valid_sample_timestamp_ns),
                              "position": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] if msg.valid or msg.historical else None})

    def on_snapshot(self, msg):
        self.record("snapshot", json.loads(msg.data))

    def on_markers(self, msg):
        self.record("markers", [{"ns": m.ns, "id": m.id, "action": m.action, "text": m.text} for m in msg.markers])

    def tick(self):
        self.phase = (self.destination / "phase.txt").read_text().strip()
        if self.phase in {"no_input", "stop", "pause_all"}:
            return
        stamp = self.get_clock().now().to_msg()
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        t = time.monotonic() - self.started
        platform = Odometry()
        platform.header.stamp, platform.header.frame_id, platform.child_frame_id = stamp, "map", "base_link"
        platform.pose.pose.position.x, platform.pose.pose.position.y = -.4 + .1 * math.sin(t), .3
        platform.pose.pose.position.z, platform.pose.pose.orientation.w = 1.8 + .1 * math.sin(.5 * t), 1.
        platform.twist.twist.linear.x, platform.twist.twist.linear.z = .1 * math.cos(t), .05 * math.cos(.5 * t)
        self.platform.publish(platform)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = platform.header, platform.child_frame_id
        transform.transform.translation.x = platform.pose.pose.position.x
        transform.transform.translation.y = platform.pose.pose.position.y
        transform.transform.translation.z = platform.pose.pose.position.z
        transform.transform.rotation = platform.pose.pose.orientation
        self.platform_tf.sendTransform(transform)
        camera = CompressedImage()
        camera.header.stamp, camera.header.frame_id, camera.format = stamp, "camera_optical_frame", "jpeg"
        camera.data = cv2.imencode(".jpg", self.image)[1].tobytes()
        self.camera.publish(camera)
        if self.phase not in {"observed", "recovered", "outlier"} or t - self.last_target < .18:
            return
        self.last_target = t
        target = TargetKinematicState()
        target.header.stamp, target.header.frame_id, target.unit = stamp, "map", "m"
        target.target_id, target.source, target.position_reference = "static_qa", "synthetic_multiview_static", "reference_feature:qa:0"
        target.state, target.valid, target.position_observed = "confirmed", True, True
        target.receive_timestamp_ns = target.last_valid_sample_timestamp_ns = ns
        target.pose.position.x = 10.4 if self.phase == "outlier" else .4
        target.pose.position.y, target.pose.position.z = .2, .1
        target.pose.orientation.x = target.pose.orientation.y = target.pose.orientation.z = target.pose.orientation.w = float("nan")
        target.velocity.x = target.velocity.y = target.velocity.z = float("nan")
        covariance = np.full((9, 9), np.nan)
        covariance[:3, :3] = np.eye(3) * .0025
        target.covariance, target.confidence = covariance.reshape(-1).tolist(), 1.
        self.quality.publish(String(data=json.dumps({
            "schema_version": 1, "subject": "target", "target_id": target.target_id,
            "source": target.source, "position_reference": target.position_reference,
            "sample_timestamp_ns": ns, "features": {"inlier_ratio": .95}})))
        self.target.publish(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--web-port", type=int, default=18766)
    parser.add_argument("--bridge-port", type=int, default=19091)
    parser.add_argument("--ros-domain-id", type=int, default=231,
                        help="isolated display fixture domain; recording regression uses 230")
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "phase.txt").write_text("no_input")
    (args.output / "fixture.json").write_text(json.dumps({
        "synthetic": True, "run_id": "synthetic_live_display_qa", "sample_clock": "ROS wall time",
        "ros_domain_id": args.ros_domain_id, "web_port": args.web_port, "bridge_port": args.bridge_port,
        "fastdds_builtin_transports": os.environ.get("FASTDDS_BUILTIN_TRANSPORTS"),
        "control_topics": []}, indent=2))
    share = Path(get_package_share_directory("nexus_viz_dashboard"))
    commands = {
        "fusion": [executable("nexus_fusion_localization", "target_kinematic_fusion_node")],
        "display": [executable("nexus_viz_dashboard", "localization_dashboard_node"), "--ros-args", "-p", "input_mode:=SIMULATION",
                    "-p", "run_id:=synthetic_live_display_qa"],
        "bridge": [executable("rosbridge_server", "rosbridge_websocket"), "--port", str(args.bridge_port), "--address", "127.0.0.1",
                   "--topics_pub_glob", "[]", "--services_glob", "[]", "--actions_glob", "[]"],
        "web": ["python3", "-m", "http.server", str(args.web_port), "--bind", "127.0.0.1", "--directory", str(share / "web")],
    }
    if args.rviz:
        commands["rviz"] = [executable("rviz2", "rviz2"), "-d", str(share / "rviz/nexus_dual_localization.rviz")]
    children, logs = {}, []
    rclpy.init()
    fixture = Fixture(args.output)
    try:
        for name, command in commands.items():
            log = (args.output / (name + ".log")).open("w")
            logs.append(log)
            children[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        (args.output / "processes.json").write_text(json.dumps({name: process.pid for name, process in children.items()}))
        print(f"Synthetic display fixture: {args.output}", flush=True)
        until = time.monotonic() + 600.
        while time.monotonic() < until and fixture.phase != "stop":
            rclpy.spin_once(fixture, timeout_sec=.05)
            if fixture.phase == "disconnect" and children["bridge"].poll() is None:
                os.killpg(children["bridge"].pid, signal.SIGINT)
    except KeyboardInterrupt:
        pass
    finally:
        for process in children.values():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children.values():
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        fixture.events.close()
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
