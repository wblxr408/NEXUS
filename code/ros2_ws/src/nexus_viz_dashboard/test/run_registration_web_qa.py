#!/usr/bin/env python3
"""Opt-in isolated browser/ROS/GPU registration QA; synthetic imagery and calibration."""
import argparse
import faulthandler
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import String
import yaml

# Resolve the current source package, including new modules not yet symlink-installed.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "nexus_vision_localization/src"))
from nexus_vision_localization.superpoint_node import SuperPointMotionNode  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--web-port", type=int, default=18768)
    parser.add_argument("--bridge-port", type=int, default=19093)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "88":
        raise ValueError("QA requires isolated ROS_DOMAIN_ID=88")
    for port in (args.web_port, args.bridge_port):
        with socket.socket() as check:
            check.bind(("127.0.0.1", port))
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "phase.txt").write_text("textured")
    calibration = args.output / "synthetic_calibration.yaml"
    calibration.write_text(yaml.safe_dump({
        "schema_version": 1, "calibration_id": "synthetic_web_registration",
        "calibration_status": "synthetic_test", "camera_frame": "camera_optical_frame",
        "image_rectified": True, "transform_body_camera": np.eye(4).tolist()}))
    rclpy.init(args=["--ros-args", "-p", "model_manifest:=" + args.manifest,
                     "-p", "calibration_file:=" + str(calibration), "-p", "allow_test_calibration:=true",
                     "-p", "enable_target_tracking:=true", "-p", "image_topic:=/qa/unused_live_image"])
    backend = SuperPointMotionNode()
    fixture = Node("synthetic_web_registration")
    executor = SingleThreadedExecutor()
    executor.add_node(backend)
    executor.add_node(fixture)
    bridge = CvBridge()
    raw = fixture.create_publisher(Image, "/camera/image_rect", 10)
    preview = fixture.create_publisher(CompressedImage, "/nexus/camera/imx219/image_raw/compressed", 10)
    info = fixture.create_publisher(CameraInfo, "/camera/camera_info", 10)
    imu = fixture.create_publisher(Imu, "/nexus/fcu/imu", 10)
    detection = fixture.create_publisher(String, "/nexus/vision/detections", 10)
    snapshot = fixture.create_publisher(String, "/nexus/viz/localization_state", 10)
    rng = np.random.default_rng(20260905)
    texture = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
    texture = cv2.GaussianBlur(texture, (5, 5), 0)
    cv2.putText(texture, "SYNTHETIC WEB QA", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 100), 3)
    texture = cv2.resize(texture, (1640, 1232))
    phases = {"textured": texture, "blank": np.zeros_like(texture)}
    emitted = {}
    events = (args.output / "events.jsonl").open("w")

    def record(kind, data):
        events.write(json.dumps({"kind": kind, "data": data}) + "\n")
        events.flush()

    def reference(message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        digest = hashlib.sha256(bytes(message.data)).hexdigest()
        record("reference", {"stamp": str(stamp), "sha256": digest, "matches_emitted": emitted.get(stamp) == digest,
                             "width": message.width, "height": message.height})

    fixture.create_subscription(Image, "/nexus/vision/target_reference_image", reference, 10)
    fixture.create_subscription(String, "/nexus/vision/target_requests", lambda msg: record("request", json.loads(msg.data)), 10)
    fixture.create_subscription(String, "/nexus/vision/target_request_status", lambda msg: record("status", json.loads(msg.data)), 10)
    fixture.create_subscription(String, "/nexus/web_qa/blocked", lambda msg: record("blocked_received", msg.data), 10)
    processes, logs = [], []
    workspace = Path(__file__).resolve().parents[1] / "web"
    executable = Path(get_package_prefix("rosbridge_server")) / "lib/rosbridge_server/rosbridge_websocket"
    commands = [
        [str(executable), "--port", str(args.bridge_port), "--address", "127.0.0.1",
         "--topics_pub_glob", "['/nexus/vision/target_reference_image','/nexus/vision/target_requests']",
         "--services_glob", "[]", "--actions_glob", "[]"],
        ["python3", "-m", "http.server", str(args.web_port), "--bind", "127.0.0.1", "--directory", str(workspace)],
    ]
    for index, command in enumerate(commands):
        log = (args.output / ("server_" + str(index) + ".log")).open("w")
        logs.append(log)
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    (args.output / "runtime.json").write_text(json.dumps({
        "synthetic": True, "domain": 88, "fixture_pid": os.getpid(), "device": str(backend.model.network.device),
        "model_manifest": args.manifest, "processes": [p.pid for p in processes]}))
    print("READY synthetic ROS/GPU fixture", args.output, flush=True)
    last = 0.
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    deadline = time.monotonic() + 900.
    try:
        while time.monotonic() < deadline and not stopping:
            phase = (args.output / "phase.txt").read_text().strip()
            if phase == "stop":
                break
            now = time.monotonic()
            if phase != "pause" and now - last > .5:
                last = now
                stamp = fixture.get_clock().now().to_msg()
                ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                pixels = phases.get(phase, texture)
                message = bridge.cv2_to_imgmsg(pixels, "bgr8")
                message.header.stamp, message.header.frame_id = stamp, "camera_optical_frame"
                emitted[ns] = hashlib.sha256(bytes(message.data)).hexdigest()
                if len(emitted) > 500:
                    emitted.pop(next(iter(emitted)))
                calibration_msg = CameraInfo()
                calibration_msg.header = message.header
                calibration_msg.width, calibration_msg.height = message.width, message.height
                calibration_msg.k = [1000., 0., 820., 0., 1000., 616., 0., 0., 1.]
                calibration_msg.p = [1000., 0., 820., 0., 0., 1000., 616., 0., 0., 0., 1., 0.]
                info.publish(calibration_msg)
                detection.publish(String(data=json.dumps({
                    "schema_version": 1, "sample_timestamp_ns": ns,
                    "frame_id": message.header.frame_id, "image_size_wh": [1640, 1232], "valid": True,
                    "dynamic_boxes": [], "detections": []})))
                raw.publish(message)
                if len(emitted) == 1:
                    print("First synthetic frame published", ns, flush=True)
                compressed = CompressedImage()
                compressed.header, compressed.format = message.header, "jpeg"
                compressed.data = cv2.imencode(".jpg", pixels)[1].tobytes()
                preview.publish(compressed)
                inertial = Imu()
                inertial.header.stamp, inertial.header.frame_id = stamp, "imu_link"
                imu.publish(inertial)
                # Explicit simulation display fixture: no claimed metric target result.
                snapshot.publish(String(data=json.dumps({
                    "schema_version": 1, "session_id": "synthetic_registration", "generated_timestamp_ns": str(ns),
                    "input_mode": "SIMULATION", "run_id": "synthetic_web_registration_qa", "frame_id": "map", "unit": "m",
                    "platform": {"display_state": "no_input", "position": None, "orientation": None,
                                 "speed_mps": None, "age_s": None}, "targets": []})))
            executor.spin_once(timeout_sec=.01)
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        events.close()
        executor.shutdown()
        backend.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
