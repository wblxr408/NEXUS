"""Reference-image -> features -> metric target over DDS, with synthetic encoders."""

import json
import time
from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import yaml

from nexus_msgs.msg import TargetKinematicState
from nexus_fusion_localization.reliability import feature_row
from nexus_fusion_localization.target_kinematic_node import TargetKinematicFusionNode
from nexus_vision_localization import superpoint_node, target_metric_node
from nexus_vision_localization.superpoint_frontend import FeatureSet
from test_target_frontend import SyntheticDense, image
from test_target_pipeline_nodes import detection_message, stamped
from test_target_multiview import CAMERA


def calibration(tmp_path):
    path = tmp_path / "synthetic_target_metric.yaml"
    path.write_text(yaml.safe_dump({
        "schema_version": 1, "calibration_id": "synthetic_target_metric", "calibration_status": "synthetic_test",
        "camera_frame": "camera_optical_frame", "image_rectified": True, "transform_body_camera": np.eye(4).tolist(),
        "extrinsic_covariance": np.zeros((6, 6)).tolist(), "pixel_sigma_px": .2,
    }))
    return path


def camera_message(stamp):
    message = stamped(CameraInfo(), stamp)
    message.width, message.height = 640, 480
    message.p = np.column_stack((CAMERA, np.zeros(3))).reshape(-1).tolist()
    message.k = np.eye(3).reshape(-1).tolist()
    return message


def platform_message(stamp, center):
    message = Odometry()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.header.frame_id, message.child_frame_id = "map", "base_link"
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = [float(v) for v in center]
    message.pose.pose.orientation.w = 1.
    return message


@pytest.mark.parametrize("variable_motion", [False, True])
def test_reference_image_to_static_target_with_variable_platform_motion_over_dds(tmp_path, monkeypatch, variable_motion):
    offsets = np.array([[x, y, 0.] for y in (-.1, .0, .1) for x in (-.15, -.05, .05, .15)])
    target = np.array([.1, -.1, 4.])
    velocity = np.zeros(3)
    motion_model = "static"
    features, centers = [], []
    for index in range(8):
        t = index * .1
        center = (np.array([.6 * t, .6 * t ** 2, .1 * np.sin(5 * t)]) if variable_motion
                  else np.array([.4 * t, 0., 0.]))
        points = target + offsets + velocity * t
        pixels = cv2.projectPoints(points, np.zeros(3), -center, CAMERA, np.zeros(5))[0].reshape(-1, 2)
        features.append(FeatureSet(pixels, np.eye(256, dtype=np.float32)[:12], np.ones(12), (640, 480)))
        centers.append(center)

    class SyntheticEncoder:
        def __init__(self, _):
            pass

        def infer(self, pixels, *, image_scale=1.):
            assert .25 <= image_scale <= 1.
            return SyntheticDense(features[int(pixels[0, 0, 0])])

    monkeypatch.setattr(superpoint_node, "SuperPointOnnx", SyntheticEncoder)
    path = calibration(tmp_path)
    rclpy.init(args=["--ros-args", "-p", f"calibration_file:={path}", "-p", "allow_test_calibration:=true",
                     "-p", "model_manifest:=synthetic_network_boundary", "-p", "enable_target_tracking:=true",
                     "-p", "max_age_ms:=3000.0", "-p", "reassociation_gap_ms:=3000.0"])
    frontend, metric, driver = superpoint_node.SuperPointMotionNode(), target_metric_node.TargetMetricNode(), Node("target_metric_test_driver")
    fusion = TargetKinematicFusionNode()
    executor = SingleThreadedExecutor()
    for node in (frontend, metric, fusion, driver):
        executor.add_node(node)
    outputs, tracks, requests, statuses, qualities = [], [], [], [], []
    driver.create_subscription(TargetKinematicState, "/nexus/vision/target_kinematics", outputs.append, 20)
    fused = []
    driver.create_subscription(TargetKinematicState, "/nexus/target/kinematics", fused.append, 20)
    driver.create_subscription(String, "/nexus/vision/target_tracks", lambda msg: tracks.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/vision/target_request_status", lambda msg: requests.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/vision/target_geometry_status", lambda msg: statuses.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/observations/quality", lambda msg: qualities.append(json.loads(msg.data)), 20)
    publishers = {"image": driver.create_publisher(Image, "/camera/image_rect", 20),
                  "camera": driver.create_publisher(CameraInfo, "/camera/camera_info", 10),
                  "pose": driver.create_publisher(Odometry, "/nexus/platform/odom", 20),
                  "reference": driver.create_publisher(Image, "/nexus/vision/target_reference_image", 10),
                  "request": driver.create_publisher(String, "/nexus/vision/target_requests", 10),
                  "detections": driver.create_publisher(String, "/nexus/vision/detections", 20)}

    def spin(predicate):
        deadline = time.monotonic() + 8.
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate(), f"metric target DDS condition timed out: {statuses[-3:]}"

    def frame(marker, stamp):
        pixels = image()
        pixels[0, 0, 0] = marker
        return stamped(CvBridge().cv2_to_imgmsg(pixels, encoding="bgr8"), stamp)

    try:
        spin(lambda: all(pub.get_subscription_count() for pub in publishers.values()) and frontend._tracks_pub.get_subscription_count() == 2
             and metric._publisher.get_subscription_count() == 2 and fusion._publisher.get_subscription_count()
             and frontend._requests_pub.get_subscription_count())
        start = driver.get_clock().now().nanoseconds - 900_000_000
        publishers["camera"].publish(camera_message(start))
        spin(lambda: frontend._camera is not None and metric._camera is not None)
        lower, upper = features[0].points_px.min(axis=0) - 4, features[0].points_px.max(axis=0) + 4
        request = {"schema_version": 1, "request_id": "metric_reference_1", "target_id": "chosen",
                   "sample_timestamp_ns": 1_000_000_000, "frame_id": "camera_optical_frame",
                   "bbox_xywh_px": np.r_[lower, upper - lower].tolist(),
                   "anchor_reference_px": features[0].points_px[0].tolist()}
        publishers["request"].publish(String(data=json.dumps(request)))
        publishers["reference"].publish(frame(0, 1_000_000_000))
        spin(lambda: any(item["state"] == "registered" for item in requests))
        for index in range(8):
            stamp = start + index * 100_000_000
            publishers["image"].publish(frame(index, stamp))
            publishers["detections"].publish(detection_message(stamp))
            # The target packet is permitted to arrive before its platform pose.
            spin(lambda: any(packet["sample_timestamp_ns"] == stamp for packet in tracks))
            publishers["pose"].publish(platform_message(stamp, centers[index]))
            spin(lambda: any(metric._stamp(message.header) == stamp for message in outputs))
        output = outputs[-1]
        assert output.valid and output.position_observed and output.target_id == "chosen"
        assert output.header.frame_id == "map" and output.unit == "m"
        assert output.position_reference.endswith(":0") and output.source.endswith(motion_model)
        expected = target + offsets[0] + velocity * .7
        np.testing.assert_allclose([output.pose.position.x, output.pose.position.y, output.pose.position.z], expected, atol=2e-4)
        assert not output.velocity_observed and not output.historical
        if output.velocity_observed:
            np.testing.assert_allclose([output.velocity.x, output.velocity.y, output.velocity.z], velocity, atol=2e-4)
        else:
            assert np.isnan(output.velocity.x)
        assert not output.orientation_observed and np.isnan(output.pose.orientation.w)
        spin(lambda: any(item.valid and item.last_valid_sample_timestamp_ns == stamp for item in fused))
        final = next(item for item in reversed(fused) if item.valid and item.last_valid_sample_timestamp_ns == stamp)
        assert final.position_reference == output.position_reference and final.velocity_observed == output.velocity_observed
        assert not final.orientation_observed and np.isnan(final.pose.orientation.w)
        np.testing.assert_allclose([final.pose.position.x, final.pose.position.y, final.pose.position.z], expected, atol=2e-4)
        if final.velocity_observed:
            np.testing.assert_allclose([final.velocity.x, final.velocity.y, final.velocity.z], velocity, atol=2e-4)
        covariance = np.array(output.covariance).reshape(9, 9)
        assert np.isfinite(covariance[:3, :3]).all() and np.isnan(covariance[6:, 6:]).all()
        assert any(item["reason"] == "insufficient_target_views" for item in statuses)
        spin(lambda: any(item.get("subject") == "target" for item in qualities))
        for item in qualities:
            if item.get("subject") == "target":
                feature_row(item["features"])
        old = metric._latest["chosen"]
        metric._invalid("chosen", start, "old_failure")
        assert metric._latest["chosen"] is old  # late failure cannot overwrite fresh state
    finally:
        executor.shutdown()
        for node in (driver, fusion, metric, frontend):
            node.destroy_node()
        rclpy.shutdown()


def test_metric_waits_for_bracket_and_does_not_commit_expired_solution(tmp_path, monkeypatch):
    path = calibration(tmp_path)
    rclpy.init(args=["--ros-args", "-p", f"calibration_file:={path}", "-p", "allow_test_calibration:=true"])
    node = target_metric_node.TargetMetricNode()
    now = [2_000_000_000]
    monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])))
    try:
        node._camera_callback(camera_message(now[0]))
        target = np.array([.1, -.1, 4.])

        def packet(stamp, center):
            pixel = cv2.projectPoints(target.reshape(1, 3), np.zeros(3), -np.array(center), CAMERA, np.zeros(5))[0].reshape(2)
            return {"schema_version": 1, "sample_timestamp_ns": stamp, "frame_id": "camera_optical_frame", "unit": "px",
                    "image_size_wh": [640, 480], "valid": True,
                    "tracks": [{"track_id": "chosen", "observed": True, "identity_ambiguous": False, "identity_confidence": 1.,
                                "reference_id": "test_reference", "motion_model": "static", "anchor_feature_id": 0,
                                "state": "confirmed", "feature_observations": [{"feature_id": 0, "pixel_px": pixel.tolist()}]}]}

        first_stamp = now[0] - 150_000_000
        node._tracks_callback(String(data=json.dumps(packet(first_stamp, [0., 0., 0.]))))
        node._platform_callback(platform_message(first_stamp - 20_000_000, [-.02, 0., 0.]))
        node._drain()
        assert first_stamp in node._pending and not node._windows
        node._platform_callback(platform_message(first_stamp + 20_000_000, [.02, 0., 0.]))
        node._drain()
        assert node._windows["chosen"][1][0].platform.interpolated
        for index in (1, 2):
            stamp = first_stamp + index * 50_000_000
            center = [.1 * index, 0., 0.]
            node._platform_callback(platform_message(stamp, center))
            node._tracks_callback(String(data=json.dumps(packet(stamp, center))))
            node._drain()
        assert node._latest["chosen"].valid
        saved_window = node._windows["chosen"]
        original = target_metric_node.solve_multiview

        def delayed(*args, **kwargs):
            result = original(*args, **kwargs)
            now[0] += 400_000_000
            return result

        monkeypatch.setattr(target_metric_node, "solve_multiview", delayed)
        stamp = now[0]
        node._platform_callback(platform_message(stamp, [.3, 0., 0.]))
        node._tracks_callback(String(data=json.dumps(packet(stamp, [.3, 0., 0.]))))
        node._drain()
        assert node._windows["chosen"] is saved_window
        assert not node._latest["chosen"].valid and node._latest["chosen"].reason == "target_geometry_stale"
        monkeypatch.setattr(target_metric_node, "solve_multiview", original)
        stamp = now[0]
        node._platform_callback(platform_message(stamp, [.4, 0., 0.]))
        node._tracks_callback(String(data=json.dumps(packet(stamp, [.4, 0., 0.]))))
        node._drain()
        assert node._latest["chosen"].valid
        last = node._latest["chosen"]
        future_invalid = packet(now[0] + 10_000_000_000, [.4, 0., 0.])
        future_invalid["valid"] = False
        node._tracks_callback(String(data=json.dumps(future_invalid)))
        assert node._latest["chosen"] is last
    finally:
        node.destroy_node()
        rclpy.shutdown()
