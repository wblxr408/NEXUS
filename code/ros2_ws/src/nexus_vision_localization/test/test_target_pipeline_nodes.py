"""ROS reference-image ingress, identity output, deadlines and mask alignment.

Synthetic network outputs are test fixtures, not learned-inference evidence.
"""

import json
import time
from types import SimpleNamespace

from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from nexus_vision_localization import superpoint_node
from test_target_frontend import SyntheticDense, image
from test_target_identity import candidate
from test_visual_pipeline_nodes import calibration


@pytest.fixture
def rig(tmp_path, monkeypatch):
    inferred = []

    class SyntheticEncoder:
        def __init__(self, _):
            pass

        def infer(self, pixels, *, image_scale=1.):
            assert .25 <= image_scale <= 1.
            marker = int(pixels[0, 0, 0])
            features = candidate(40 if marker == 1 else 240 + 6 * (marker - 2), y=100).features
            if marker == 9:
                features = features.select(np.zeros(12, bool))
            dense = SyntheticDense(features)
            inferred.append((marker, dense))
            return dense

    monkeypatch.setattr(superpoint_node, "SuperPointOnnx", SyntheticEncoder)
    path = calibration(tmp_path)
    rclpy.init(args=["--ros-args", "-p", f"calibration_file:={path}", "-p", "allow_test_calibration:=true",
                     "-p", "model_manifest:=synthetic_network_boundary", "-p", "enable_target_tracking:=true",
                     "-p", "max_age_ms:=3000.0"])
    node, driver = superpoint_node.SuperPointMotionNode(), Node("target_pipeline_regression_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(driver)
    outputs, requests, statuses = [], [], []
    driver.create_subscription(String, "/nexus/vision/target_tracks", lambda msg: outputs.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/vision/target_request_status", lambda msg: requests.append(json.loads(msg.data)), 20)
    driver.create_subscription(String, "/nexus/vision/motion_status", lambda msg: statuses.append(json.loads(msg.data)), 20)
    publishers = {
        "image": driver.create_publisher(Image, "/camera/image_rect", 10),
        "camera": driver.create_publisher(CameraInfo, "/camera/camera_info", 10),
        "reference": driver.create_publisher(Image, "/nexus/vision/target_reference_image", 10),
        "request": driver.create_publisher(String, "/nexus/vision/target_requests", 10),
        "detections": driver.create_publisher(String, "/nexus/vision/detections", 10),
    }

    def spin_until(predicate, timeout=8.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate(), "target DDS condition timed out"

    try:
        yield SimpleNamespace(node=node, driver=driver, spin=spin_until, pubs=publishers, inferred=inferred,
                              outputs=outputs, requests=requests, statuses=statuses)
    finally:
        executor.shutdown()
        node.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()


def stamped(message, stamp):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.header.frame_id = "camera_optical_frame"
    return message


def image_message(marker, stamp):
    pixels = image()
    pixels[0, 0, 0] = marker
    return stamped(CvBridge().cv2_to_imgmsg(pixels, encoding="bgr8"), stamp)


def camera_message(stamp):
    message = stamped(CameraInfo(), stamp)
    message.width, message.height = 640, 480
    matrix = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    message.p = np.column_stack((matrix, np.zeros(3))).reshape(-1).tolist()
    return message


def detection_message(stamp):
    return String(data=json.dumps({"schema_version": 1, "sample_timestamp_ns": stamp, "frame_id": "camera_optical_frame",
                                   "image_size_wh": [640, 480], "valid": True, "dynamic_boxes": [], "detections": []}))


def request_message(identifier="select_1", stamp=1_000_000_000):
    return String(data=json.dumps({"schema_version": 1, "request_id": identifier, "target_id": "chosen",
                                   "sample_timestamp_ns": stamp, "frame_id": "camera_optical_frame",
                                   "bbox_xywh_px": [40., 100., 40., 40.]}))


def test_reference_only_target_tracks_over_dds_when_static_background_is_insufficient(rig):
    rig.spin(lambda: all(pub.get_subscription_count() for pub in rig.pubs.values())
             and rig.node._tracks_pub.get_subscription_count() and rig.node._requests_pub.get_subscription_count()
             and rig.node._status_pub.get_subscription_count())
    rig.pubs["camera"].publish(camera_message(rig.driver.get_clock().now().nanoseconds))
    rig.spin(lambda: rig.node._camera is not None)
    rig.pubs["request"].publish(request_message())
    rig.spin(lambda: any(item["state"] == "pending" for item in rig.requests))
    assert not rig.inferred
    # Historical image pairs by exact original stamp, not live age or arrival.
    rig.pubs["reference"].publish(image_message(1, 1_000_000_000))
    rig.spin(lambda: any(item["state"] == "registered" for item in rig.requests))
    assert rig.node.targets.tracker.tracks["chosen"].last_observed_ns is None
    for marker in (2, 3, 4):
        stamp = rig.driver.get_clock().now().nanoseconds - 20_000_000
        rig.pubs["image"].publish(image_message(marker, stamp))
        rig.pubs["detections"].publish(detection_message(stamp))
        rig.spin(lambda: any(item["sample_timestamp_ns"] == stamp for item in rig.outputs))
        packet = next(item for item in rig.outputs if item["sample_timestamp_ns"] == stamp)
        assert len(packet["tracks"]) == 1 and packet["tracks"][0]["track_id"] == "chosen"
        assert packet["tracks"][0]["observed"] and packet["unit"] == "px"
        # Last feature selection is static background, after reference masking.
        x = 240 + 6 * (marker - 2)
        assert not rig.inferred[-1][1].masks[-1][100:140, x:x + 40].any()
    assert packet["tracks"][0]["state"] == "confirmed"
    assert not packet["tracks"][0]["orientation_observed"]
    rig.spin(lambda: any(item["reason"] == "insufficient_static_features" for item in rig.statuses))
    assert rig.node.tracker.reference is None
    count = len(rig.inferred)
    rig.pubs["request"].publish(request_message())
    registered_count = sum(item["state"] == "registered" for item in rig.requests)
    rig.spin(lambda: sum(item["state"] == "registered" for item in rig.requests) > registered_count)
    assert len(rig.inferred) == count  # successful request IDs are idempotent


def test_expired_processing_rolls_back_both_trackers_and_watchdog_preserves_sample_time(rig, monkeypatch):
    now = [2_000_000_000]
    monkeypatch.setattr(rig.node, "get_clock", lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])))
    rig.node._max_age_ns = 300_000_000
    rig.node._camera_callback(camera_message(now[0]))
    rig.node._request_callback(request_message())
    rig.node._reference_callback(image_message(1, 1_000_000_000))
    rig.node._drain_references()
    rig.node._image_callback(image_message(2, now[0]))
    rig.node._detections_callback(detection_message(now[0]))
    rig.node._drain()
    accepted = rig.node.targets.tracker
    accepted_motion = rig.node.tracker
    packet = rig.node._last_target_packet
    assert packet["valid"] and packet["tracks"][0]["observed"]
    original_prepare = rig.node.targets.prepare

    def delayed_prepare(*args, **kwargs):
        result = original_prepare(*args, **kwargs)
        now[0] += 400_000_000
        return result

    monkeypatch.setattr(rig.node.targets, "prepare", delayed_prepare)
    now[0] += 100_000_000
    rig.node._image_callback(image_message(3, now[0]))
    rig.node._detections_callback(detection_message(now[0]))
    rig.node._drain()
    assert rig.node.targets.tracker is accepted and rig.node.tracker is accepted_motion
    assert rig.node._last_reason == "superpoint_frontend_stale"
    assert rig.node._last_target_packet is packet
    rig.node._target_watchdog(now[0])
    assert rig.node._last_target_packet["sample_timestamp_ns"] == 2_000_000_000
    assert not rig.node._last_target_packet["valid"] and not rig.node._last_target_packet["tracks"][0]["observed"]
    assert accepted.last_stamp_ns == 2_000_000_000
    monkeypatch.setattr(rig.node.targets, "prepare", original_prepare)
    rig.node._image_callback(image_message(3, now[0]))
    rig.node._detections_callback(detection_message(now[0]))
    rig.node._drain()
    assert rig.node._last_target_packet["valid"] and rig.node._last_target_packet["tracks"][0]["observed"]


def test_reference_pair_timeout_and_inference_timeout_leave_no_reference(rig, monkeypatch):
    monotonic = [100.]
    monkeypatch.setattr(superpoint_node, "perf_counter", lambda: monotonic[0])
    rig.node._request_callback(request_message())
    rig.node._reference_callback(image_message(1, 1_000_000_001))
    rig.node._drain_references()
    assert not rig.inferred and "select_1" in rig.node._target_requests
    monotonic[0] += 11.
    rig.node._drain_references()
    assert rig.node._request_results["select_1"]["reason"] == "reference_pair_timeout"
    rig.node._request_callback(request_message("select_2"))
    rig.node._reference_callback(image_message(1, 1_000_000_000))
    original = rig.node.model.infer

    def slow_reference(image):
        result = original(image)
        monotonic[0] += 11.
        return result

    monkeypatch.setattr(rig.node.model, "infer", slow_reference)
    rig.node._drain_references()
    assert rig.node._request_results["select_2"]["reason"] == "reference_processing_timeout"
    assert not rig.node.targets.tracker.tracks


def test_segmentation_alignment_and_bounded_image_queue(rig, monkeypatch):
    now = 2_000_000_000
    monkeypatch.setattr(rig.node, "get_clock", lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now)))
    rig.node._mask_input, rig.node.targets = "segmentation", None
    rig.node._camera_callback(camera_message(now))
    rig.node._image_callback(image_message(2, now))
    mask = np.zeros((480, 640), np.uint8)
    mask[100:140, 240:280] = 255
    rig.node._mask_callback(stamped(CvBridge().cv2_to_imgmsg(mask, encoding="mono8"), now - 1))
    rig.node._drain()
    assert not rig.inferred
    rig.node._mask_callback(stamped(CvBridge().cv2_to_imgmsg(mask, encoding="mono8"), now))
    rig.node._drain()
    assert len(rig.inferred) == 1 and not rig.inferred[-1][1].masks[-1][100:140, 240:280].any()
    for delta in range(1, 8):
        now += 1
        rig.node._image_callback(image_message(2, now))
    assert len(rig.node._images) == rig.node._queue_size and rig.node._dropped == 3
