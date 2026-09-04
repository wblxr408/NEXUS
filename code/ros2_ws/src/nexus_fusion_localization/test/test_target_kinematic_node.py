"""DDS integration and deterministic deadlines for correlated target fusion."""

from copy import deepcopy
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

from nexus_msgs.msg import TargetKinematicState
from nexus_fusion_localization.kinematic_filter import KinematicTargetFilter
from nexus_fusion_localization.reliability import ReliabilityMLP, covariance_scales
from nexus_fusion_localization.target_kinematic_node import TargetKinematicFusionNode


def message(stamp, x=1., *, velocity=False, orientation=False, target_id="metric_one"):
    msg = TargetKinematicState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.header.frame_id, msg.target_id, msg.unit = "map", target_id, "m"
    msg.receive_timestamp_ns = msg.last_valid_sample_timestamp_ns = stamp
    msg.valid = msg.position_observed = True
    msg.velocity_observed = velocity
    msg.orientation_observed = orientation
    msg.state, msg.source, msg.position_reference = "confirmed", "superpoint_multiview_static", "reference_feature:sample:0"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), 2., 3.
    msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = msg.pose.orientation.w = float("nan")
    if orientation:
        quaternion = Rotation.from_euler("xyz", [.3, -.2, .5]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quaternion.tolist()
        msg.source = "known_keypoint_pnp"
    msg.velocity.x, msg.velocity.y, msg.velocity.z = (.2, 0., 0.) if velocity else (float("nan"),) * 3
    covariance = np.full((9, 9), np.nan)
    indices = np.r_[np.arange(3), np.arange(3, 6) if velocity else [], np.arange(6, 9) if orientation else []].astype(int)
    covariance[np.ix_(indices, indices)] = np.eye(len(indices)) * .01
    if velocity:
        covariance[:3, 3:6] = covariance[3:6, :3] = np.eye(3) * .002
    msg.covariance, msg.confidence = covariance.reshape(-1).tolist(), 1.
    return msg


@pytest.mark.parametrize("velocity,orientation", [(False, False), (False, True)])
def test_static_dds_preserves_state_rejects_outlier_and_reports_history(velocity, orientation):
    rclpy.init(args=["--ros-args", "-p", "max_age_ms:=150.0", "-p", "reassociation_gap_ms:=450.0"])
    estimator = TargetKinematicFusionNode()
    driver = Node("metric_fusion_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(estimator)
    executor.add_node(driver)
    outputs, statuses = [], []
    driver.create_subscription(TargetKinematicState, "/nexus/target/kinematics", outputs.append, 30)
    driver.create_subscription(String, "/nexus/target/kinematic_fusion_status", lambda msg: statuses.append(json.loads(msg.data)), 30)
    publisher = driver.create_publisher(TargetKinematicState, "/nexus/vision/target_kinematics", 30)

    def spin_until(predicate, timeout=4.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate(), f"target fusion DDS condition timed out: {statuses[-3:]}"

    try:
        spin_until(lambda: publisher.get_subscription_count() and estimator._publisher.get_subscription_count() and estimator._status_publisher.get_subscription_count())
        first = message(driver.get_clock().now().nanoseconds, velocity=velocity, orientation=orientation)
        publisher.publish(first)
        spin_until(lambda: any(item.valid for item in outputs))
        output = next(item for item in outputs if item.valid)
        assert output.position_observed and output.velocity_observed == velocity and output.orientation_observed == orientation
        if orientation:
            np.testing.assert_allclose([output.pose.orientation.x, output.pose.orientation.y, output.pose.orientation.z, output.pose.orientation.w],
                                       [first.pose.orientation.x, first.pose.orientation.y, first.pose.orientation.z, first.pose.orientation.w])
            assert np.isnan(output.velocity.x)
        else:
            assert np.isnan(output.pose.orientation.w)
        np.testing.assert_allclose(output.covariance, first.covariance)
        assert output.last_valid_sample_timestamp_ns == first.last_valid_sample_timestamp_ns
        bad = message(driver.get_clock().now().nanoseconds, x=100., velocity=velocity, orientation=orientation)
        publisher.publish(bad)
        spin_until(lambda: any(item.get("reason") == "kinematic_innovation_outlier" for item in statuses))
        assert estimator.estimator.tracks[first.target_id].estimate.stamp_ns == first.last_valid_sample_timestamp_ns
        good = message(driver.get_clock().now().nanoseconds, x=1.01, velocity=velocity, orientation=orientation)
        publisher.publish(good)
        spin_until(lambda: any(item.valid and item.last_valid_sample_timestamp_ns == good.last_valid_sample_timestamp_ns for item in outputs))
        spin_until(lambda: any(item.historical and item.reason == "target_not_observed" for item in outputs))
        history = next(item for item in outputs if item.historical and item.reason == "target_not_observed")
        assert not history.valid and not history.position_observed and not history.velocity_observed
        assert not history.orientation_observed
        assert history.last_valid_sample_timestamp_ns == good.last_valid_sample_timestamp_ns
        accepted = next(item for item in outputs if item.valid and item.last_valid_sample_timestamp_ns == good.last_valid_sample_timestamp_ns)
        np.testing.assert_allclose([history.pose.position.x, history.pose.position.y, history.pose.position.z],
                                   [accepted.pose.position.x, accepted.pose.position.y, accepted.pose.position.z])
        assert not any(item.reason == "prediction_only" for item in outputs)
    finally:
        executor.shutdown()
        driver.destroy_node()
        estimator.destroy_node()
        rclpy.shutdown()


def test_learned_quality_waits_for_exact_source_reference_and_timestamp(tmp_path, monkeypatch):
    rng = np.random.default_rng(14)
    model = ReliabilityMLP(seed=14)
    model.fit(rng.normal(size=(30, 20)), rng.uniform(.2, .8, size=(30, 4)),
              [f"synthetic_{i // 10}" for i in range(30)], epochs=3,
              label_provenance="synthetic kinematic message wiring test")
    path = tmp_path / "quality.npz"
    model.save(path)
    rclpy.init(args=["--ros-args", "-p", "reliability_mode:=learned", "-p", f"reliability_model:={path}"])
    node = TargetKinematicFusionNode()
    now = [2_000_000_000]
    outputs, statuses = [], []
    monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])))
    node._publisher = SimpleNamespace(publish=outputs.append)
    node._status_publisher = SimpleNamespace(publish=lambda msg: statuses.append(json.loads(msg.data)))
    try:
        msg = message(now[0], velocity=False)
        payload = {"schema_version": 1, "subject": "target", "target_id": msg.target_id,
                   "source": msg.source, "position_reference": msg.position_reference,
                   "sample_timestamp_ns": now[0], "features": {"inlier_ratio": .8}}
        node._callback(msg)
        node._drain()
        assert not outputs and node._pending
        for changed in ({"source": "wrong_source"}, {"position_reference": "wrong_reference"}, {"sample_timestamp_ns": now[0] - 1}):
            node._quality_callback(String(data=json.dumps({**payload, **changed})))
        node._drain()
        assert not outputs
        node._quality_callback(String(data=json.dumps(payload)))
        node._drain()
        expected_scale = covariance_scales(model.predict({"inlier_ratio": .8, "network_latency_s": 0.}))[0]
        assert outputs[-1].valid and statuses[-1]["quality_matched"]
        np.testing.assert_allclose(np.array(outputs[-1].covariance).reshape(9, 9)[:3, :3], np.eye(3) * .01 * expected_scale)
        assert np.isnan(outputs[-1].velocity.x) and np.isnan(outputs[-1].pose.orientation.w)
        # Missing quality waits only the configured interval, then exposes its
        # missing feature mask to the learned model; it never borrows old data.
        now[0] += 20_000_000
        second = message(now[0], velocity=False, target_id="second")
        node._callback(second)
        node._drain()
        assert len(outputs) == 1
        now[0] += 31_000_000
        node._drain()
        assert len(outputs) == 2 and not statuses[-1]["quality_matched"]
        missing_scale = covariance_scales(model.predict({"network_latency_s": 0.}))[0]
        np.testing.assert_allclose(np.array(outputs[-1].covariance).reshape(9, 9)[:3, :3], np.eye(3) * .01 * missing_scale)
        # An upstream invalid packet cancels an uninitialized pending estimate
        # and prevents a delayed duplicate from reviving that same sample.
        now[0] += 10_000_000
        pending = message(now[0], velocity=False, target_id="cancelled")
        node._callback(pending)
        assert any(key[0] == "cancelled" for key in node._pending)
        invalid = deepcopy(pending)
        invalid.valid, invalid.reason = False, "target_identity_ambiguous"
        node._callback(invalid)
        assert not any(key[0] == "cancelled" for key in node._pending)
        node._callback(pending)
        assert outputs[-1].reason == "superseded_by_upstream_invalid"
        now[0] += 40_000_000
        node._drain()
        assert "cancelled" not in node.estimator.tracks
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_deadline_rollback_old_invalid_future_and_identity_ambiguity(monkeypatch):
    rclpy.init(args=[])
    node = TargetKinematicFusionNode()
    now = [2_000_000_000]
    outputs = []
    monkeypatch.setattr(node, "get_clock", lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0])))
    node._publisher = SimpleNamespace(publish=outputs.append)
    node._status_publisher = SimpleNamespace(publish=lambda msg: None)
    try:
        first = message(now[0])
        node._callback(first)
        node._drain()
        assert outputs[-1].valid
        original_track = node.estimator.tracks[first.target_id]
        count = len(outputs)
        future = message(now[0] + 1)
        future.valid = False
        future.reason = "target_identity_ambiguous"
        node._callback(future)
        assert len(outputs) == count and node.estimator.tracks[first.target_id] is original_track
        old = deepcopy(first)
        old.header.stamp.nanosec = 999_999_999
        old.header.stamp.sec = 1
        old.valid, old.reason = False, "target_identity_ambiguous"
        node._callback(old)
        assert len(outputs) == count and not original_track.identity_uncertain

        original_update = KinematicTargetFilter.update

        def delayed(self, *args, **kwargs):
            result = original_update(self, *args, **kwargs)
            now[0] += 300_000_000
            return result

        monkeypatch.setattr(KinematicTargetFilter, "update", delayed)
        now[0] += 10_000_000
        late = message(now[0])
        node._callback(late)
        node._drain()
        assert outputs[-1].reason == "target_fusion_solution_stale"
        assert node.estimator.tracks[first.target_id] is original_track
        assert node.estimator.source_stamps[first.target_id][first.source] == first.last_valid_sample_timestamp_ns
        monkeypatch.setattr(KinematicTargetFilter, "update", original_update)
        fresh = message(now[0])
        node._callback(fresh)
        node._drain()
        assert outputs[-1].valid
        ambiguous = deepcopy(fresh)
        ambiguous.valid, ambiguous.reason = False, "target_identity_ambiguous"
        node._callback(ambiguous)
        now[0] += 100_000_000
        node._watchdog()
        assert outputs[-1].reason == "target_identity_ambiguous"
        assert outputs[-1].historical and not outputs[-1].valid
        assert outputs[-1].pose.position.x == fresh.pose.position.x
        assert node.estimator.history(fresh.target_id, now[0])["historical"]
        # A long outage preserves the map point; nonzero target velocity is rejected.
        now[0] += 60_000_000_000
        node._watchdog()
        assert fresh.target_id in node.estimator.tracks
        moving = message(now[0], velocity=True)
        node._callback(moving)
        assert outputs[-1].reason == "static_target_velocity_must_be_unobserved"
        assert outputs[-1].historical and outputs[-1].last_valid_sample_timestamp_ns == fresh.last_valid_sample_timestamp_ns
    finally:
        node.destroy_node()
        rclpy.shutdown()
