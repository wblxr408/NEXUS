#!/usr/bin/env python3
"""Fuse static map targets; publish historical positions without extrapolation."""

from copy import deepcopy
import json

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

from nexus_msgs.msg import TargetKinematicState
from nexus_fusion_localization.kinematic_filter import KinematicEstimate, KinematicFilterConfig, KinematicTargetFilter
from nexus_fusion_localization.reliability import FEATURE_NAMES, ReliabilityMLP, covariance_scales, feature_row


class TargetKinematicFusionNode(Node):
    def __init__(self):
        super().__init__("nexus_target_kinematic_fusion")
        defaults = {
            "input_topic": "/nexus/vision/target_kinematics", "output_topic": "/nexus/target/kinematics",
            "quality_topic": "/nexus/observations/quality", "status_topic": "/nexus/target/kinematic_fusion_status",
            "target_frame": "map", "max_age_ms": 250., "reassociation_gap_ms": 500., "retention_ms": 5000.,
            "confirmation_hits": 3, "maximum_tracks": 64,
            "soft_probability": .95, "hard_probability": .999,
            "reliability_mode": "fixed", "reliability_model": "", "quality_wait_ms": 30., "queue_size": 128,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.config = KinematicFilterConfig(
            frame_id=values["target_frame"], max_age_ns=int(values["max_age_ms"] * 1e6),
            reassociation_gap_ns=int(values["reassociation_gap_ms"] * 1e6), retention_ns=int(values["retention_ms"] * 1e6),
            confirmation_hits=values["confirmation_hits"], maximum_tracks=values["maximum_tracks"],
            soft_probability=values["soft_probability"], hard_probability=values["hard_probability"],
        )
        self.estimator = KinematicTargetFilter(self.config)
        self._mode = values["reliability_mode"]
        if self._mode not in {"fixed", "rule", "learned"}:
            raise ValueError("reliability_mode must be fixed, rule, or learned")
        self._model = ReliabilityMLP.load(values["reliability_model"]) if self._mode == "learned" else None
        self._quality_wait_ns = int(values["quality_wait_ms"] * 1e6)
        self._queue_size = int(values["queue_size"])
        if not 0 <= self._quality_wait_ns < self.config.max_age_ns or self._queue_size < 1:
            raise ValueError("invalid target quality wait/queue configuration")
        self._pending, self._quality, self._last_lost, self._invalid_stamps = {}, {}, {}, {}
        self._publisher = self.create_publisher(TargetKinematicState, values["output_topic"], 20)
        self._status_publisher = self.create_publisher(String, values["status_topic"], 20)
        self.create_subscription(TargetKinematicState, values["input_topic"], self._callback, 30)
        self.create_subscription(String, values["quality_topic"], self._quality_callback, 30)
        self.create_timer(.01, self._drain)
        self.create_timer(.05, self._watchdog)

    @staticmethod
    def _stamp(message):
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    @staticmethod
    def _key(record):
        return record.target_id, record.source, record.position_reference, record.stamp_ns

    def _status(self, reason, stamp_ns, identifier="", **details):
        self._status_publisher.publish(String(data=json.dumps({
            "schema_version": 1, "target_id": identifier, "sample_timestamp_ns": stamp_ns,
            "reason": str(reason), "reliability_mode": self._mode, **details}, allow_nan=False)))

    def _empty(self, identifier, stamp_ns, reason):
        output = TargetKinematicState()
        output.header.frame_id = self.config.frame_id
        output.header.stamp.sec, output.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        output.receive_timestamp_ns = max(stamp_ns, self.get_clock().now().nanoseconds)
        output.target_id, output.unit, output.source = identifier, "m", "correlated_target_fusion"
        output.reason, output.state = reason, "degraded"
        track = self.estimator.tracks.get(identifier)
        output.last_valid_sample_timestamp_ns = track.estimate.stamp_ns if track is not None else 0
        output.position_reference = track.estimate.position_reference if track is not None else ""
        output.pose.position.x = output.pose.position.y = output.pose.position.z = float("nan")
        output.pose.orientation.x = output.pose.orientation.y = output.pose.orientation.z = output.pose.orientation.w = float("nan")
        output.velocity.x = output.velocity.y = output.velocity.z = float("nan")
        output.covariance = [float("nan")] * 81
        return output

    def _publish(self, estimate):
        output = self._empty(estimate.target_id, estimate.stamp_ns, "")
        output.valid = output.position_observed = True
        output.orientation_observed = estimate.rotation is not None
        output.state = estimate.state
        output.position_reference = estimate.position_reference
        output.pose.position.x, output.pose.position.y, output.pose.position.z = estimate.position.tolist()
        if estimate.rotation is not None:
            q = Rotation.from_matrix(estimate.rotation).as_quat()
            output.pose.orientation.x, output.pose.orientation.y, output.pose.orientation.z, output.pose.orientation.w = q.tolist()
        output.covariance = estimate.covariance.reshape(-1).tolist()
        output.confidence = float(estimate.confidence)
        self._publisher.publish(output)

    def _invalid(self, identifier, stamp_ns, reason):
        track = self.estimator.tracks.get(identifier)
        if track is not None and stamp_ns < track.estimate.stamp_ns:
            return
        output = self._empty(identifier, stamp_ns, reason)
        output.state = "lost" if reason in {"target_not_observed", "target_identity_ambiguous", "reference_change_pending"} else "degraded"
        if track is not None:
            # This is explicitly the old reference point, including during
            # identity ambiguity or a pending switch to another feature.
            output.historical = True
            output.pose.position.x, output.pose.position.y, output.pose.position.z = track.estimate.position.tolist()
            output.covariance = track.estimate.covariance.reshape(-1).tolist()
            if track.estimate.rotation is not None:
                q = Rotation.from_matrix(track.estimate.rotation).as_quat()
                output.pose.orientation.x, output.pose.orientation.y, output.pose.orientation.z, output.pose.orientation.w = q.tolist()
        self._publisher.publish(output)

    def _callback(self, message):
        stamp = self._stamp(message)
        now = self.get_clock().now().nanoseconds
        identifier = message.target_id
        if (not identifier or not 0 < stamp <= now or not stamp <= message.receive_timestamp_ns <= now
                or message.header.frame_id != self.config.frame_id or message.unit != "m"):
            self._status("invalid_target_header", stamp, identifier, decision="rejected")
            return
        if not message.valid:
            track = self.estimator.tracks.get(identifier)
            floor = max(self._invalid_stamps.get(identifier, 0), track.estimate.stamp_ns if track is not None else 0)
            if stamp >= floor and now - stamp <= self.config.retention_ns:
                self._invalid_stamps[identifier] = stamp
                self._pending = {key: value for key, value in self._pending.items() if key[0] != identifier or key[3] > stamp}
                while len(self._invalid_stamps) > self.config.maximum_tracks:
                    del self._invalid_stamps[min(self._invalid_stamps, key=self._invalid_stamps.get)]
            if self.estimator.invalidate(identifier, stamp, message.reason, now):
                self._invalid(identifier, stamp, message.reason or "upstream_invalid")
            self._status(message.reason or "upstream_invalid", stamp, identifier, decision="rejected")
            return
        try:
            if stamp <= self._invalid_stamps.get(identifier, 0):
                raise ValueError("superseded_by_upstream_invalid")
            if message.historical or not message.position_observed or message.last_valid_sample_timestamp_ns != stamp:
                raise ValueError("input_must_be_a_fresh_observed_position")
            if message.velocity_observed:
                raise ValueError("static_target_velocity_must_be_unobserved")
            p, v, q = message.pose.position, message.velocity, message.pose.orientation
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if message.orientation_observed:
                if not np.all(np.isfinite(quaternion)) or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-5):
                    raise ValueError("invalid_target_quaternion")
                rotation = Rotation.from_quat(quaternion).as_matrix()
            else:
                if not np.all(np.isnan(quaternion)):
                    raise ValueError("unknown_orientation_must_be_nan")
                rotation = None
            velocity = np.array([v.x, v.y, v.z])
            if not message.velocity_observed and not np.all(np.isnan(velocity)):
                raise ValueError("unknown_velocity_must_be_nan")
            record = KinematicEstimate(
                identifier, stamp, int(message.receive_timestamp_ns), message.source, message.position_reference,
                np.array([p.x, p.y, p.z]), np.array(message.covariance).reshape(9, 9),
                velocity if message.velocity_observed else None, rotation, float(message.confidence),
                message.header.frame_id, message.unit, message.state,
            ).validate(self.config.frame_id)
            if not 0 <= now - stamp <= self.config.max_age_ns or record.receive_ns > now:
                raise ValueError("stale_or_future_kinematic_observation")
            key = self._key(record)
            if key in self._pending:
                raise ValueError("duplicate_pending_target_estimate")
            self._pending[key] = (record, now)
            while len(self._pending) > self._queue_size:
                old = min(self._pending, key=lambda item: item[3])
                self._pending.pop(old)
                self._invalid(old[0], old[3], "target_fusion_queue_full")
                self._status("target_fusion_queue_full", old[3], old[0], decision="rejected")
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            self._status(str(error), stamp, identifier, decision="rejected")
            self._invalid(identifier, stamp, str(error))

    def _quality_callback(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or data.get("schema_version") != 1:
                raise ValueError("target_quality_schema_invalid")
            if data.get("subject") != "target" or "source" not in data or "position_reference" not in data:
                return  # Other producers share this topic; never cross-pair.
            key = (data["target_id"], data["source"], data["position_reference"], data["sample_timestamp_ns"])
            now = self.get_clock().now().nanoseconds
            if (any(not isinstance(value, str) or not value for value in key[:3])
                    or not isinstance(key[3], int) or isinstance(key[3], bool) or not 0 < key[3] <= now
                    or now - key[3] > self.config.max_age_ns or not isinstance(data["features"], dict)):
                raise ValueError("target_quality_identity_or_time_invalid")
            # Quality packets can carry extra diagnostics for the scheduler;
            # the fixed learned-reliability feature contract consumes only its
            # declared subset rather than rejecting the whole observation.
            feature_row({name: data["features"].get(name) for name in FEATURE_NAMES})
            self._quality[key] = dict(data["features"])
            while len(self._quality) > self._queue_size:
                del self._quality[min(self._quality, key=lambda item: item[3])]
        except (KeyError, TypeError, ValueError) as error:
            self._status(str(error), 0, decision="quality_rejected")

    def _scale(self, record, values):
        if self._mode == "fixed":
            return 1.
        outlier = float(values.get("outlier_probability", 0.))
        if not np.isfinite(outlier) or not 0. <= outlier <= 1.:
            raise ValueError("invalid target outlier_probability")
        if self._mode == "rule":
            # Identity confidence and geometric outlier risk measure different
            # failure modes. Both may only inflate observation covariance.
            return float(1. + 99. * (1. - record.confidence) ** 2 + 99. * outlier ** 2)
        features = {name: values.get(name) for name in FEATURE_NAMES}
        features["network_latency_s"] = (record.receive_ns - record.stamp_ns) / 1e9
        track = self.estimator.tracks.get(record.target_id)
        if track is not None:
            features["dt_s"] = (record.stamp_ns - track.estimate.stamp_ns) / 1e9
        learned = float(covariance_scales(self._model.predict(features))[0])
        # Preserve the learned model's feature contract while making an
        # explicit online geometric rejection risk effective in all modes.
        return max(learned, 1. + 99. * outlier ** 2)

    def _drain(self):
        now = self.get_clock().now().nanoseconds
        self._quality = {key: value for key, value in self._quality.items() if 0 <= now - key[3] <= self.config.max_age_ns}
        waiting = set()
        for key, (record, received) in sorted(list(self._pending.items()), key=lambda item: item[0][3]):
            if record.target_id in waiting:
                continue
            matched = key in self._quality
            if (self._mode == "learned" and not matched and now - received < self._quality_wait_ns
                    and now - record.stamp_ns <= self.config.max_age_ns):
                waiting.add(record.target_id)
                continue
            self._pending.pop(key)
            values = self._quality.pop(key, {})
            try:
                scale = self._scale(record, values)
                candidate = deepcopy(self.estimator)
                result = candidate.update(record, now, covariance_scale=scale)
                end = self.get_clock().now().nanoseconds
                if not 0 <= end - record.stamp_ns <= self.config.max_age_ns:
                    raise ValueError("target_fusion_solution_stale")
                self.estimator = candidate
                diagnostic = dict(candidate.last_diagnostic)
                reason = diagnostic.pop("reason", "")
                diagnostic.pop("target_id", None)
                self._status(reason, record.stamp_ns, record.target_id, quality_matched=matched,
                             counts=dict(candidate.counts), **diagnostic)
                if result is not None:
                    self._publish(result)
                    self._last_lost.pop(record.target_id, None)
                else:
                    self._invalid(record.target_id, record.stamp_ns, reason)
            except (ValueError, np.linalg.LinAlgError) as error:
                self._invalid(record.target_id, record.stamp_ns, str(error))
                self._status(str(error), record.stamp_ns, record.target_id, decision="rejected", quality_matched=matched)
            now = self.get_clock().now().nanoseconds

    def _watchdog(self):
        now = self.get_clock().now().nanoseconds
        self._invalid_stamps = {
            key: stamp for key, stamp in self._invalid_stamps.items() if now - stamp <= self.config.retention_ns}
        for identifier, track in list(self.estimator.tracks.items()):
            if now < track.estimate.stamp_ns:
                continue
            if now - track.estimate.stamp_ns <= self.config.max_age_ns and not track.invalid_reason:
                continue
            reason = self.estimator.association_block_reason(identifier) or "target_not_observed"
            if self._last_lost.get(identifier) != (track.estimate.stamp_ns, reason):
                self._invalid(identifier, now, reason)
                self._status(reason, now, identifier, decision="lost")
                self._last_lost[identifier] = track.estimate.stamp_ns, reason
        self.estimator.expire(now)
        self._last_lost = {key: value for key, value in self._last_lost.items() if key in self.estimator.tracks}


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetKinematicFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
