#!/usr/bin/env python3
"""Record the current FanciSwarm MAVLink UWB demo stream for replay.

The recorder preserves the existing temporary protocol adapter: four values
from BATTERY_STATUS.voltages[2:6] are written as UWB ranges in metres. This
is suitable for communication/replay testing only until the vendor's real
range message is confirmed.
"""
from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(ROOT / "code" / "analysis"))
sys.path.insert(0, str(ROOT / "simulation"))

from algorithms.uwb.live_protocol import ANCHORS, parse_battery_distances
from algorithms.uwb.live_protocol import convert_fc_position


RANGE_FIELDS = (
    "frame_seq", "sample_timestamp_ns", "receive_timestamp_ns", "tag_id",
    "anchor_id", "anchor_x_m", "anchor_y_m", "anchor_z_m", "range_m",
    "stddev_m", "source_message",
)
POSITION_FIELDS = (
    "sample_timestamp_ns", "receive_timestamp_ns", "x_m", "y_m", "z_m",
    "source_message", "reference_semantics", "raw_usec", "raw_x", "raw_y",
    "raw_z",
)


class FanciSwarmUwbRecorder:
    """Persist the two MAVLink streams needed by the UWB replay path."""

    def __init__(self, connection, output_dir, *, clock=None, heartbeat_period_s=1.0):
        self.connection = connection
        self.output_dir = Path(output_dir)
        self.clock = clock or time.time_ns
        self.heartbeat_period_ns = int(float(heartbeat_period_s) * 1e9)
        self.last_heartbeat_ns = 0
        self.frame_seq = 0
        self.message_counts = {}
        self.invalid_battery_count = 0
        self.range_stream = None
        self.position_stream = None
        self.range_writer = None
        self.position_writer = None

    def open(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.range_stream = (self.output_dir / "uwb_range.csv").open(
            "w", encoding="utf-8", newline="")
        self.position_stream = (self.output_dir / "fc_position.csv").open(
            "w", encoding="utf-8", newline="")
        self.range_writer = csv.DictWriter(self.range_stream, fieldnames=RANGE_FIELDS)
        self.position_writer = csv.DictWriter(self.position_stream, fieldnames=POSITION_FIELDS)
        self.range_writer.writeheader()
        self.position_writer.writeheader()
        self.range_stream.flush()
        self.position_stream.flush()

    def close(self):
        for stream in (self.range_stream, self.position_stream):
            if stream is not None:
                stream.close()
        self.range_stream = self.position_stream = None
        self.range_writer = self.position_writer = None
        self._write_metadata()

    def _write_metadata(self):
        metadata = {
            "data_type": "measured",
            "is_measured_result": False,
            "reference_position_is_ground_truth": False,
            "range_source": "BATTERY_STATUS.voltages[2:6] temporary adapter",
            "range_unit": "m",
            "range_stddev_m": 0.05,
            "anchor_configuration": [
                {"anchor_id": anchor_id, "position_m": list(position)}
                for anchor_id, position in ANCHORS
            ],
            "sample_timestamp_source": (
                "ground_receive_time; MAVLink BATTERY_STATUS has no verified UWB sample timestamp"
            ),
            "fc_position_semantics": "flight_controller_reference_only",
            "message_counts": self.message_counts,
            "invalid_battery_count": self.invalid_battery_count,
        }
        (self.output_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def _send_heartbeat(self, now_ns):
        if now_ns - self.last_heartbeat_ns < self.heartbeat_period_ns:
            return
        mav = getattr(self.connection, "mav", None)
        if mav is None or not hasattr(mav, "heartbeat_encode"):
            return
        # An outgoing heartbeat registers this udpout socket with the Pi
        # bridge, which then sends UART telemetry back to its ephemeral port.
        protocol = getattr(self.connection, "mavlink", None)
        if protocol is None:
            try:
                from pymavlink import mavutil
                protocol = mavutil.mavlink
            except ImportError:
                protocol = mav
        autopilot_invalid = getattr(protocol, "MAV_AUTOPILOT_INVALID", 8)
        mav_type_onboard = getattr(protocol, "MAV_TYPE_ONBOARD_CONTROLLER", 18)
        mav_state_active = getattr(protocol, "MAV_STATE_ACTIVE", 4)
        message = mav.heartbeat_encode(
            mav_type_onboard, autopilot_invalid, 0, 0, mav_state_active)
        mav.send(message, force_mavlink1=True)
        self.last_heartbeat_ns = now_ns

    def record_message(self, message, *, receive_timestamp_ns=None):
        if message is None:
            return False
        receive_ns = int(self.clock() if receive_timestamp_ns is None else receive_timestamp_ns)
        message_type = message.get_type()
        self.message_counts[message_type] = self.message_counts.get(message_type, 0) + 1

        if message_type == "BATTERY_STATUS":
            try:
                distances = parse_battery_distances(message.voltages)
            except (AttributeError, TypeError, ValueError):
                self.invalid_battery_count += 1
                return False
            self.frame_seq += 1
            for (anchor_id, position), distance in zip(ANCHORS, distances):
                self.range_writer.writerow({
                    "frame_seq": self.frame_seq,
                    "sample_timestamp_ns": receive_ns,
                    "receive_timestamp_ns": receive_ns,
                    "tag_id": "2",
                    "anchor_id": anchor_id,
                    "anchor_x_m": position[0],
                    "anchor_y_m": position[1],
                    "anchor_z_m": position[2],
                    "range_m": distance,
                    "stddev_m": 0.05,
                    "source_message": "BATTERY_STATUS.voltages[2:6]",
                })
            self.range_stream.flush()
            return True

        if message_type == "GLOBAL_VISION_POSITION_ESTIMATE":
            try:
                position = convert_fc_position(message)
            except (AttributeError, TypeError, ValueError, OverflowError):
                return False
            self.position_writer.writerow({
                "sample_timestamp_ns": receive_ns,
                "receive_timestamp_ns": receive_ns,
                "x_m": position[0],
                "y_m": position[1],
                "z_m": position[2],
                "source_message": message_type,
                "reference_semantics": "flight_controller_reference_only",
                "raw_usec": getattr(message, "usec", ""),
                "raw_x": getattr(message, "x", ""),
                "raw_y": getattr(message, "y", ""),
                "raw_z": getattr(message, "z", ""),
            })
            self.position_stream.flush()
            return True

        return False

    def run(self, *, duration_s=None, poll_timeout_s=0.2):
        if self.range_writer is None:
            self.open()
        started_ns = int(self.clock())
        deadline_ns = None if duration_s is None else started_ns + int(float(duration_s) * 1e9)
        try:
            while deadline_ns is None or int(self.clock()) < deadline_ns:
                now_ns = int(self.clock())
                self._send_heartbeat(now_ns)
                message = self.connection.recv_match(
                    blocking=True, timeout=float(poll_timeout_s))
                self.record_message(message, receive_timestamp_ns=int(self.clock()))
        finally:
            self.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udpout:192.168.1.143:14550")
    parser.add_argument("--output-dir", required=True,
                        help="experiment directory for uwb_range.csv, fc_position.csv, and metadata.json")
    parser.add_argument("--duration-s", type=float,
                        help="stop after this many seconds; omit to record until Ctrl-C")
    parser.add_argument("--heartbeat-period-s", type=float, default=1.0)
    parser.add_argument("--poll-timeout-s", type=float, default=0.2)
    args = parser.parse_args(argv)
    if args.duration_s is not None and args.duration_s <= 0:
        parser.error("--duration-s must be positive")
    if args.heartbeat_period_s <= 0 or args.poll_timeout_s <= 0:
        parser.error("heartbeat and poll timeouts must be positive")
    try:
        from pymavlink import mavutil
        connection = mavutil.mavlink_connection(
            args.connection, source_system=254, source_component=191,
            force_mavlink1=True)
        print(f"MAVLink recorder connection: {args.connection}")
        recorder = FanciSwarmUwbRecorder(
            connection, args.output_dir,
            heartbeat_period_s=args.heartbeat_period_s)
        recorder.run(duration_s=args.duration_s, poll_timeout_s=args.poll_timeout_s)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
