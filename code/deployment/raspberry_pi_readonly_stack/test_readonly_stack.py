import json
import sys
import tempfile
import socket
import time
import unittest
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
LEGACY = HERE.parent / "raspberry_pi_uav_readonly_adapter"
CAMERA = HERE.parent / "raspberry_pi_e016_self_localization" / "app"
sys.path[:0] = [str(HERE), str(LEGACY), str(CAMERA)]

from capture_live_camera_v2 import write_frame
from readonly_health_check import evaluate
from uav_readonly_adapter_v2 import (
    ReadOnlyMavlinkAdapterV2,
    StateProjectorV2,
    TcpJsonPublisher,
)
from uav_readonly_adapter import AdapterConfig


class Message:
    def __init__(self, kind, **fields):
        self.kind = kind
        self.__dict__.update(fields)

    def get_type(self):
        return self.kind


class Frame:
    rgb = np.zeros((4, 6, 3), dtype=np.uint8)
    captured_monotonic_ns = 10
    sensor_timestamp_ns = 9
    exposure_time_us = 100
    analogue_gain = 1.0


class ReadOnlyV2Tests(unittest.TestCase):
    def test_hardware_configuration_matches_confirmed_inputs(self):
        config = yaml.safe_load((HERE / "config/hardware_user_v01.yaml").read_text())
        self.assertEqual(config["uwb"]["tag_id"], 2)
        self.assertEqual(config["uwb"]["range_slot_anchor_ids"], ["1", "2", "3", "4"])
        self.assertEqual(config["map"]["yaw_offset_deg"], 9.0)
        self.assertEqual(config["camera"]["mounting"], "bottom_lens_down")
        self.assertEqual(config["verification_status"], "user_provided_not_surveyed")

    def test_new_stack_has_no_mavlink_control_send_calls(self):
        source = (HERE / "uav_readonly_adapter.py").read_text()
        source += (HERE / "uav_readonly_adapter_v2.py").read_text()
        forbidden = (
            "command_long_send", "set_mode_send", "mission_item_send",
            "set_position_target", "param_set_send", "manual_control_send",
        )
        self.assertTrue(all(name not in source for name in forbidden))
        self.assertIn("heartbeat_send", source)

    def test_tag2_imu_conversion_and_uwb_timestamp(self):
        projector = StateProjectorV2(AdapterConfig(uwb_tag_id=2))
        projector.process(Message("SCALED_IMU", time_boot_ms=12, xacc=1000, yacc=-2000,
                                  zacc=-9870, xgyro=1, ygyro=2, zgyro=3), now_monotonic=1)
        projector.process(Message("BATTERY_STATUS", voltages=[0, 12000, 100, 200, 300, 400],
                                  current_battery=0, battery_remaining=50),
                          now_monotonic=1, now_unix_ns=123456)
        state = projector.snapshot(now_unix=1, now_monotonic=1)
        self.assertEqual(state["uwb"]["tag_id"], 2)
        self.assertEqual(state["uwb"]["anchor_ids"], ["1", "2", "3", "4"])
        self.assertEqual(state["uwb"]["sample_timestamp_ns"], 123456)
        self.assertEqual(state["imu"]["acceleration_mps2"], {"x": 1.0, "y": -2.0, "z": -9.87})

    def test_camera_metadata_and_health(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_frame(root, Frame(), 180, captured_unix_ns=1_000_000_000)
            metadata = json.loads((root / "latest.json").read_text())
            self.assertEqual(metadata["sample_timestamp_domain"], "ros_unix_ns_receive")
            report = evaluate(None, root / "latest.json", now_ns=1_100_000_000)
            self.assertTrue(report["ok"])

    def test_incomplete_battery_packet_cannot_refresh_old_uwb(self):
        projector = StateProjectorV2(AdapterConfig(uwb_tag_id=2))
        projector.process(Message("BATTERY_STATUS", voltages=[0, 12000, 100, 200, 300, 400],
                                  current_battery=0, battery_remaining=50),
                          now_monotonic=1, now_unix_ns=123456)
        projector.process(Message("BATTERY_STATUS", voltages=[0, 12000],
                                  current_battery=0, battery_remaining=50),
                          now_monotonic=2, now_unix_ns=223456)
        state = projector.snapshot(now_unix=1, now_monotonic=2)
        self.assertEqual(state["uwb"]["anchor_ranges_m"], [None] * 4)
        self.assertIsNone(state["uwb"]["sample_timestamp_ns"])

    def test_tcp_publisher_streams_json_without_control_transport(self):
        publisher = TcpJsonPublisher(listen_host="127.0.0.1", listen_port=0)
        client = socket.create_connection(publisher.server.getsockname(), timeout=1)
        client.settimeout(1)
        try:
            # TCP accept and data publication are intentionally handled by the
            # continuous publish loop; a connection arriving between ticks
            # receives the following record.
            for _ in range(3):
                publisher.publish({"schema_version": "2.0", "online": True})
                time.sleep(0.01)
            self.assertTrue(json.loads(client.makefile("rb").readline().decode())["online"])
        finally:
            client.close()
            publisher.close()

    def test_each_scaled_imu_is_forwarded(self):
        class Publisher:
            def __init__(self):
                self.states = []

            def publish(self, state):
                self.states.append(state)

        class Mav:
            def heartbeat_send(self, *_):
                pass

        class Mavutil:
            class mavlink:
                MAV_TYPE_GCS = 6
                MAV_AUTOPILOT_INVALID = 8
                MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
                MAV_STATE_ACTIVE = 4

        publisher = Publisher()
        adapter = ReadOnlyMavlinkAdapterV2(AdapterConfig(publish_interval_s=10), publisher)

        class Connection:
            mav = Mav()

            def __init__(self):
                self.messages = [
                    Message("SCALED_IMU", time_boot_ms=10, xacc=1, yacc=2, zacc=3,
                            xgyro=4, ygyro=5, zgyro=6),
                    Message("SCALED_IMU", time_boot_ms=15, xacc=1, yacc=2, zacc=3,
                            xgyro=4, ygyro=5, zgyro=6),
                ]

            def recv_match(self, **_):
                if self.messages:
                    return self.messages.pop(0)
                adapter.running = False
                return None

        adapter._run_connection(Connection(), Mavutil())
        stamps = [state["imu"]["sample_timestamp_us"] for state in publisher.states
                  if state["imu"]["sample_timestamp_us"] is not None]
        self.assertIn(10_000, stamps)
        self.assertIn(15_000, stamps)


if __name__ == "__main__":
    unittest.main()
