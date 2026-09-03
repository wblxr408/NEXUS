import json
import socket
import unittest
from io import StringIO

from uav_readonly_adapter import AdapterConfig, JsonPublisher, StateProjector


class FakeMessage:
    def __init__(self, message_type, **fields):
        self._message_type = message_type
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self):
        return self._message_type


class StateProjectorTests(unittest.TestCase):
    def setUp(self):
        self.projector = StateProjector(AdapterConfig())

    def test_vendor_position_is_converted_from_cm_to_m(self):
        self.projector.process(
            FakeMessage(
                "GLOBAL_VISION_POSITION_ESTIMATE",
                x=329.1619,
                y=513.1796,
                z=1.7142,
                roll=0.0039,
                pitch=-0.0318,
                yaw=-1.6691,
            ),
            now_monotonic=10.0,
        )
        state = self.projector.snapshot(now_unix=1000.0, now_monotonic=10.5)
        self.assertEqual(state["position"]["x_m"], 3.2916)
        self.assertEqual(state["position"]["y_m"], 5.1318)
        self.assertIsNone(state["position"]["z_m"])
        self.assertIsNone(state["position"]["sample_timestamp_ns"])
        self.assertTrue(state["online"])

    def test_heartbeat_maps_lock_state_without_issuing_commands(self):
        self.projector.process(
            FakeMessage("HEARTBEAT", base_mode=0, custom_mode=7, system_status=3),
            now_monotonic=1.0,
        )
        self.assertFalse(self.projector.snapshot(now_monotonic=1.0)["armed"])
        self.projector.process(
            FakeMessage("HEARTBEAT", base_mode=128, custom_mode=7, system_status=4),
            now_monotonic=2.0,
        )
        self.assertTrue(self.projector.snapshot(now_monotonic=2.0)["armed"])

    def test_velocity_battery_and_vendor_uwb_mapping(self):
        self.projector.process(
            FakeMessage("GLOBAL_POSITION_INT", vx=12, vy=-34, vz=5),
            now_monotonic=1.0,
        )
        self.projector.process(
            FakeMessage(
                "BATTERY_STATUS",
                voltages=[4722, 4517, 664, 381, 200, 551],
                current_battery=20,
                battery_remaining=0,
            ),
            now_monotonic=1.1,
        )
        state = self.projector.snapshot(now_monotonic=1.1)
        self.assertEqual(state["velocity_mps"], {"x": 0.12, "y": -0.34, "z": 0.05})
        self.assertEqual(state["battery"]["voltage_v"], 4.517)
        self.assertEqual(state["battery"]["current_a"], 0.2)
        self.assertEqual(state["uwb"]["anchor_ranges_m"], [6.64, 3.81, 2.0, 5.51])

    def test_scaled_imu_uses_vendor_units_and_preserves_sample_time(self):
        self.projector.process(
            FakeMessage(
                "SCALED_IMU", time_boot_ms=1234,
                xacc=-582, yacc=44, zacc=-9873,
                xgyro=1000, ygyro=-250, zgyro=11,
            ), now_monotonic=1.0,
        )
        imu = self.projector.snapshot(now_monotonic=1.0)["imu"]
        self.assertEqual(imu["sample_timestamp_us"], 1_234_000)
        self.assertEqual(imu["sample_timestamp_domain"], "flight_boot_unverified")
        self.assertEqual(imu["acceleration_mps2"], {"x": -0.582, "y": 0.044, "z": -9.873})
        self.assertEqual(imu["angular_velocity_rps"], {"x": 1.0, "y": -0.25, "z": 0.011})
        self.assertEqual(imu["unit_source"], "vendor_scaled_imu_mm_s2_mrad_s")

    def test_unavailable_uwb_ranges_become_null_instead_of_655_metres(self):
        self.projector.process(
            FakeMessage(
                "BATTERY_STATUS",
                voltages=[4722, 4517, 0, 65535, 200, 551],
                current_battery=-1,
                battery_remaining=-1,
            ),
            now_monotonic=1.0,
        )
        state = self.projector.snapshot(now_monotonic=1.0)
        self.assertEqual(state["uwb"]["anchor_ranges_m"], [None, None, 2.0, 5.51])

    def test_state_becomes_offline_when_stale(self):
        self.projector.process(FakeMessage("HEARTBEAT", base_mode=0), now_monotonic=10.0)
        self.assertTrue(self.projector.snapshot(now_monotonic=12.9)["online"])
        self.assertFalse(self.projector.snapshot(now_monotonic=13.1)["online"])

    def test_stale_position_is_cleared_instead_of_being_republished(self):
        self.projector.process(
            FakeMessage("GLOBAL_VISION_POSITION_ESTIMATE", x=100, y=200, z=0),
            now_monotonic=1.0,
        )
        state = self.projector.snapshot(now_monotonic=4.1)
        self.assertIsNone(state["position"]["x_m"])
        self.assertIsNone(state["position"]["y_m"])

    def test_malformed_optional_fields_do_not_crash_projection(self):
        self.projector.process(FakeMessage("HEARTBEAT", base_mode=None), now_monotonic=1.0)
        self.projector.process(
            FakeMessage("BATTERY_STATUS", voltages=[None, None, None, None, None, None],
                        current_battery=None, battery_remaining=None),
            now_monotonic=1.1,
        )
        self.assertFalse(self.projector.snapshot(now_monotonic=1.1)["armed"])

    def test_json_publisher_emits_one_valid_json_object_per_line(self):
        publisher = JsonPublisher()
        publisher.stream = StringIO()
        publisher.publish({"agent_id": "drone_001", "online": True})
        decoded = json.loads(publisher.stream.getvalue())
        self.assertEqual(decoded["agent_id"], "drone_001")

    def test_json_publisher_can_fan_out_to_udp_without_touching_serial(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1.0)
        host, port = receiver.getsockname()
        publisher = JsonPublisher(udp_host=host, udp_port=port)
        try:
            publisher.publish({"schema_version": "1.0", "online": True})
            payload, _ = receiver.recvfrom(4096)
        finally:
            publisher.close()
            receiver.close()
        self.assertTrue(json.loads(payload.decode("utf-8"))["online"])


if __name__ == "__main__":
    unittest.main()
