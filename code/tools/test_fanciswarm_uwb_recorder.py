import csv
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).parents[2] / "tools" / "fanciswarm_uwb_recorder.py"
SPEC = importlib.util.spec_from_file_location("fanciswarm_uwb_recorder", MODULE_PATH)
recorder_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder_module)


class FakeConnection:
    mav = SimpleNamespace()


def message(message_type, **fields):
    return SimpleNamespace(get_type=lambda: message_type, **fields)


def test_udpout_registration_heartbeat_is_sent_once_per_period():
    sent = []

    class FakeMav:
        def heartbeat_encode(self, *args):
            return ("heartbeat", args)

        def send(self, packet, **kwargs):
            sent.append((packet, kwargs))

    connection = SimpleNamespace(mav=FakeMav(), mavlink=SimpleNamespace(
        MAV_AUTOPILOT_INVALID=8, MAV_TYPE_ONBOARD_CONTROLLER=18, MAV_STATE_ACTIVE=4))
    recorder = recorder_module.FanciSwarmUwbRecorder(
        connection, "/tmp/unused-recorder-test", heartbeat_period_s=1.0)
    recorder._send_heartbeat(1_000_000_000)
    recorder._send_heartbeat(1_500_000_000)
    recorder._send_heartbeat(2_000_000_000)
    assert len(sent) == 2
    assert sent[0][1] == {"force_mavlink1": True}


def test_recorder_writes_algorithm_compatible_ranges_and_reference_position(tmp_path):
    recorder = recorder_module.FanciSwarmUwbRecorder(
        FakeConnection(), tmp_path, clock=lambda: 1_000)
    recorder.open()
    assert recorder.record_message(
        message("BATTERY_STATUS", voltages=[0, 0, 341, 354, 488, 443]),
        receive_timestamp_ns=1_000,
    )
    assert recorder.record_message(
        message("GLOBAL_VISION_POSITION_ESTIMATE", usec=9, x=100, y=-250, z=300),
        receive_timestamp_ns=1_100,
    )
    recorder.close()

    with (tmp_path / "uwb_range.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 4
    assert rows[0]["frame_seq"] == "1"
    assert rows[0]["anchor_id"] == "1"
    assert float(rows[0]["range_m"]) == 3.41
    assert rows[0]["source_message"] == "BATTERY_STATUS.voltages[2:6]"

    with (tmp_path / "fc_position.csv").open(encoding="utf-8", newline="") as stream:
        positions = list(csv.DictReader(stream))
    assert positions[0]["reference_semantics"] == "flight_controller_reference_only"
    assert (float(positions[0]["x_m"]), float(positions[0]["y_m"]), float(positions[0]["z_m"])) == (1.0, -2.5, -3.0)

    metadata = json.loads((tmp_path / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["reference_position_is_ground_truth"] is False
