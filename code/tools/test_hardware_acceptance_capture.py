import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))

import hardware_acceptance_capture as capture


def test_safe_topic_name_is_stable_for_evidence_files():
    assert capture.safe_topic_name("/nexus/fcu/odom") == "nexus_fcu_odom"
    assert capture.safe_topic_name("/") == "root"


def test_inspect_sd_csv_records_header_hash_and_not_raw_data(tmp_path):
    source = tmp_path / "flight_log.csv"
    source.write_text("timestamp,x,y,z\n1,2,3,4\n", encoding="utf-8")

    record = capture.inspect_sd_csv(source)

    assert record["header"] == ["timestamp", "x", "y", "z"]
    assert len(record["sha256"]) == 64
    assert record["data_copied_into_repository"] is False


def test_capture_writes_partial_bundle_when_ros_is_unavailable(tmp_path, monkeypatch):
    def fake_command(command, timeout_s):
        if command[:2] == ["rostopic", "list"]:
            return {"command": command, "returncode": 0, "stdout": "/odom_global_001\n", "stderr": ""}
        if command[:2] == ["rostopic", "type"]:
            return {"command": command, "returncode": 0, "stdout": "nav_msgs/Odometry\n", "stderr": ""}
        if command[:2] == ["rostopic", "echo"]:
            return {"command": command, "returncode": 0, "stdout": "header: {}\n", "stderr": ""}
        return {"command": command, "returncode": 0, "stdout": "value\n", "stderr": ""}

    monkeypatch.setattr(capture, "run_command", fake_command)
    sd_csv = tmp_path / "sd.csv"
    with sd_csv.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerow(["time_ns", "x"])

    output = tmp_path / "evidence"
    summary = capture.capture(
        output, ("/odom_global_001",), [sd_csv], "fcu_header_stamp", 1.0)

    assert summary["capture_status"] == "complete"
    assert (output / "rostopic_list.txt").read_text(encoding="utf-8") == "/odom_global_001\n"
    metadata = json.loads((output / "metadata_draft.json").read_text(encoding="utf-8"))
    assert metadata["device_time_source"] == "fcu_header_stamp"
    assert metadata["coordinate_frame"] == "TBD"
