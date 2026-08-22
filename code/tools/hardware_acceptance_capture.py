#!/usr/bin/env python3
"""Capture non-sensitive evidence for the first ROS1 hardware acceptance.

The tool never copies a rosbag or an SD log.  It records only ROS command
output, SD CSV headers, file sizes, and SHA256 values so that large raw data
can remain in the approved external location.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_TOPICS = ("/odom_global_001", "/imu_global_001")


def utc_now():
    """Return one UTC instant in ISO-8601 and Unix-nanosecond forms."""
    now = datetime.now(timezone.utc)
    return {
        "utc": now.isoformat().replace("+00:00", "Z"),
        "unix_timestamp_ns": int(now.timestamp() * 1_000_000_000),
    }


def safe_topic_name(topic):
    """Make a topic usable as an evidence filename without changing the topic."""
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", topic.strip("/")) or "root"


def run_command(command, timeout_s):
    """Run a read-only diagnostic command and retain its stdout/stderr."""
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout_s, check=False)
    except FileNotFoundError as error:
        return {
            "command": command,
            "returncode": 127,
            "stdout": "",
            "stderr": str(error),
        }
    except subprocess.TimeoutExpired as error:
        return {
            "command": command,
            "returncode": 124,
            "stdout": error.stdout or "",
            "stderr": (error.stderr or "") + f"\ncommand timed out after {timeout_s}s",
        }
    return {
        "command": command,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_sd_csv(path):
    """Return metadata without exporting the SD CSV body."""
    resolved = path.expanduser().resolve()
    record = {
        "path": str(resolved),
        "exists": resolved.is_file(),
        "data_copied_into_repository": False,
    }
    if not resolved.is_file():
        record["error"] = "file_not_found"
        return record
    try:
        with resolved.open("r", encoding="utf-8-sig", newline="") as stream:
            header = next(csv.reader(stream), [])
    except (OSError, UnicodeError, csv.Error) as error:
        record["error"] = f"header_read_failed: {error}"
        return record
    record.update({
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
        "header": header,
    })
    if not header:
        record["error"] = "empty_csv_header"
    return record


def write_text(path, content):
    path.write_text(content, encoding="utf-8")


def capture_topics(output_dir, topics, timeout_s):
    list_result = run_command(["rostopic", "list"], timeout_s)
    write_text(output_dir / "rostopic_list.txt", list_result["stdout"])
    write_text(output_dir / "rostopic_list.stderr.txt", list_result["stderr"])

    topic_records = []
    for topic in topics:
        stem = safe_topic_name(topic)
        type_result = run_command(["rostopic", "type", topic], timeout_s)
        echo_result = run_command(["rostopic", "echo", "-n", "1", topic], timeout_s)
        write_text(output_dir / f"topic_{stem}.type.txt", type_result["stdout"])
        write_text(output_dir / f"topic_{stem}.type.stderr.txt", type_result["stderr"])
        write_text(output_dir / f"topic_{stem}.echo.yaml", echo_result["stdout"])
        write_text(output_dir / f"topic_{stem}.echo.stderr.txt", echo_result["stderr"])
        topic_records.append({
            "topic": topic,
            "type_returncode": type_result["returncode"],
            "echo_returncode": echo_result["returncode"],
            "type_file": f"topic_{stem}.type.txt",
            "echo_file": f"topic_{stem}.echo.yaml",
        })
    return list_result, topic_records


def metadata_draft(captured_at, device_time_source, sd_records):
    """Create a JSON draft that leaves hardware facts explicitly unconfirmed."""
    return {
        "metadata_status": "draft_from_hardware_acceptance",
        "captured_at": captured_at,
        "data_type": "measured",
        "run_id": "TBD",
        "source": "flight_controller_sd_csv",
        "device": "TBD",
        "device_time_source": device_time_source,
        "coordinate_frame": "TBD",
        "axis_convention": "TBD",
        "position_unit": "TBD",
        "target_or_platform_semantic_object": "unknown",
        "raw_data_external_only": True,
        "sd_csv_files": sd_records,
        "notes": [
            "This draft is not a calibration record or an accuracy conclusion.",
            "Confirm the vendor SD header, frame, unit, time source, and physical object before conversion.",
        ],
    }


def capture(output_dir, topics, sd_csv_paths, device_time_source, timeout_s):
    """Write an acceptance evidence bundle and return its machine-readable summary."""
    output_dir.mkdir(parents=True, exist_ok=True)
    captured_at = utc_now()
    list_result, topic_records = capture_topics(output_dir, topics, timeout_s)
    version_commands = {
        "ros_distro": ["rosversion", "-d"],
        "roscpp": ["rosversion", "roscpp"],
        "roslaunch": ["rosversion", "roslaunch"],
        "git_commit": ["git", "rev-parse", "HEAD"],
    }
    versions = {
        name: run_command(command, timeout_s)
        for name, command in version_commands.items()
    }
    time_status = {
        "host_capture_clock": captured_at,
        "host_platform": platform.platform(),
        "ros_use_sim_time": run_command(["rosparam", "get", "/use_sim_time"], timeout_s),
        "host_time_sync": run_command(
            ["timedatectl", "show", "--property=NTPSynchronized", "--property=NTP",
             "--property=LocalRTC"], timeout_s),
        "device_message_timestamp_source": device_time_source,
        "device_message_timestamp_source_note": (
            "Do not infer this from ROS receipt time. Confirm from the device/manual "
            "and the captured message header."),
    }
    sd_records = [inspect_sd_csv(path) for path in sd_csv_paths]
    draft = metadata_draft(captured_at, device_time_source, sd_records)

    required_failures = (
        list_result["returncode"] != 0
        or any(item["type_returncode"] != 0 or item["echo_returncode"] != 0
               for item in topic_records)
        or any("error" in item for item in sd_records)
    )
    summary = {
        "capture_status": "partial" if required_failures else "complete",
        "captured_at": captured_at,
        "requested_topics": list(topics),
        "topic_records": topic_records,
        "sd_csv_files": sd_records,
        "unverified_hardware_facts": {
            "device_message_timestamp_source": device_time_source,
            "coordinate_frame": "TBD",
            "position_unit": "TBD",
            "semantic_object": "unknown",
        },
        "sensitive_data_policy": (
            "No device serial number, credential, raw rosbag, or SD CSV body is copied "
            "by this tool."),
    }
    (output_dir / "versions.json").write_text(
        json.dumps(versions, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output_dir / "time_source.json").write_text(
        json.dumps(time_status, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output_dir / "metadata_draft.json").write_text(
        json.dumps(draft, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output_dir / "capture_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Capture ROS1 first-day acceptance evidence without copying raw data")
    parser.add_argument("--output-dir", required=True,
                        help="new or empty evidence directory, normally under a run artifacts directory")
    parser.add_argument("--topic", action="append", dest="topics",
                        help="ROS1 topic to inspect; repeat as needed (default: official odom and imu topics)")
    parser.add_argument("--sd-csv", action="append", default=[], type=Path,
                        help="external flight-controller SD CSV; only header, size, and SHA256 are recorded")
    parser.add_argument("--device-time-source", default="TBD",
                        help="confirmed device sample-time source, or keep TBD until verified")
    parser.add_argument("--timeout-s", default=5.0, type=float,
                        help="timeout for each read-only ROS/system command")
    parser.add_argument("--strict", action="store_true",
                        help="return non-zero when a requested ROS command or SD header check fails")
    args = parser.parse_args(argv)
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    topics = tuple(args.topics or DEFAULT_TOPICS)
    summary = capture(
        Path(args.output_dir), topics, args.sd_csv, args.device_time_source, args.timeout_s)
    print(json.dumps({
        "capture_status": summary["capture_status"],
        "output_dir": str(Path(args.output_dir).resolve()),
    }, ensure_ascii=False))
    return 1 if args.strict and summary["capture_status"] != "complete" else 0


if __name__ == "__main__":
    sys.exit(main())
