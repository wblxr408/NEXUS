"""Normalize ROS/flight-controller exports without assigning accuracy meaning."""

from __future__ import annotations

import csv
import math
import subprocess
from pathlib import Path


OUTPUT_FIELDS = (
    "sample_timestamp_ns", "receive_timestamp_ns", "target_id", "frame_id",
    "x_m", "y_m", "z_m", "source_mode", "validity", "invalid_reason",
    "unit", "confidence", "reprojection_error_px", "data_type",
)


def _first(row, names, default=""):
    for name in names:
        if name in row and str(row[name]).strip() != "":
            return row[name]
    return default


def _timestamp_ns(row, receive=False):
    ns_names = (
        ("receive_timestamp_ns", "receive_time_ns", "%time") if receive else
        ("sample_timestamp_ns", "timestamp_ns", "time_ns", "field.header.stamp")
    )
    value = _first(row, ns_names, "")
    if value != "":
        return int(float(value))
    seconds_names = (
        ("receive_timestamp_s", "receive_time_s") if receive else
        ("sample_timestamp_s", "timestamp_s", "time_s")
    )
    value = _first(row, seconds_names, "")
    return int(float(value) * 1e9) if value != "" else 0


def _position(row, axis):
    names = (
        f"{axis}_m", axis, f"position_{axis}", f"pos_{axis}",
        f"field.pose.position.{axis}", f"field.pose.pose.position.{axis}",
    )
    value = _first(row, names, "")
    return float(value) if value != "" else math.nan


def normalize_rows(rows, target_id="target_0", frame_id="TBD",
                   source_mode="UNKNOWN", data_type="measured"):
    normalized = []
    for row in rows:
        unit = str(_first(row, ("unit", "position_unit"), "m")).strip().lower()
        if unit not in {"m", "cm"}:
            raise ValueError(f"unsupported position unit: {unit}")
        scale = 0.01 if unit == "cm" else 1.0
        sample_ns = _timestamp_ns(row)
        receive_ns = _timestamp_ns(row, receive=True) or sample_ns
        if sample_ns <= 0 or receive_ns < sample_ns:
            raise ValueError("each row requires positive sample time and non-earlier receive time")
        raw_validity = str(_first(row, ("validity", "valid", "status"), "VALID")).upper()
        validity = "VALID" if raw_validity in {"1", "TRUE", "VALID", "OK"} else "INVALID"
        reason = str(_first(row, ("invalid_reason", "reason"), ""))
        if validity == "INVALID" and not reason:
            reason = "source_marked_invalid"
        confidence_value = _first(row, ("confidence",), "")
        confidence = float(confidence_value) if confidence_value != "" else math.nan
        reprojection = _first(row, ("reprojection_error_px",), "")
        normalized.append({
            "sample_timestamp_ns": sample_ns,
            "receive_timestamp_ns": receive_ns,
            "target_id": str(_first(row, ("target_id",), target_id)),
            "frame_id": str(_first(row, ("frame_id", "frame", "field.header.frame_id"), frame_id)),
            "x_m": _position(row, "x") * scale,
            "y_m": _position(row, "y") * scale,
            "z_m": _position(row, "z") * scale,
            "source_mode": str(_first(row, ("source_mode", "source"), source_mode)),
            "validity": validity,
            "invalid_reason": reason,
            "unit": "m",
            "confidence": confidence,
            "reprojection_error_px": float(reprojection) if reprojection != "" else math.nan,
            "data_type": data_type,
        })
    return normalized


def read_csv_rows(path):
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def read_ros1_bag_rows(path, topic):
    try:
        process = subprocess.run(
            ["rostopic", "echo", "-b", str(path), "-p", topic],
            check=True, capture_output=True, text=True)
    except FileNotFoundError as error:
        raise RuntimeError(
            "ROS1 rostopic is unavailable; run this conversion on the Noetic host") from error
    except subprocess.CalledProcessError as error:
        raise RuntimeError(f"rostopic export failed: {error.stderr.strip()}") from error
    return list(csv.DictReader(process.stdout.splitlines()))


def read_ros2_bag_rows(path, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"topic not found in ROS2 bag: {topic}")
    message_type = get_message(topic_types[topic])
    rows = []
    while reader.has_next():
        current_topic, serialized, bag_timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_type)
        header = getattr(message, "header", None)
        sample_ns = bag_timestamp_ns
        frame_id = ""
        if header is not None:
            stamped = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
            sample_ns = stamped or bag_timestamp_ns
            frame_id = header.frame_id
        pose = getattr(message, "pose", None)
        if hasattr(pose, "pose"):
            pose = pose.pose
        if pose is None:
            raise ValueError(f"unsupported ROS2 message without pose field: {topic_types[topic]}")
        rows.append({
            "sample_timestamp_ns": sample_ns,
            "receive_timestamp_ns": getattr(message, "receive_timestamp_ns", bag_timestamp_ns),
            "target_id": getattr(message, "target_id", ""),
            "frame_id": frame_id,
            "x_m": pose.position.x,
            "y_m": pose.position.y,
            "z_m": pose.position.z,
            "source_mode": getattr(message, "source_mode", "UNKNOWN"),
            "validity": getattr(message, "validity", "VALID"),
            "invalid_reason": getattr(message, "invalid_reason", ""),
            "unit": getattr(message, "unit", "m"),
            "confidence": getattr(message, "confidence", math.nan),
        })
    return rows


def write_normalized_csv(path, rows):
    with Path(path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

