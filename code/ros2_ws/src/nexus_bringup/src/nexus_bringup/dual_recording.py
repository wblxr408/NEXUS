"""Explicit dual-localization rosbag inputs and non-overwriting provenance."""

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def recording_topics(image_topic="/camera/image_rect", camera_info_topic="/camera/camera_info",
                     preview_topic="/nexus/camera/imx219/image_raw/compressed"):
    return list(dict.fromkeys([
        image_topic, camera_info_topic, preview_topic, "/clock", "/tf", "/tf_static",
        "/nexus/fcu/imu", "/nexus/fcu/odom", "/nexus/uwb/ranges",
        "/nexus/uwb/startup_statistics", "/nexus/platform/initialization_status",
        "/nexus/platform/initialization_request", "/nexus/pi/fcu_clock_status",
        "/nexus/pi/odom_raw", "/nexus/pi/odom_aligned", "/nexus/pi/imu_raw", "/nexus/pi/ranges_raw", "/nexus/pi/imu_aligned", "/nexus/pi/ranges_aligned",
        "/nexus/platform/initial_pose", "/nexus/platform/odom", "/nexus/platform/status",
        "/apriltag/detections", "/nexus/vision/platform_pose", "/nexus/vision/reference_status",
        "/nexus/vision/body_motion", "/nexus/vision/motion_status", "/nexus/vision/detections", "/nexus/vision/target_tracks",
        "/nexus/vision/target_requests", "/nexus/vision/target_reference_image", "/nexus/vision/target_request_status",
        "/nexus/vision/target_kinematics", "/nexus/vision/target_geometry_status", "/nexus/observations/quality",
        "/nexus/target/kinematics", "/nexus/target/kinematic_fusion_status",
        "/nexus/optimization/compute_plan", "/nexus/optimization/observation_plan", "/nexus/optimization/vibration_quality",
        "/nexus/viz/localization_state", "/nexus/viz/localization_markers",
    ]))


def prepare_recording(output, run_id, input_mode, configs, topics, runtime_env=None):
    """Called only with explicit record=true. A claimed run directory is never reused."""
    if (not output or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", run_id or "")
            or run_id == "UNREGISTERED" or input_mode not in {"LIVE", "REPLAY", "SIMULATION"}):
        raise ValueError("recording requires a new output directory, explicit run_id and input mode")
    destination = Path(output).expanduser().resolve()
    assets = {name: (Path(path).resolve(), Path(path).read_bytes()) for name, path in configs.items()}
    destination.mkdir(parents=True, exist_ok=False)
    saved = destination / "configuration"
    saved.mkdir()
    provenance = {}
    for name, (source, payload) in assets.items():
        filename = name + source.suffix
        (saved / filename).write_bytes(payload)
        provenance[name] = {"source": str(source), "copy": "configuration/" + filename,
                            "sha256": hashlib.sha256(payload).hexdigest()}
    revision = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=False)
    dirty = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=False)
    runtime = {**os.environ, **(runtime_env or {})}
    metadata = {"schema_version": 1, "run_id": run_id, "input_mode": input_mode, "topics": topics,
                "git_revision": revision.stdout.strip() if revision.returncode == 0 else None,
                "working_tree_dirty": bool(dirty.stdout.strip()) if dirty.returncode == 0 else None,
                "runtime_environment": {key: runtime.get(key) for key in (
                    "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "FASTDDS_BUILTIN_TRANSPORTS")},
                "configuration": provenance, "target_model": "static", "control_commands": False,
                "target_covariance": "conditional_on_platform_and_mount",
                "timestamps": "message sample timestamps are preserved; bag timestamps are recording times"}
    (destination / "run.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    # Sensor-data QoS accepts both reliable and best-effort publishers. TF
    # static retains transient-local history when joining an existing graph.
    qos = {topic: {"reliability": "best_effort", "durability": "volatile", "history": "keep_last", "depth": 100}
           for topic in topics}
    qos["/tf_static"] = {"reliability": "reliable", "durability": "transient_local", "history": "keep_last", "depth": 100}
    (destination / "qos.json").write_text(json.dumps(qos, indent=2) + "\n", encoding="utf-8")
    return ["ros2", "bag", "record", "--storage", "sqlite3", "--output", str(destination / "bag"),
            "--qos-profile-overrides-path", str(destination / "qos.json"), *topics]
