#!/usr/bin/env python3
"""Check freshness and required fields in a Pi read-only capture."""

import argparse
import json
import time
from pathlib import Path


def load_last_json_line(path):
    with path.open(encoding="utf-8") as stream:
        lines = [line for line in stream if line.strip()]
    if not lines:
        raise ValueError("telemetry file is empty")
    return json.loads(lines[-1]), len(lines)


def evaluate(telemetry_path, camera_metadata_path, now_ns=None, max_age_s=3.0):
    now_ns = time.time_ns() if now_ns is None else int(now_ns)
    report = {"schema_version": 1, "ok": True, "checks": {}}
    if telemetry_path:
        state, count = load_last_json_line(Path(telemetry_path))
        age = max(0.0, now_ns / 1e9 - float(state.get("timestamp", 0.0)))
        ranges = (state.get("uwb") or {}).get("anchor_ranges_m") or []
        report["checks"]["telemetry"] = {
            "records": count, "age_s": age, "online": state.get("online") is True,
            "imu_available": all(v is not None for group in (
                (state.get("imu") or {}).get("acceleration_mps2", {}),
                (state.get("imu") or {}).get("angular_velocity_rps", {}),
            ) for v in group.values()),
            "valid_uwb_ranges": sum(value is not None for value in ranges),
        }
        report["ok"] &= age <= max_age_s and state.get("online") is True
    if camera_metadata_path:
        metadata = json.loads(Path(camera_metadata_path).read_text(encoding="utf-8"))
        age = max(0.0, (now_ns - int(metadata["captured_unix_ns"])) / 1e9)
        report["checks"]["camera"] = {
            "age_s": age, "width": metadata["width"], "height": metadata["height"],
            "sensor_timestamp_present": metadata.get("sensor_timestamp_ns") is not None,
        }
        report["ok"] &= age <= max_age_s
    report["ok"] = bool(report["ok"])
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry-jsonl")
    parser.add_argument("--camera-metadata")
    parser.add_argument("--max-age-seconds", type=float, default=3.0)
    args = parser.parse_args()
    if not args.telemetry_jsonl and not args.camera_metadata:
        raise SystemExit("provide telemetry and/or camera input")
    report = evaluate(args.telemetry_jsonl, args.camera_metadata, max_age_s=args.max_age_seconds)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
