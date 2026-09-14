#!/usr/bin/env python3
"""Wait for an online, armed state from the local read-only telemetry log."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any


def is_armed_online(state: Any) -> bool:
    return isinstance(state, dict) and state.get("online") is True and state.get("armed") is True


def wait_for_armed(telemetry_path: Path, timeout_seconds: float, poll_seconds: float = 0.1) -> dict:
    deadline = time.monotonic() + timeout_seconds
    offset = 0
    while time.monotonic() < deadline:
        try:
            with telemetry_path.open("r", encoding="utf-8") as stream:
                stream.seek(offset)
                lines = stream.readlines()
                offset = stream.tell()
        except FileNotFoundError:
            lines = []
        for line in lines:
            try:
                state = json.loads(line)
            except json.JSONDecodeError:
                continue
            if is_armed_online(state):
                return state
        time.sleep(poll_seconds)
    raise TimeoutError(f"timed out after {timeout_seconds:g}s waiting for online armed telemetry")


def main() -> int:
    parser = argparse.ArgumentParser(description="Wait for online armed telemetry before camera capture")
    parser.add_argument("--telemetry-jsonl", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    parser.add_argument("--event-output", type=Path)
    args = parser.parse_args()
    if args.timeout_seconds <= 0:
        parser.error("--timeout-seconds must be positive")
    try:
        state = wait_for_armed(args.telemetry_jsonl, args.timeout_seconds)
    except TimeoutError as exc:
        print(f"ARM_TRIGGER_TIMEOUT {exc}")
        return 3
    event = {
        "schema_version": 1,
        "event": "camera_capture_armed_trigger",
        "triggered_unix_ns": time.time_ns(),
        "trigger": {"online": True, "armed": True},
        "telemetry_timestamp": state.get("timestamp"),
        "telemetry_timestamp_iso": state.get("timestamp_iso"),
    }
    if args.event_output is not None:
        args.event_output.parent.mkdir(parents=True, exist_ok=True)
        args.event_output.write_text(json.dumps(event, indent=2) + "\n", encoding="utf-8")
    print(f"ARM_TRIGGER_OK telemetry={args.telemetry_jsonl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

