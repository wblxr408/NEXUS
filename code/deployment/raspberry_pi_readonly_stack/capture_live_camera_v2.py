#!/usr/bin/env python3
"""Add-only IMX219 capture with an explicit Unix receive timestamp."""

from __future__ import annotations

import argparse
import json
import os
import signal
import time
from pathlib import Path

from imx219_frame_source import Imx219FrameSource


STOP = False


def request_stop(*_):
    global STOP
    STOP = True


def write_frame(output_dir, frame, rotation_deg, captured_unix_ns=None, frame_sequence=None):
    from PIL import Image

    captured_unix_ns = time.time_ns() if captured_unix_ns is None else int(captured_unix_ns)
    image_tmp = output_dir / ".latest.jpg.tmp"
    meta_tmp = output_dir / ".latest.json.tmp"
    Image.fromarray(frame.rgb, mode="RGB").save(image_tmp, format="JPEG", quality=95)
    metadata = {
        "schema_version": 2,
        "image": "latest.jpg",
        "pixel_format": "RGB888",
        "width": int(frame.rgb.shape[1]),
        "height": int(frame.rgb.shape[0]),
        "rotation_deg": int(rotation_deg),
        "captured_unix_ns": captured_unix_ns,
        "sample_timestamp_domain": "ros_unix_ns_receive",
        "captured_monotonic_ns": frame.captured_monotonic_ns,
        "sensor_timestamp_ns": frame.sensor_timestamp_ns,
        "exposure_time_us": frame.exposure_time_us,
        "analogue_gain": frame.analogue_gain,
    }
    if frame_sequence is not None:
        metadata["frame_sequence"] = int(frame_sequence)
    meta_tmp.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    os.replace(image_tmp, output_dir / "latest.jpg")
    os.replace(meta_tmp, output_dir / "latest.json")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="outputs/live_camera_v2")
    parser.add_argument("--width", type=int, default=1640)
    parser.add_argument("--height", type=int, default=1232)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--rotation", type=int, choices=(0, 180), default=180)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--duration-seconds", type=float, default=0.0)
    args = parser.parse_args()
    if args.frames < 0 or args.duration_seconds < 0:
        raise SystemExit("frame and duration limits must be non-negative")
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    started, count = time.monotonic(), 0
    with Imx219FrameSource(width=args.width, height=args.height, fps=args.fps,
                           rotation_deg=args.rotation) as camera:
        while not STOP:
            frame = camera.capture()
            write_frame(output_dir, frame, args.rotation, frame_sequence=count)
            count += 1
            if args.frames and count >= args.frames:
                break
            if args.duration_seconds and time.monotonic() - started >= args.duration_seconds:
                break
    print(f"CAMERA_V2_OK frames={count} output={output_dir}")


if __name__ == "__main__":
    main()
