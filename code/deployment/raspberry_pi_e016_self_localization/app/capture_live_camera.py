#!/usr/bin/env python3
"""Publish the latest IMX219 RGB frame for the real GDR-Net input boundary.

Only ``latest.jpg`` and ``latest.json`` are replaced by default, so a long
run does not silently accumulate raw camera data.  Both files are atomically
replaced, allowing an inference process to read a complete matching pair.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import time
from pathlib import Path

from imx219_frame_source import CameraFrame, Imx219FrameSource


STOP_REQUESTED = False


def _request_stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def _write_latest(output_dir: Path, frame: CameraFrame, *, rotation_deg: int) -> None:
    image_path = output_dir / "latest.jpg"
    image_temporary = output_dir / ".latest.jpg.tmp"
    metadata_path = output_dir / "latest.json"
    metadata_temporary = output_dir / ".latest.json.tmp"

    # Pillow is available with Picamera2 on Raspberry Pi OS.  Keep the input
    # contract RGB explicit instead of relying on OpenCV's BGR convention.
    from PIL import Image

    Image.fromarray(frame.rgb, mode="RGB").save(image_temporary, format="JPEG", quality=95)
    os.replace(image_temporary, image_path)
    metadata = {
        "image": image_path.name,
        "pixel_format": "RGB888",
        "width": int(frame.rgb.shape[1]),
        "height": int(frame.rgb.shape[0]),
        "rotation_deg": rotation_deg,
        "captured_monotonic_ns": frame.captured_monotonic_ns,
        "sensor_timestamp_ns": frame.sensor_timestamp_ns,
        "exposure_time_us": frame.exposure_time_us,
        "analogue_gain": frame.analogue_gain,
        "gdr_net_contract": "Read latest.jpg as RGB; provide calibrated K/distortion separately.",
    }
    metadata_temporary.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    os.replace(metadata_temporary, metadata_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="outputs/live_camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--rotation", choices=(0, 180), type=int, default=180)
    parser.add_argument("--frames", type=int, default=0, help="0 means run until interrupted")
    parser.add_argument("--duration-seconds", type=float, default=0.0, help="0 means no time limit")
    args = parser.parse_args()
    if args.frames < 0 or args.duration_seconds < 0:
        raise SystemExit("--frames and --duration-seconds must be non-negative")
    if args.frames == 0 and args.duration_seconds == 0:
        print("Capturing until Ctrl-C. Use --frames or --duration-seconds for a bounded run.")

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    started = time.monotonic()
    captured = 0
    with Imx219FrameSource(
        width=args.width,
        height=args.height,
        fps=args.fps,
        rotation_deg=args.rotation,
    ) as camera:
        while not STOP_REQUESTED:
            frame = camera.capture()
            _write_latest(output_dir, frame, rotation_deg=args.rotation)
            captured += 1
            print(
                f"CAMERA_FRAME_OK index={captured} "
                f"sensor_timestamp_ns={frame.sensor_timestamp_ns} path={output_dir / 'latest.jpg'}",
                flush=True,
            )
            if args.frames and captured >= args.frames:
                break
            if args.duration_seconds and time.monotonic() - started >= args.duration_seconds:
                break

    print(f"LIVE_CAMERA_CAPTURE_OK frames={captured} output={output_dir}")


if __name__ == "__main__":
    main()
