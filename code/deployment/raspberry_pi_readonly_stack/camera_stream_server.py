#!/usr/bin/env python3
"""Serve bottom-facing IMX219 frames over an add-only read-only TCP stream."""

import argparse
import io
import json
import socket
import struct
import time

from PIL import Image

from imx219_frame_source import Imx219FrameSource
from camera_clock_server import CameraClockServer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=14552)
    parser.add_argument("--width", type=int, default=1640)
    parser.add_argument("--height", type=int, default=1232)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--rotation", type=int, choices=(0, 180), default=180)
    parser.add_argument("--clock-port", type=int, default=14553)
    parser.add_argument("--frames", type=int, default=0)
    args = parser.parse_args()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen_host, args.listen_port))
    server.listen(2)
    server.setblocking(False)
    clock = CameraClockServer(args.listen_host, args.clock_port)
    clock.start()
    clients = []
    count = 0
    try:
        with Imx219FrameSource(width=args.width, height=args.height, fps=args.fps,
                               rotation_deg=args.rotation) as camera:
            while not args.frames or count < args.frames:
                try:
                    while True:
                        client, _ = server.accept()
                        # Checkerboard detection on the edge host can briefly take
                        # longer than one camera period.  Keep the read-only stream
                        # alive rather than dropping a valid, slower consumer.
                        client.settimeout(2.0)
                        clients.append(client)
                except BlockingIOError:
                    pass
                frame = camera.capture()
                captured_unix_ns = time.time_ns()
                encoded = io.BytesIO()
                Image.fromarray(frame.rgb, mode="RGB").save(encoded, format="JPEG", quality=90)
                image = encoded.getvalue()
                metadata = json.dumps({
                    "schema_version": 1, "captured_unix_ns": captured_unix_ns,
                    "sample_timestamp_domain": "ros_unix_ns_receive",
                    "sensor_timestamp_ns": frame.sensor_timestamp_ns,
                    "sensor_timestamp_domain": "CLOCK_BOOTTIME",
                    "boot_id": clock.boot_id,
                    "exposure_time_us": frame.exposure_time_us,
                    "frame_sequence": count,
                    "width": int(frame.rgb.shape[1]), "height": int(frame.rgb.shape[0]),
                    "encoding": "rgb8", "rotation_deg": args.rotation,
                }, separators=(",", ":")).encode()
                packet = struct.pack("!II", len(metadata), len(image)) + metadata + image
                alive = []
                for client in clients:
                    try:
                        client.sendall(packet)
                        alive.append(client)
                    except OSError:
                        client.close()
                clients = alive
                count += 1
    finally:
        for client in clients:
            client.close()
        server.close()
        clock.close()
    print(f"CAMERA_STREAM_OK frames={count}")


if __name__ == "__main__":
    main()
