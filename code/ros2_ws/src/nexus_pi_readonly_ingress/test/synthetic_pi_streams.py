#!/usr/bin/env python3
"""Serve deterministic telemetry/camera packets with the Pi v2 wire format."""

import argparse
import io
import json
import socket
import struct
import threading
import time

from PIL import Image


def listener(port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(1)
    connection, _ = server.accept()
    return server, connection


def telemetry(duration):
    server, connection = listener(14551)
    started = time.monotonic()
    index = 0
    uwb_stamp = time.time_ns()
    try:
        while time.monotonic() - started < duration:
            if index % 200 == 0:
                uwb_stamp = time.time_ns()
            state = {
                "schema_version": "2.0",
                "imu": {
                    "sample_timestamp_us": 1_000_000 + index * 5_000,
                    "sample_timestamp_domain": "flight_boot_unverified",
                    "acceleration_mps2": {"x": 0.1, "y": 0.2, "z": -9.8},
                    "angular_velocity_rps": {"x": 0.01, "y": 0.02, "z": 0.03},
                },
                "uwb": {
                    "tag_id": 2,
                    "anchor_ranges_m": [5.21, 2.35, 3.91, 5.55],
                    "sample_timestamp_ns": uwb_stamp,
                    "sample_timestamp_domain": "ros_unix_ns_receive",
                },
            }
            connection.sendall((json.dumps(state, separators=(",", ":")) + "\n").encode())
            index += 1
            time.sleep(0.005)
    finally:
        connection.close()
        server.close()


def camera(duration):
    server, connection = listener(14552)
    image_file = io.BytesIO()
    Image.new("RGB", (1640, 1232), (20, 40, 60)).save(image_file, "JPEG", quality=90)
    image = image_file.getvalue()
    started = time.monotonic()
    index = 0
    try:
        while time.monotonic() - started < duration:
            now = time.time_ns()
            metadata = json.dumps({
                "schema_version": 1,
                "captured_unix_ns": now,
                "sample_timestamp_domain": "ros_unix_ns_receive",
                "sensor_timestamp_ns": 1_000_000_000 + index * 200_000_000,
                "frame_sequence": index,
                "width": 1640,
                "height": 1232,
                "encoding": "rgb8",
                "rotation_deg": 180,
            }, separators=(",", ":")).encode()
            connection.sendall(struct.pack("!II", len(metadata), len(image)) + metadata + image)
            index += 1
            time.sleep(0.2)
    finally:
        connection.close()
        server.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15.0)
    args = parser.parse_args()
    threads = [threading.Thread(target=telemetry, args=(args.duration,)),
               threading.Thread(target=camera, args=(args.duration,))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
