#!/usr/bin/env python3
"""Synthetic tag-2 telemetry server for cross-device transport tests only."""

import argparse
import json
import socket
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=14551)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen_host, args.listen_port))
    server.listen(1)
    connection, address = server.accept()
    started = time.monotonic()
    index = 0
    uwb_stamp = time.time_ns()
    try:
        while time.monotonic() - started < args.duration:
            if index % 200 == 0:
                uwb_stamp = time.time_ns()
            state = {
                "schema_version": "2.0",
                "online": True,
                "source": {"simulated": True, "purpose": "cross_device_transport_test"},
                "imu": {
                    "sample_timestamp_us": 1_000_000 + index * 5_000,
                    "sample_timestamp_domain": "flight_boot_unverified",
                    "acceleration_mps2": {"x": 0.1, "y": 0.2, "z": -9.8},
                    "angular_velocity_rps": {"x": 0.01, "y": 0.02, "z": 0.03},
                },
                "uwb": {
                    "tag_id": 2,
                    "anchor_ids": ["1", "2", "3", "4"],
                    "anchor_ranges_m": [5.21, 2.35, 3.91, 5.55],
                    "sample_timestamp_ns": uwb_stamp,
                    "sample_timestamp_domain": "ros_unix_ns_receive",
                },
            }
            connection.sendall((json.dumps(state, separators=(",", ":")) + "\n").encode())
            index += 1
            time.sleep(0.005)
    except (BrokenPipeError, ConnectionResetError):
        pass
    finally:
        connection.close()
        server.close()
    print(f"SYNTHETIC_TELEMETRY_TEST_OK peer={address[0]} samples={index}")


if __name__ == "__main__":
    main()
