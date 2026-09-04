#!/usr/bin/env python3
"""Read one JSON record from the Raspberry Pi read-only TCP stream."""

import argparse
import json
import socket


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.1.143")
    parser.add_argument("--port", type=int, default=14551)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    with socket.create_connection((args.host, args.port), args.timeout) as connection:
        connection.settimeout(args.timeout)
        stream = connection.makefile("rb")
        payload = json.loads(stream.readline().decode("utf-8"))
    print(json.dumps({"received": True, "schema_version": payload.get("schema_version"),
                      "tag_id": (payload.get("uwb") or {}).get("tag_id"),
                      "has_imu": bool(payload.get("imu")),
                      "ranges": (payload.get("uwb") or {}).get("anchor_ranges_m")},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
