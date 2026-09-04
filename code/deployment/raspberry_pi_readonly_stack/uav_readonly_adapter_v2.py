#!/usr/bin/env python3
"""Add-only tag-2 adapter with explicit UWB receive timestamps.

This module extends, but never modifies, ``uav_readonly_adapter.py``.  The
inherited transport sends only the passive GCS heartbeat required by the
vendor endpoint; it exposes no flight-control API.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import time
from pathlib import Path

from uav_readonly_adapter import (
    AdapterConfig,
    JsonPublisher,
    ReadOnlyMavlinkAdapter,
    StateProjector,
)


class StateProjectorV2(StateProjector):
    def __init__(self, config: AdapterConfig, anchor_ids=("1", "2", "3", "4")):
        super().__init__(config)
        self.state["schema_version"] = "2.0"
        self.state["uwb"].update(
            anchor_ids=list(anchor_ids),
            sample_timestamp_ns=None,
            sample_timestamp_domain=None,
        )

    def process(self, message, *, now_monotonic=None, now_unix_ns=None):
        super().process(message, now_monotonic=now_monotonic)
        if self._message_type(message) == "BATTERY_STATUS":
            voltages = list(self._get(message, "voltages", []) or [])
            if len(voltages) >= 6:
                self.state["uwb"]["sample_timestamp_ns"] = int(
                    time.time_ns() if now_unix_ns is None else now_unix_ns
                )
                self.state["uwb"]["sample_timestamp_domain"] = "ros_unix_ns_receive"
            else:
                self.state["uwb"]["anchor_ranges_m"] = [None, None, None, None]
                self.state["uwb"]["sample_timestamp_ns"] = None
                self.state["uwb"]["sample_timestamp_domain"] = None

    def snapshot(self, **kwargs):
        result = super().snapshot(**kwargs)
        if not any(value is not None for value in result["uwb"]["anchor_ranges_m"]):
            result["uwb"]["sample_timestamp_ns"] = None
            result["uwb"]["sample_timestamp_domain"] = None
        return result


class ReadOnlyMavlinkAdapterV2(ReadOnlyMavlinkAdapter):
    def __init__(self, config: AdapterConfig, publisher: JsonPublisher):
        super().__init__(config, publisher)
        self.projector = StateProjectorV2(config)

    def _run_connection(self, connection, mavutil):
        """Forward every IMU sample; publish other accumulated state periodically."""
        next_heartbeat = next_status = 0.0
        last_rx = time.monotonic()
        while self.running:
            now = time.monotonic()
            if now >= next_heartbeat:
                self._send_readonly_gcs_heartbeat(connection, mavutil)
                next_heartbeat = now + self.config.heartbeat_interval_s
            message = connection.recv_match(blocking=True, timeout=0.05)
            if message is not None:
                self.projector.process(message)
                last_rx = time.monotonic()
                if self.projector._message_type(message) == "SCALED_IMU":
                    self.publisher.publish(self.projector.snapshot(now_monotonic=last_rx))
            now = time.monotonic()
            if now - last_rx > self.config.reconnect_on_silence_s:
                raise ConnectionError(
                    f"连续 {self.config.reconnect_on_silence_s:.1f} 秒未收到 MAVLink 数据"
                )
            if now >= next_status:
                self.publisher.publish(self.projector.snapshot(now_monotonic=now))
                next_status = now + self.config.publish_interval_s


class TcpJsonPublisher(JsonPublisher):
    """Append JSONL and expose the same records on a read-only TCP stream."""

    def __init__(self, output_path=None, *, listen_host="0.0.0.0", listen_port=14551):
        super().__init__(output_path)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((listen_host, int(listen_port)))
        self.server.listen(4)
        self.server.setblocking(False)
        self.clients = []

    def publish(self, state):
        super().publish(state)
        try:
            while True:
                client, _ = self.server.accept()
                client.setblocking(True)
                self.clients.append(client)
        except BlockingIOError:
            pass
        payload = (json.dumps(state, ensure_ascii=False, separators=(",", ":")) + "\n").encode()
        alive = []
        for client in self.clients:
            try:
                client.sendall(payload)
                alive.append(client)
            except (TimeoutError, BlockingIOError, BrokenPipeError, ConnectionResetError, OSError) as exc:
                logging.getLogger("uav-readonly-adapter-v2").warning(
                    "dropping telemetry TCP client after send failure: %s", exc)
                client.close()
        self.clients = alive

    def close(self):
        for client in self.clients:
            client.close()
        self.server.close()
        super().close()


def parse_v2_args(argv=None):
    parser = argparse.ArgumentParser(description="Mcontroller tag-2 read-only adapter v2")
    parser.add_argument("--host", default="192.168.1.126")
    parser.add_argument("--port", type=int, default=333)
    parser.add_argument("--agent-id", default="drone_001")
    parser.add_argument("--uwb-tag-id", type=int, choices=(2,), default=2)
    parser.add_argument("--frame-id", default="uwb_raw")
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--tcp-listen-host", default="0.0.0.0")
    parser.add_argument("--tcp-listen-port", type=int, default=14551)
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    args = parser.parse_args(argv)
    if args.publish_rate <= 0:
        parser.error("--publish-rate must be positive")
    return args


def main(argv=None):
    args = parse_v2_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    config = AdapterConfig(
        host=args.host,
        port=args.port,
        agent_id=args.agent_id,
        uwb_tag_id=2,
        frame_id=args.frame_id,
        publish_interval_s=1.0 / args.publish_rate,
    )
    publisher = TcpJsonPublisher(args.output, listen_host=args.tcp_listen_host,
                                 listen_port=args.tcp_listen_port)
    adapter = ReadOnlyMavlinkAdapterV2(config, publisher)
    signal.signal(signal.SIGINT, adapter.stop)
    signal.signal(signal.SIGTERM, adapter.stop)
    try:
        return adapter.run()
    finally:
        publisher.close()


if __name__ == "__main__":
    raise SystemExit(main())
