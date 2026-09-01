#!/usr/bin/env python3
"""Transparent Raspberry Pi MAVLink/UART <-> UDP bridge for FanciSwarm."""
from __future__ import annotations

import argparse
import select
import socket
import time


class TransparentBridge:
    def __init__(self, serial_port, udp_socket, udp_target, *, mavlink=None, heartbeat_period=1.0):
        self.serial = serial_port
        self.socket = udp_socket
        self.udp_target = tuple(udp_target)
        self.mavlink = mavlink
        self.heartbeat_period = float(heartbeat_period)
        self.flight_controller_ids = None
        self.control_endpoints = set()
        self.last_heartbeat = 0.0
        self._parser = mavlink.MAVLink(None) if mavlink is not None else None
        if self._parser is not None:
            self._parser.force_mavlink1 = True

    def _heartbeat_packet(self):
        if self.mavlink is None:
            return b""
        message = self.mavlink.MAVLink_heartbeat_message(
            self.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, self.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, self.mavlink.MAV_STATE_ACTIVE)
        message._header.srcSystem = 255
        message._header.srcComponent = 190
        return message.pack(self._parser)

    def _parse_flight_ids(self, data):
        if self._parser is None:
            return
        for byte in data:
            try:
                message = self._parser.parse_char(bytes((byte,)))
            except Exception:
                continue
            if message is not None and message.get_type() == "HEARTBEAT":
                self.flight_controller_ids = (message.get_srcSystem(), message.get_srcComponent())

    def handle_uart_bytes(self, data):
        if not data:
            return
        self._parse_flight_ids(data)
        endpoints = {self.udp_target} | self.control_endpoints
        for endpoint in endpoints:
            self.socket.sendto(data, endpoint)

    def handle_udp_datagram(self, data, endpoint):
        if endpoint:
            self.control_endpoints.add(tuple(endpoint))
        if data:
            self.serial.write(data)

    def send_onboard_heartbeat(self, now=None):
        now = time.monotonic() if now is None else float(now)
        if self.flight_controller_ids is None:
            return False
        if now - self.last_heartbeat < self.heartbeat_period:
            return False
        packet = self._heartbeat_packet()
        if packet:
            self.serial.write(packet)
        self.last_heartbeat = now
        return True

    def run(self):
        self.socket.setblocking(False)
        while True:
            self.send_onboard_heartbeat()
            readable, _, _ = select.select([self.serial, self.socket], [], [], 0.1)
            for source in readable:
                if source is self.serial:
                    self.handle_uart_bytes(self.serial.read(4096))
                else:
                    data, endpoint = self.socket.recvfrom(65535)
                    self.handle_udp_datagram(data, endpoint)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--udp-host", default="192.168.1.140")
    parser.add_argument("--udp-port", type=int, default=14550)
    args = parser.parse_args(argv)
    try:
        import serial
        from pymavlink.dialects.v10 import common as mavlink
        serial_port = serial.Serial(args.serial, args.baud, timeout=0)
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        bridge = TransparentBridge(serial_port, udp_socket, (args.udp_host, args.udp_port), mavlink=mavlink)
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        for resource in (locals().get("serial_port"), locals().get("udp_socket")):
            if resource is not None:
                resource.close()


if __name__ == "__main__":
    main()
