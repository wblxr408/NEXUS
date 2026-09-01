#!/usr/bin/env python3
"""Conservative MAVLink autonomous-flight demo for FanciSwarm."""
from __future__ import annotations

import argparse
import math
import time

from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code" / "analysis"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation"))
from algorithms.uwb.live_protocol import convert_fc_position, parse_battery_distances

HEARTBEAT_MAX_AGE_S = 2.0
UWB_MAX_AGE_S = 1.0
FC_MAX_AGE_S = 1.0
POSITION_SPEED_LIMIT_MPS = 5.0
WAYPOINT_TOLERANCE_M = 0.2
SETPOINT_RATE_HZ = 5.0


class FlightSafetyError(RuntimeError):
    pass


class AutonomousFlight:
    def __init__(self, connection, *, dry_run=False, takeoff_only=False,
                 takeoff_height=1.0, hover_seconds=5.0, clock=None):
        self.connection = connection
        self.dry_run = bool(dry_run)
        self.takeoff_only = bool(takeoff_only)
        self.takeoff_height = float(takeoff_height)
        self.hover_seconds = float(hover_seconds)
        self.clock = clock or time.monotonic
        self.last_heartbeat = None
        self.last_uwb = None
        self.last_fc = None
        self.origin = None
        self.target_system = 1
        self.target_component = 1
        self.last_armed = None
        self.stopping = False

    def _now(self): return float(self.clock())

    def send_onboard_heartbeat(self):
        mav = self.connection.mav
        message = mav.heartbeat_encode(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)
        self._send(message, control=False)

    def _send(self, message, *, control=True):
        if self.dry_run and control:
            print("DRY-RUN", message.get_type() if hasattr(message, "get_type") else message)
            return
        self.connection.mav.send(message)

    def _handle(self, message):
        if message is None: return
        typ = message.get_type()
        now = self._now()
        if typ == "HEARTBEAT":
            self.last_heartbeat = now
            self.target_system = getattr(message, "get_srcSystem", lambda: self.target_system)() or self.target_system
            self.target_component = getattr(message, "get_srcComponent", lambda: self.target_component)() or self.target_component
            armed_flag = getattr(self.connection.mav, "MAV_MODE_FLAG_SAFETY_ARMED", 128)
            self.last_armed = (bool(message.base_mode & armed_flag)
                               if hasattr(message, "base_mode") else None)
        elif typ == "BATTERY_STATUS":
            try: parse_battery_distances(message.voltages); self.last_uwb = now
            except ValueError: pass
        elif typ == "GLOBAL_VISION_POSITION_ESTIMATE":
            try:
                position = convert_fc_position(message)
                if self.last_fc is not None:
                    dt = max(now - self.last_fc[1], 1e-3)
                    speed = math.dist(position, self.last_fc[0]) / dt
                    if not math.isfinite(speed) or speed > POSITION_SPEED_LIMIT_MPS:
                        raise FlightSafetyError(f"FC position jump speed {speed:.2f} m/s")
                self.last_fc = (position, now)
                if self.origin is None: self.origin = position
            except (TypeError, ValueError, OverflowError):
                pass

    def poll(self, timeout=0.2):
        message = self.connection.recv_match(blocking=True, timeout=timeout)
        self._handle(message)
        return message

    def ready(self):
        now = self._now()
        return (self.last_heartbeat is not None and now - self.last_heartbeat <= HEARTBEAT_MAX_AGE_S and
                self.last_uwb is not None and now - self.last_uwb <= UWB_MAX_AGE_S and
                self.last_fc is not None and now - self.last_fc[1] <= FC_MAX_AGE_S and self.origin is not None)

    def wait_ready(self, timeout=30.0):
        deadline = self._now() + timeout
        while self._now() < deadline:
            self.poll(0.2)
            if self.ready(): return
        raise FlightSafetyError("telemetry preconditions not satisfied before timeout")

    def command(self, command, params=(), timeout=3.0):
        mav = self.connection.mav
        values = list(params) + [0.0] * (7 - len(params))
        message = mav.command_long_encode(self.target_system, self.target_component, command, 0, *values[:7])
        self._send(message)
        if self.dry_run: return True
        deadline = self._now() + timeout
        while self._now() < deadline:
            received = self.poll(0.2)
            if received is not None and received.get_type() == "COMMAND_ACK" and received.command == command:
                if received.result not in (0, getattr(mav, "MAV_RESULT_ACCEPTED", 0)):
                    raise FlightSafetyError(f"command {command} rejected: {received.result}")
                return True
        raise FlightSafetyError(f"command {command} ACK timeout")

    def setpoint(self, x, y, z):
        mav = self.connection.mav
        mask = 0x0DF8
        message = mav.set_position_target_local_ned_encode(0, self.target_system, self.target_component,
            mav.MAV_FRAME_LOCAL_NED, mask, float(x), float(y), float(z), 0, 0, 0, 0, 0, 0, 0, 0)
        self._send(message)

    def arm(self):
        self.command(self.connection.mav.MAV_CMD_COMPONENT_ARM_DISARM, (1.0,))
        if self.dry_run: return
        deadline = self._now() + 3.0
        while self._now() < deadline:
            self.poll(0.2)
            if self.last_armed is True: return
            if self.last_armed is None: return
        raise FlightSafetyError("ARM ACK received but heartbeat is not armed")

    def takeoff(self):
        self.command(self.connection.mav.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, 0, 0, 0, self.takeoff_height))
        if self.dry_run: return
        deadline = self._now() + 15.0
        while self._now() < deadline:
            self.poll(0.2)
            if self.last_fc is not None and self.origin is not None:
                altitude = self.last_fc[0][2] - self.origin[2]
                if altitude >= 0.8 * self.takeoff_height: return
        raise FlightSafetyError("TAKEOFF altitude telemetry timeout")

    def land(self):
        try: self.command(self.connection.mav.MAV_CMD_NAV_LAND, timeout=2.0)
        except Exception as exc: print(f"LAND failed: {exc}")

    def hover(self, seconds=None):
        if self.dry_run:
            print(f"DRY-RUN HOVER {self.hover_seconds if seconds is None else seconds:.1f}s")
            return
        deadline = self._now() + (self.hover_seconds if seconds is None else seconds)
        while self._now() < deadline:
            self.poll(0.1)

    def fly_waypoint(self, x, y, z= -1.0):
        if self.dry_run:
            self.setpoint(x, y, z)
            return
        deadline = self._now() + max(self.hover_seconds, 1.0)
        while self._now() < deadline:
            self.setpoint(x, y, z)
            self.poll(1.0 / SETPOINT_RATE_HZ)
            if self.last_fc is not None and self.origin is not None:
                dx = self.last_fc[0][0] - self.origin[0] - x
                dy = self.last_fc[0][1] - self.origin[1] - y
                if math.hypot(dx, dy) <= WAYPOINT_TOLERANCE_M: return

    def stop_safely(self):
        if self.stopping: return
        self.stopping = True
        self.land()

    def run(self):
        self.send_onboard_heartbeat()
        try:
            self.wait_ready()
            self.arm(); self.takeoff(); self.hover()
            if not self.takeoff_only:
                self.fly_waypoint(0.5, 0.0); self.hover()
                self.fly_waypoint(0.5, 0.5); self.hover()
                self.fly_waypoint(0.0, 0.0); self.hover()
            self.land()
        except KeyboardInterrupt:
            self.stop_safely(); raise
        except Exception:
            self.stop_safely(); raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udp:192.168.1.143:14550")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--takeoff-only", action="store_true")
    parser.add_argument("--takeoff-height", type=float, default=1.0)
    parser.add_argument("--hover-seconds", type=float, default=5.0)
    args = parser.parse_args(argv)
    try:
        from pymavlink import mavutil
        connection = mavutil.mavlink_connection(args.connection)
        flight = AutonomousFlight(connection, dry_run=args.dry_run, takeoff_only=args.takeoff_only,
                                   takeoff_height=args.takeoff_height, hover_seconds=args.hover_seconds)
        flight.run()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
