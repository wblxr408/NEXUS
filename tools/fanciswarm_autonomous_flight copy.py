#!/usr/bin/env python3
"""Conservative MAVLink autonomous-flight demo for FanciSwarm."""
from __future__ import annotations

import argparse
import csv
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
DEFAULT_MAX_SPEED_MPS = 0.2
# Keep this materially below the smallest teaching-route leg so a point is
# not accepted while the aircraft is still visibly between taught points.
WAYPOINT_TOLERANCE_M = 0.05
SETPOINT_RATE_HZ = 5.0


class VirtualFence:
    """Axis-aligned local-NED flight volume around the telemetry origin."""

    def __init__(self, min_x_m=-0.6, max_x_m=0.6, min_y_m=-0.6, max_y_m=0.6,
                 min_z_m=-1.5, max_z_m=0.5):
        self.min_x_m = float(min_x_m)
        self.max_x_m = float(max_x_m)
        self.min_y_m = float(min_y_m)
        self.max_y_m = float(max_y_m)
        self.min_z_m = float(min_z_m)
        self.max_z_m = float(max_z_m)
        values = (self.min_x_m, self.max_x_m, self.min_y_m, self.max_y_m,
                  self.min_z_m, self.max_z_m)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("virtual fence bounds must be finite")
        if self.min_x_m >= self.max_x_m or self.min_y_m >= self.max_y_m or self.min_z_m >= self.max_z_m:
            raise ValueError("virtual fence minimum must be less than maximum")

    def contains(self, point):
        x, y, z = (float(value) for value in point)
        return (self.min_x_m <= x <= self.max_x_m and
                self.min_y_m <= y <= self.max_y_m and
                self.min_z_m <= z <= self.max_z_m)


def load_waypoints(path):
    """Load local-NED teaching points from a CSV with x_m,y_m,z_m columns."""
    required = {"x_m", "y_m", "z_m"}
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError("waypoint CSV missing columns: " + ", ".join(sorted(missing)))
        points = []
        for line_number, row in enumerate(reader, start=2):
            try:
                point = tuple(float(row[name]) for name in ("x_m", "y_m", "z_m"))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid waypoint at CSV line {line_number}") from exc
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f"waypoint must be finite at CSV line {line_number}")
            points.append(point)
    if not points:
        raise ValueError("waypoint CSV must contain at least one point")
    return tuple(points)


class FlightSafetyError(RuntimeError):
    pass


class AutonomousFlight:
    def __init__(self, connection, *, dry_run=False, takeoff_only=False,
                 takeoff_height=1.0, hover_seconds=5.0,
                 waypoint_distance=0.5, waypoints=None, fence=None,
                 max_speed_mps=DEFAULT_MAX_SPEED_MPS, clock=None):
        self.connection = connection
        # MAVLink message objects expose encode/send methods; protocol enum
        # constants live on the dialect module instead.
        try:
            from pymavlink import mavutil
            self.protocol = mavutil.mavlink
        except ImportError:
            self.protocol = getattr(connection, "mavlink", connection.mav)
        self.dry_run = bool(dry_run)
        self.takeoff_only = bool(takeoff_only)
        self.takeoff_height = float(takeoff_height)
        self.hover_seconds = float(hover_seconds)
        self.waypoint_distance = float(waypoint_distance)
        if not math.isfinite(self.takeoff_height) or self.takeoff_height <= 0.0:
            raise ValueError("takeoff_height must be positive")
        if not math.isfinite(self.waypoint_distance) or self.waypoint_distance <= 0.0:
            raise ValueError("waypoint_distance must be positive")
        self.max_speed_mps = float(max_speed_mps)
        if not math.isfinite(self.max_speed_mps) or self.max_speed_mps <= 0.0:
            raise ValueError("max_speed_mps must be positive")
        if self.max_speed_mps > DEFAULT_MAX_SPEED_MPS:
            raise ValueError(f"max_speed_mps cannot exceed {DEFAULT_MAX_SPEED_MPS:.1f} m/s")
        self.fence = fence or VirtualFence()
        self.waypoints = tuple(waypoints or ())
        for point in self.waypoints:
            if not self.fence.contains(point):
                raise ValueError(f"waypoint {point} is outside the virtual fence")
        takeoff_z = -self.takeoff_height  # local-NED: upward is negative
        if not self.fence.contains((0.0, 0.0, takeoff_z)):
            raise ValueError("takeoff height is outside the virtual fence")
        self.clock = clock or time.monotonic
        self.last_heartbeat = None
        self.last_uwb = None
        self.last_fc = None
        self.origin = None
        self.target_system = 1
        self.target_component = 1
        self.last_base_mode = None
        self.last_armed = None
        self.last_tx_heartbeat = -math.inf
        self.stopping = False

    def _now(self): return float(self.clock())

    def send_onboard_heartbeat(self, now=None):
        mav = self.connection.mav
        message = mav.heartbeat_encode(
            self.protocol.MAV_TYPE_ONBOARD_CONTROLLER,
            self.protocol.MAV_AUTOPILOT_INVALID, 0, 0,
            self.protocol.MAV_STATE_ACTIVE,
        )
        self._send(message, control=False)
        self.last_tx_heartbeat = self._now() if now is None else float(now)

    def maybe_send_onboard_heartbeat(self):
        now = self._now()
        if now - self.last_tx_heartbeat >= 1.0:
            self.send_onboard_heartbeat(now)

    def _send(self, message, *, control=True):
        if self.dry_run and control:
            print("DRY-RUN", message.get_type() if hasattr(message, "get_type") else message)
            return
        self.connection.mav.send(message, force_mavlink1=True)

    def _handle(self, message):
        if message is None: return
        typ = message.get_type()
        now = self._now()
        if typ == "HEARTBEAT":
            # Ignore only the bridge's own known heartbeat.  Do not reject an
            # FC heartbeat solely because its dialect reports autopilot=INVALID.
            invalid_autopilot = getattr(self.protocol, "MAV_AUTOPILOT_INVALID", 8)
            source_system = getattr(message, "get_srcSystem", lambda: None)()
            source_component = getattr(message, "get_srcComponent", lambda: None)()
            if ((source_system, source_component) == (254, 190) and
                    getattr(message, "autopilot", invalid_autopilot) == invalid_autopilot):
                return
            self.last_heartbeat = now
            self.target_system = getattr(message, "get_srcSystem", lambda: self.target_system)() or self.target_system
            self.target_component = getattr(message, "get_srcComponent", lambda: self.target_component)() or self.target_component
            self.last_base_mode = getattr(message, "base_mode", None)
            armed_flag = getattr(self.protocol, "MAV_MODE_FLAG_SAFETY_ARMED", 128)
            self.last_armed = (bool(self.last_base_mode & armed_flag)
                               if self.last_base_mode is not None else None)
        elif typ == "BATTERY_STATUS":
            try: parse_battery_distances(message.voltages); self.last_uwb = now
            except ValueError: pass
        elif typ == "GLOBAL_VISION_POSITION_ESTIMATE":
            try:
                position = convert_fc_position(message)
                if self.last_fc is not None:
                    dt = max(now - self.last_fc[1], 1e-3)
                    speed = math.dist(position, self.last_fc[0]) / dt
                    if not math.isfinite(speed) or speed > self.max_speed_mps:
                        raise FlightSafetyError(
                            f"FC position speed {speed:.2f} m/s exceeds limit {self.max_speed_mps:.2f} m/s")
                self.last_fc = (position, now)
                if self.origin is None: self.origin = position
                else:
                    relative = tuple(position[index] - self.origin[index] for index in range(3))
                    if not self.fence.contains(relative):
                        raise FlightSafetyError(
                            f"FC position {relative} is outside virtual fence")
            except (TypeError, ValueError, OverflowError):
                pass

    def poll(self, timeout=0.2):
        self.maybe_send_onboard_heartbeat()
        message = self.connection.recv_match(blocking=True, timeout=timeout)
        self._handle(message)
        return message

    def ready(self):
        now = self._now()
        return (self.last_heartbeat is not None and now - self.last_heartbeat <= HEARTBEAT_MAX_AGE_S and
                self.last_fc is not None and now - self.last_fc[1] <= FC_MAX_AGE_S and self.origin is not None)

    def wait_ready(self, timeout=30.0):
        deadline = self._now() + timeout
        while self._now() < deadline:
            self.poll(0.2)
            if self.ready(): return
        raise FlightSafetyError(
            "flight-controller telemetry preconditions not satisfied before timeout "
            f"(heartbeat={self.last_heartbeat is not None}, "
            f"fc_position={self.last_fc is not None}, "
            f"valid_uwb={self.last_uwb is not None})")

    def send_command_no_ack(self, command, params=()):
        mav = self.connection.mav
        values = list(params) + [0.0] * (7 - len(params))
        message = mav.command_long_encode(self.target_system, self.target_component, command, 0, *values[:7])
        self._send(message)

    def command(self, command, params=(), timeout=3.0):
        """Send a command and require an accepted COMMAND_ACK."""
        mav = self.connection.mav
        values = list(params) + [0.0] * (7 - len(params))
        message = mav.command_long_encode(
            self.target_system, self.target_component, command, 0, *values[:7])
        self._send(message)
        if self.dry_run:
            return True
        deadline = self._now() + timeout
        accepted = getattr(self.protocol, "MAV_RESULT_ACCEPTED", 0)
        while self._now() < deadline:
            received = self.poll(0.2)
            if (received is not None and received.get_type() == "COMMAND_ACK" and
                    getattr(received, "command", None) == command):
                result = getattr(received, "result", None)
                if result not in (0, accepted):
                    raise FlightSafetyError(f"command {command} rejected: {result}")
                return True
        raise FlightSafetyError(f"command {command} ACK timeout")

    def setpoint(self, x, y, z):
        if self.origin is not None and not self.fence.contains((x, y, z)):
            raise FlightSafetyError(f"setpoint {(x, y, z)} is outside virtual fence")
        mav = self.connection.mav
        mask = 0x0DF8
        message = mav.set_position_target_local_ned_encode(0, self.target_system, self.target_component,
            self.protocol.MAV_FRAME_LOCAL_NED, mask, float(x), float(y), float(z), 0, 0, 0, 0, 0, 0, 0, 0)
        self._send(message)

    def velocity_setpoint(self, vx, vy, vz):
        """Send a capped local-NED velocity target for slow waypoint motion."""
        velocity = (float(vx), float(vy), float(vz))
        speed = math.sqrt(sum(value * value for value in velocity))
        if not math.isfinite(speed) or speed > self.max_speed_mps + 1e-9:
            raise FlightSafetyError(
                f"velocity command {speed:.2f} m/s exceeds limit {self.max_speed_mps:.2f} m/s")
        mav = self.connection.mav
        # Ignore position and acceleration fields; use only vx/vy/vz.
        mask = 0x0DC7
        message = mav.set_position_target_local_ned_encode(
            0, self.target_system, self.target_component,
            self.protocol.MAV_FRAME_LOCAL_NED, mask,
            0, 0, 0, velocity[0], velocity[1], velocity[2],
            0, 0, 0, 0, 0)
        self._send(message)

    def arm(self):
        if self.dry_run:
            print("DRY-RUN SET_MODE MAV_MODE_AUTO_ARMED")
            return
        print("Sending FanciSwarm UNLOCK via SET_MODE")
        self.connection.mav.set_mode_send(
            self.target_system,
            self.protocol.MAV_MODE_AUTO_ARMED,
            0,
            force_mavlink1=True,
        )
        deadline = self._now() + 5.0
        while self._now() < deadline:
            self.poll(0.2)
            print("UNLOCK wait:", "system=", self.target_system,
                  "component=", self.target_component,
                  "base_mode=", self.last_base_mode,
                  "soft_armed=", self.last_armed)
            if self.last_armed:
                print("FC reports ARMED")
                return
        raise FlightSafetyError("FanciSwarm unlock failed")

    def takeoff(self):
        if self.dry_run:
            print("DRY-RUN MAV_CMD_NAV_TAKEOFF")
            return
        print("Sending FanciSwarm TAKEOFF")
        self.send_command_no_ack(
            self.protocol.MAV_CMD_NAV_TAKEOFF,
            (0, 0, 0, 0, 0, 0, self.takeoff_height),
        )
        soft_arm_deadline = self._now() + 5.0
        while self._now() < soft_arm_deadline:
            self.poll(0.2)
            print("TAKEOFF wait:", "base_mode=", self.last_base_mode,
                  "soft_armed=", self.last_armed)
            if self.last_armed:
                print("FC reports SOFT ARMED - takeoff sequence started")
                break
        else:
            raise FlightSafetyError("TAKEOFF sent but FC did not become soft-armed")

        observation_deadline = self._now() + 15.0
        while self._now() < observation_deadline:
            self.poll(0.2)
            if self.last_fc is not None and self.origin is not None:
                dz = self.last_fc[0][2] - self.origin[2]
                print(f"FC position: {self.last_fc[0]}, dz={dz:.3f}")
        print("TAKEOFF observation window finished")

    def land(self):
        if self.dry_run:
            print("DRY-RUN MAV_CMD_NAV_LAND")
            return
        print("Sending FanciSwarm LAND")
        try:
            self.send_command_no_ack(self.protocol.MAV_CMD_NAV_LAND)
        except Exception as exc: print(f"LAND failed: {exc}")

    def hover(self, seconds=None):
        if self.dry_run:
            print(f"DRY-RUN HOVER {self.hover_seconds if seconds is None else seconds:.1f}s")
            return
        deadline = self._now() + (self.hover_seconds if seconds is None else seconds)
        while self._now() < deadline:
            if self.last_fc is None or self._now() - self.last_fc[1] > FC_MAX_AGE_S:
                raise FlightSafetyError("FC position telemetry stale during hover")
            self.poll(0.1)

    def planned_route(self):
        """Return the validated local-NED route before arming the vehicle."""
        if self.waypoints:
            route = self.waypoints
        else:
            d = self.waypoint_distance
            route = ((d, 0.0, -1.0), (d, d, -1.0), (0.0, 0.0, -1.0))
        for point in route:
            if not self.fence.contains(point):
                raise FlightSafetyError(f"planned waypoint {point} is outside virtual fence")
        return route

    def fly_waypoint(self, x, y, z= -1.0):
        target = (float(x), float(y), float(z))
        if not self.fence.contains(target):
            raise FlightSafetyError(f"waypoint {target} is outside virtual fence")
        if self.dry_run:
            self.velocity_setpoint(0.0, 0.0, 0.0)
            return
        deadline = self._now() + max(self.hover_seconds, 1.0)
        while self._now() < deadline:
            if self.last_fc is None or self.origin is None:
                raise FlightSafetyError("FC position telemetry unavailable while flying")
            if self._now() - self.last_fc[1] > FC_MAX_AGE_S:
                raise FlightSafetyError("FC position telemetry stale while flying")
            current = tuple(self.last_fc[0][index] - self.origin[index] for index in range(3))
            delta = tuple(target[index] - current[index] for index in range(3))
            distance = math.sqrt(sum(value * value for value in delta))
            if distance <= WAYPOINT_TOLERANCE_M:
                self.velocity_setpoint(0.0, 0.0, 0.0)
                return
            scale = min(self.max_speed_mps, distance * SETPOINT_RATE_HZ)
            velocity = tuple(value * scale / distance for value in delta)
            self.velocity_setpoint(*velocity)
            self.poll(1.0 / SETPOINT_RATE_HZ)
            if self.last_fc is not None and self.origin is not None:
                current = tuple(self.last_fc[0][index] - self.origin[index] for index in range(3))
                if math.dist(current, target) <= WAYPOINT_TOLERANCE_M:
                    self.velocity_setpoint(0.0, 0.0, 0.0)
                    return
        raise FlightSafetyError(f"waypoint {target} was not reached before timeout")

    def stop_safely(self):
        if self.stopping: return
        self.stopping = True
        self.land()

    def run(self):
        try:
            route = () if self.takeoff_only else self.planned_route()
            self.wait_ready()
            self.arm(); self.takeoff(); self.hover()
            if not self.takeoff_only:
                for waypoint in route:
                    self.fly_waypoint(*waypoint); self.hover()
            self.land()
        except KeyboardInterrupt:
            self.stop_safely(); raise
        except Exception:
            self.stop_safely(); raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udpout:192.168.1.143:14550",
                        help="MAVLink UDP output endpoint; the bridge returns telemetry to this socket")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--takeoff-only", action="store_true")
    parser.add_argument("--takeoff-height", type=float, default=1.0)
    parser.add_argument("--hover-seconds", type=float, default=5.0)
    parser.add_argument("--waypoint-distance", type=float, default=0.3,
                        help="horizontal distance in metres for the demo route")
    parser.add_argument("--waypoints-file",
                        help="CSV teaching route with x_m,y_m,z_m local-NED columns")
    parser.add_argument("--fence-half-extent-m", type=float, default=0.6,
                        help="symmetric X/Y virtual-fence half extent in metres")
    parser.add_argument("--fence-min-z-m", type=float, default=-1.5,
                        help="minimum local-NED Z inside the virtual fence")
    parser.add_argument("--fence-max-z-m", type=float, default=0.5,
                        help="maximum local-NED Z inside the virtual fence")
    parser.add_argument("--max-speed-mps", type=float, default=DEFAULT_MAX_SPEED_MPS,
                        help="maximum commanded and measured speed (capped at 0.2 m/s)")
    args = parser.parse_args(argv)
    if args.waypoint_distance <= 0:
        parser.error("--waypoint-distance must be greater than zero")
    if args.fence_half_extent_m <= 0:
        parser.error("--fence-half-extent-m must be positive")
    try:
        fence = VirtualFence(
            -args.fence_half_extent_m, args.fence_half_extent_m,
            -args.fence_half_extent_m, args.fence_half_extent_m,
            args.fence_min_z_m, args.fence_max_z_m)
        waypoints = load_waypoints(args.waypoints_file) if args.waypoints_file else None
    except ValueError as exc:
        parser.error(str(exc))
    try:
        from pymavlink import mavutil
        connection = mavutil.mavlink_connection(
            args.connection,
            source_system=254,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            force_mavlink1=True,
        )
        print(f"MAVLink connection: {args.connection} (forced MAVLink 1)")
        flight = AutonomousFlight(connection, dry_run=args.dry_run, takeoff_only=args.takeoff_only,
                                   takeoff_height=args.takeoff_height, hover_seconds=args.hover_seconds,
                                   waypoint_distance=args.waypoint_distance, waypoints=waypoints,
                                   fence=fence, max_speed_mps=args.max_speed_mps)
        flight.run()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
