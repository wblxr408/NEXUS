import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


MODULE_PATH = Path(__file__).parents[2] / "tools" / "fanciswarm_autonomous_flight.py"
SPEC = importlib.util.spec_from_file_location("fanciswarm_autonomous_flight", MODULE_PATH)
flight_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(flight_module)


class FakeMav:
    MAV_AUTOPILOT_INVALID = 8
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    FANCISWARM_UNLOCK_FLAG = 1
    MAV_FRAME_LOCAL_NED = 1
    MAV_CMD_COMPONENT_ARM_DISARM = 400

    def __init__(self):
        self.setpoint_calls = []
        self.sent = []

    def send(self, message, **kwargs):
        self.sent.append((message, kwargs))

    def set_position_target_local_ned_encode(self, *args):
        self.setpoint_calls.append(args)
        return SimpleNamespace(get_type=lambda: "SET_POSITION_TARGET_LOCAL_NED")


class FakeConnection:
    mav = FakeMav()


def heartbeat(system, component, autopilot, base_mode=0):
    return SimpleNamespace(
        get_type=lambda: "HEARTBEAT",
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: component,
        autopilot=autopilot,
        base_mode=base_mode,
    )


def test_companion_heartbeat_does_not_replace_flight_controller_target():
    flight = flight_module.AutonomousFlight(FakeConnection(), clock=lambda: 10.0)

    flight._handle(heartbeat(254, 190, FakeMav.MAV_AUTOPILOT_INVALID))
    assert flight.last_heartbeat is None
    assert (flight.target_system, flight.target_component) == (1, 1)

    flight._handle(heartbeat(42, 1, 3, FakeMav.FANCISWARM_UNLOCK_FLAG))
    assert flight.last_heartbeat == 10.0
    assert (flight.target_system, flight.target_component) == (42, 1)
    assert flight.last_base_mode == FakeMav.FANCISWARM_UNLOCK_FLAG
    assert flight.last_armed is True

    invalid_fc = flight_module.AutonomousFlight(FakeConnection(), clock=lambda: 10.0)
    invalid_fc._handle(heartbeat(42, 1, FakeMav.MAV_AUTOPILOT_INVALID))
    assert invalid_fc.last_heartbeat == 10.0


def test_fanciswarm_unlock_bit_is_distinct_from_standard_armed_bit():
    flight = flight_module.AutonomousFlight(FakeConnection(), clock=lambda: 10.0)

    flight._handle(heartbeat(42, 1, 3, 1))

    assert flight.last_base_mode == 1
    assert flight.last_armed is True
    assert bool(flight.last_base_mode & FakeMav.MAV_MODE_FLAG_SAFETY_ARMED) is False


def test_arm_uses_standard_component_arm_command():
    flight = flight_module.AutonomousFlight(FakeConnection(), clock=lambda: 0.0)
    calls = []
    flight.send_command_no_ack = lambda command, params=(): calls.append((command, params))
    flight.poll = lambda timeout: setattr(flight, "last_armed", True)

    flight.arm()

    assert calls == [(FakeMav.MAV_CMD_COMPONENT_ARM_DISARM, (1.0, 0.0))]


def test_teaching_route_is_loaded_and_checked_against_virtual_fence(tmp_path):
    route = tmp_path / "route.csv"
    route.write_text("x_m,y_m,z_m\n0.0,0.0,-1.0\n0.3,0.0,-1.0\n", encoding="utf-8")
    points = flight_module.load_waypoints(route)
    assert points == ((0.0, 0.0, -1.0), (0.3, 0.0, -1.0))

    fence = flight_module.VirtualFence(-0.4, 0.4, -0.4, 0.4, -1.5, 0.3)
    flight = flight_module.AutonomousFlight(
        FakeConnection(), waypoints=points, fence=fence, max_speed_mps=0.2)
    assert flight.max_speed_mps == 0.2
    assert flight.planned_route() == points

    with pytest.raises(ValueError, match="outside the virtual fence"):
        flight_module.AutonomousFlight(
            FakeConnection(), waypoints=((0.5, 0.0, -1.0),), fence=fence)

    with pytest.raises(flight_module.FlightSafetyError, match="planned waypoint"):
        flight_module.AutonomousFlight(
            FakeConnection(), waypoint_distance=0.8, fence=fence).planned_route()


def test_speed_and_fence_violations_raise_flight_safety_error():
    clock_now = [10.0]
    flight = flight_module.AutonomousFlight(
        FakeConnection(), fence=flight_module.VirtualFence(-0.4, 0.4, -0.4, 0.4, -1.5, 0.3),
        max_speed_mps=0.2, clock=lambda: clock_now[0])
    first = SimpleNamespace(get_type=lambda: "GLOBAL_VISION_POSITION_ESTIMATE", x=0, y=0, z=0)
    flight._handle(first)

    clock_now[0] = 11.0
    too_fast = SimpleNamespace(get_type=lambda: "GLOBAL_VISION_POSITION_ESTIMATE", x=30, y=0, z=0)
    with pytest.raises(flight_module.FlightSafetyError, match="exceeds limit"):
        flight._handle(too_fast)

    fence_flight = flight_module.AutonomousFlight(
        FakeConnection(), fence=flight_module.VirtualFence(-0.4, 0.4, -0.4, 0.4, -1.5, 0.3),
        max_speed_mps=0.2, clock=lambda: clock_now[0])
    fence_flight._handle(first)
    clock_now[0] = 20.0
    outside = SimpleNamespace(get_type=lambda: "GLOBAL_VISION_POSITION_ESTIMATE", x=0, y=50, z=0)
    with pytest.raises(flight_module.FlightSafetyError, match="outside virtual fence"):
        fence_flight._handle(outside)


def test_velocity_setpoint_uses_velocity_mask_and_hard_speed_cap():
    connection = FakeConnection()
    flight = flight_module.AutonomousFlight(connection, max_speed_mps=0.2)

    flight.velocity_setpoint(0.2, 0.0, 0.0)
    call = connection.mav.setpoint_calls[-1]
    assert call[4] == 0x0DC7
    assert tuple(call[8:11]) == (0.2, 0.0, 0.0)
    assert tuple(call[5:8]) == (0, 0, 0)

    with pytest.raises(flight_module.FlightSafetyError, match="exceeds limit"):
        flight.velocity_setpoint(0.15, 0.15, 0.0)


def test_stale_fc_position_aborts_hover_and_waypoint_motion():
    flight = flight_module.AutonomousFlight(
        FakeConnection(), clock=lambda: 2.0, hover_seconds=1.0)
    flight.origin = (0.0, 0.0, 0.0)
    flight.last_fc = ((0.0, 0.0, 0.0), 0.0)

    with pytest.raises(flight_module.FlightSafetyError, match="stale"):
        flight.hover(1.0)
    with pytest.raises(flight_module.FlightSafetyError, match="stale"):
        flight.fly_waypoint(0.1, 0.0, -1.0)
