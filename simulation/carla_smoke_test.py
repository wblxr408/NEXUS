#!/usr/bin/env python3
"""Minimal CARLA connectivity smoke test.

Connects to a running CARLA server, prints versions/map, spawns one vehicle
plus an RGB camera, ticks a few synchronous frames, and saves a single frame.
Run with the py3.12 venv that has the carla 0.9.16 wheel installed.
"""
import argparse
import importlib.metadata
import os
import queue
import shutil
import socket
import struct
import subprocess
import sys
import time

DEFAULT_HOST, DEFAULT_PORT = "127.0.0.1", 2000
OUT = "/tmp/carla_smoke_frame.png"


def _default_gateway(route_file: str = "/proc/net/route") -> str | None:
    """Return the IPv4 default gateway reported by the WSL routing table."""
    try:
        with open(route_file, encoding="ascii") as routes:
            next(routes, None)  # skip the header
            for line in routes:
                fields = line.split()
                if len(fields) < 4 or fields[1] != "00000000":
                    continue
                try:
                    flags = int(fields[3], 16)
                    gateway = int(fields[2], 16)
                except ValueError:
                    continue
                if flags & 0x2 and gateway:
                    # /proc/net/route stores the address in host byte order.
                    return socket.inet_ntoa(struct.pack("<I", gateway))
    except (FileNotFoundError, OSError):
        pass
    return None


def _port_is_open(host: str, port: int, timeout: float = 0.25) -> bool:
    """Check whether a TCP endpoint is reachable without touching CARLA."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _wsl_networking_mode() -> str | None:
    """Return WSL's configured networking mode when wslinfo is available."""
    wslinfo = shutil.which("wslinfo")
    if not wslinfo:
        return None
    try:
        result = subprocess.run(
            [wslinfo, "--networking-mode"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    mode = result.stdout.strip().lower()
    return mode or None


def resolve_host(requested_host: str | None, port: int) -> str:
    """Resolve CLI/environment host, or discover mirrored WSL versus NAT.

    In mirrored mode CARLA is reachable at localhost.  In NAT mode the WSL
    default gateway forwards to the Windows host.  Modern WSL reports the mode
    through wslinfo; endpoint probing remains as a fallback for older WSL.
    """
    if requested_host:
        return requested_host
    env_host = os.environ.get("CARLA_HOST", "").strip()
    if env_host:
        return env_host

    mode = _wsl_networking_mode()
    if mode == "mirrored":
        return DEFAULT_HOST
    if mode == "nat":
        return _default_gateway() or DEFAULT_HOST

    if _port_is_open(DEFAULT_HOST, port):
        return DEFAULT_HOST
    gateway = _default_gateway()
    if gateway and _port_is_open(gateway, port):
        return gateway
    return gateway or DEFAULT_HOST


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default=None,
        help="CARLA server host (default: CARLA_HOST, then WSL auto-detection)",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=DEFAULT_PORT,
        metavar="PORT",
        help=f"CARLA RPC TCP port (default: {DEFAULT_PORT})",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    host = resolve_host(args.host, args.port)
    print(f"connecting to CARLA at {host}:{args.port}")
    try:
        import carla
    except ImportError as exc:
        raise RuntimeError(
            "CARLA Python API is not installed; run with simulation/carla/.venv-carla/bin/python"
        ) from exc
    client = carla.Client(host, args.port)
    client.set_timeout(60.0)
    client_api_version = client.get_client_version()
    server_version = client.get_server_version()
    # CARLA's released wheel reports its git build id from get_client_version()
    # (for 0.9.16 this is commonly ``294096eb1-dirty``), while the server
    # reports the package version.  Compare the wheel metadata to the server,
    # not those two different representations.
    try:
        client_package_version = importlib.metadata.version("carla")
    except importlib.metadata.PackageNotFoundError:
        client_package_version = None
    print("client version:", client_api_version)
    if client_package_version:
        print("client wheel version:", client_package_version)
    print("server version:", server_version)
    if client_package_version and client_package_version != server_version:
        raise RuntimeError(
            "CARLA client/server version mismatch: "
            f"client_wheel={client_package_version}, server={server_version}"
        )
    print("CARLA versions match:", client_package_version or server_version)

    # RPC port opens before the world finishes streaming; retry get_world().
    world = None
    last_err = None
    for attempt in range(6):
        try:
            world = client.get_world()
            break
        except RuntimeError as e:
            last_err = e
            print(f"  get_world() not ready yet (attempt {attempt + 1}/6): {e}")
            if attempt < 5:
                time.sleep(2.0)
    if world is None:
        if last_err is not None:
            raise RuntimeError("CARLA world was not ready after 6 attempts") from last_err
        raise RuntimeError("CARLA client returned no world")
    print("current map:", world.get_map().name)

    # deterministic ticking
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    print("spawn points available:", len(spawn_points))

    actors = []
    try:
        if not spawn_points:
            raise RuntimeError("CARLA map has no vehicle spawn points")
        vehicle_bp = bp_lib.filter("vehicle.*")[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
        actors.append(vehicle)
        print("spawned vehicle:", vehicle.type_id, "at", spawn_points[0].location)

        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "640")
        cam_bp.set_attribute("image_size_y", "480")
        cam = world.spawn_actor(
            cam_bp, carla.Transform(carla.Location(x=1.5, z=1.6)), attach_to=vehicle
        )
        actors.append(cam)

        images: queue.Queue[carla.Image] = queue.Queue()
        cam.listen(images.put)

        for _ in range(20):
            world.tick()
        image = images.get(timeout=10.0)
        while not images.empty():
            image = images.get_nowait()
        image.save_to_disk(OUT)
        if not os.path.isfile(OUT) or os.path.getsize(OUT) == 0:
            raise RuntimeError(f"camera frame was not saved correctly: {OUT}")
        print("ticked 20 frames; saved camera frame id:", image.frame)
        print("saved frame ->", OUT)
        print("total actors in world:", len(world.get_actors()))
        print("SMOKE_TEST_OK")
        return 0
    finally:
        for a in reversed(actors):
            try:
                a.destroy()
            except Exception:
                pass
        if world is not None:
            world.apply_settings(original_settings)


if __name__ == "__main__":
    sys.exit(main())
