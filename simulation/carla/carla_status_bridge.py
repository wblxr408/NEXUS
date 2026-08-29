#!/usr/bin/env python3
"""WSL CARLA status bridge for the dual-window dashboard.

The Windows CARLA process remains the renderer and RPC server. This process
only observes that server and broadcasts a small JSON snapshot to browsers.
Use --mock to validate the WebSocket contract without a running CARLA server.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import logging
import math
import os
import struct
import threading
import time
import zlib
from dataclasses import dataclass, field
from typing import Any


LOG = logging.getLogger("carla_status_bridge")


def _speed(vector: Any) -> float:
    if vector is None:
        return 0.0
    return math.sqrt(float(vector.x) ** 2 + float(vector.y) ** 2 + float(vector.z) ** 2)


def _transform_payload(transform: Any) -> dict[str, float]:
    return {
        "x": round(float(transform.location.x), 4),
        "y": round(float(transform.location.y), 4),
        "z": round(float(transform.location.z), 4),
        "roll": round(float(transform.rotation.roll), 3),
        "pitch": round(float(transform.rotation.pitch), 3),
        "yaw": round(float(transform.rotation.yaw), 3),
    }


def _actor_counts(actors: Any) -> dict[str, int]:
    counts = {"actor_count": 0, "vehicle_count": 0, "walker_count": 0, "traffic_light_count": 0}
    for actor in actors:
        counts["actor_count"] += 1
        type_id = getattr(actor, "type_id", "")
        if type_id.startswith("vehicle."):
            counts["vehicle_count"] += 1
        elif type_id.startswith("walker."):
            counts["walker_count"] += 1
        elif type_id.startswith("traffic.traffic_light"):
            counts["traffic_light_count"] += 1
    return counts


@dataclass
class BridgeState:
    payload: dict[str, Any] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def replace(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.payload = payload

    def merge(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.payload = {**self.payload, **payload}

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return json.loads(json.dumps(self.payload))


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)


def _image_data_uri(image: Any, max_width: int = 480) -> str | None:
    """Encode a CARLA BGRA image as a dependency-free PNG data URI."""
    width, height = int(image.width), int(image.height)
    if width <= 0 or height <= 0 or not image.raw_data:
        return None
    scale = max(1, math.ceil(width / max_width))
    out_width, out_height = math.ceil(width / scale), math.ceil(height / scale)
    raw = bytes(image.raw_data)
    rows = bytearray()
    for y in range(out_height):
        source_y = min(height - 1, y * scale)
        rows.append(0)
        for x in range(out_width):
            source_x = min(width - 1, x * scale)
            offset = (source_y * width + source_x) * 4
            b, g, r = raw[offset : offset + 3]
            rows.extend((r, g, b, 255))
    png = bytearray(b"\x89PNG\r\n\x1a\n")
    png.extend(_png_chunk(b"IHDR", struct.pack(">IIBBBBB", out_width, out_height, 8, 6, 0, 0, 0)))
    png.extend(_png_chunk(b"IDAT", zlib.compress(bytes(rows), level=3)))
    png.extend(_png_chunk(b"IEND", b""))
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")


class CarlaObserver:
    def __init__(self, host: str, port: int, map_name: str, state: BridgeState, spawn_demo_ego: bool = False):
        self.host = host
        self.port = port
        self.map_name = map_name
        self.state = state
        self.spawn_demo_ego = spawn_demo_ego
        self.stop_event = threading.Event()
        self.demo_actors: list[Any] = []
        self._last_thumbnail_wall = 0.0

    def close(self) -> None:
        self.stop_event.set()
        for actor in reversed(self.demo_actors):
            try:
                actor.destroy()
            except Exception:
                pass
        self.demo_actors.clear()

    def run(self) -> None:
        try:
            import carla
        except ImportError:
            self.state.replace({"carla_connected": False, "endpoint": f"{self.host}:{self.port}", "log": "CARLA Python API unavailable"})
            return
        while not self.stop_event.is_set():
            try:
                client = carla.Client(self.host, self.port)
                client.set_timeout(5.0)
                world = client.get_world()
                if self.map_name and not world.get_map().name.endswith(self.map_name):
                    world = client.load_world(self.map_name)
                LOG.info("connected to CARLA map=%s", world.get_map().name)
                self._observe_world(world)
            except Exception as exc:
                LOG.warning("CARLA connection lost: %s", exc)
                self.state.replace({
                    "carla_connected": False,
                    "endpoint": f"{self.host}:{self.port}",
                    "map": self.map_name,
                    "log": {"level": "ERROR", "text": f"CARLA connection lost: {exc}"},
                })
                self.stop_event.wait(1.0)

    def _observe_world(self, world: Any) -> None:
        if self.spawn_demo_ego:
            self._ensure_demo_ego(world)
        previous_wall = time.monotonic()
        previous_frame = None
        tick_hz = 0.0
        while not self.stop_event.is_set():
            snapshot = world.wait_for_tick(seconds=1.0)
            if snapshot is None:
                continue
            now = time.monotonic()
            if previous_frame is not None and now > previous_wall:
                tick_hz = 1.0 / (now - previous_wall)
            previous_wall = now
            previous_frame = snapshot.frame
            actors = world.get_actors()
            counts = _actor_counts(actors)
            ego = next((a for a in actors if "hero" in str(a.attributes.get("role_name", ""))), None)
            if ego is None:
                ego = next((a for a in actors if getattr(a, "type_id", "").startswith("vehicle.")), None)
            ego_pose = _transform_payload(ego.get_transform()) if ego else None
            if ego:
                ego_pose["speed"] = round(_speed(ego.get_velocity()), 4)
            traffic = []
            for light in actors.filter("traffic.traffic_light*"):
                traffic.append(str(getattr(light, "state", "UNKNOWN")).split(".")[-1])
            previous = self.state.snapshot()
            sensors = {"carla_rpc": "ok", "ego": "ok" if ego else "missing"}
            if previous.get("thumbnail"):
                sensors["rgb_front"] = "ok"
            payload = {
                "carla_connected": True,
                "endpoint": f"{self.host}:{self.port}",
                "map": world.get_map().name,
                "sync_mode": bool(world.get_settings().synchronous_mode),
                "tick_hz": round(tick_hz, 2),
                **counts,
                "frame": int(snapshot.frame),
                "timestamp": float(snapshot.timestamp.elapsed_seconds),
                "ego_pose": ego_pose,
                "sensors": sensors,
                "traffic_summary": ", ".join(traffic[:12]) if traffic else "none",
                "log": {"level": "INFO", "text": f"CARLA frame {snapshot.frame} received"},
            }
            if previous.get("thumbnail"):
                payload["thumbnail"] = previous["thumbnail"]
                payload["camera_frame"] = previous.get("camera_frame")
            self.state.replace(payload)

    def _on_camera(self, image: Any) -> None:
        try:
            now = time.monotonic()
            if now - self._last_thumbnail_wall < 0.4:
                return
            self._last_thumbnail_wall = now
            thumbnail = _image_data_uri(image)
            if thumbnail:
                self.state.merge({"thumbnail": thumbnail, "camera_frame": int(image.frame), "sensors": {"rgb_front": "ok"}})
        except Exception as exc:
            LOG.debug("camera encoding failed: %s", exc)

    def _ensure_demo_ego(self, world: Any) -> None:
        try:
            import carla
            actors = world.get_actors()
            ego = next((a for a in actors if "hero" in str(a.attributes.get("role_name", ""))), None)
            if ego is not None:
                return
            spawn_points = world.get_map().get_spawn_points()
            if not spawn_points:
                LOG.warning("sandbox map has no spawn point; demo ego not spawned")
                return
            blueprint = world.get_blueprint_library().filter("vehicle.*")[0]
            if blueprint.has_attribute("role_name"):
                blueprint.set_attribute("role_name", "hero")
            ego = world.try_spawn_actor(blueprint, spawn_points[0])
            if ego is None:
                LOG.warning("unable to spawn demo ego")
                return
            camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
            camera_bp.set_attribute("image_size_x", "480")
            camera_bp.set_attribute("image_size_y", "270")
            camera_bp.set_attribute("fov", "90")
            camera = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=1.5, z=1.6)), attach_to=ego)
            camera.listen(self._on_camera)
            self.demo_actors = [ego, camera]
            LOG.info("spawned demo ego=%s and RGB camera", ego.id)
        except Exception as exc:
            LOG.warning("demo ego setup failed: %s", exc)



def mock_payload(frame: int) -> dict[str, Any]:
    return {
        "carla_connected": True,
        "endpoint": "127.0.0.1:2000",
        "map": "sandbox-v29",
        "sync_mode": True,
        "tick_hz": 20.0,
        "actor_count": 8,
        "vehicle_count": 2,
        "walker_count": 1,
        "traffic_light_count": 4,
        "frame": frame,
        "timestamp": round(frame / 20.0, 3),
        "ego_pose": {"x": 221.3 + frame * 0.01, "y": -0.6, "z": 0.8, "roll": 0.0, "pitch": 0.0, "yaw": -88.7, "speed": 7.2},
        "target": {"x": 223.1, "y": 12.4, "z": 0.0},
        "sensors": {"rgb_front": "ok", "imu": "ok", "gnss": "ok"},
        "traffic_summary": "GREEN · RED · RED · GREEN",
        "algorithms": {
            "orb_slam2": {"status": "TRACKING", "summary": "54 features", "latencyMs": 38.4},
            "uwb": {"status": "OK", "summary": "4 anchors", "latencyMs": 11.2},
            "detector": {"status": "OK", "summary": "target bbox", "latencyMs": 28.9},
            "fusion": {"status": "VALID", "summary": "confidence 0.91", "latencyMs": 16.7},
        },
        "log": {"level": "INFO", "text": "RGB frame received"},
    }


async def serve(state: BridgeState, host: str, port: int, mock: bool) -> None:
    try:
        import websockets
    except ImportError as exc:
        raise RuntimeError("websockets is required; install with pip install websockets") from exc
    clients: set[Any] = set()

    async def handler(websocket: Any) -> None:
        clients.add(websocket)
        try:
            await websocket.send(json.dumps(state.snapshot(), ensure_ascii=False))
            await websocket.wait_closed()
        finally:
            clients.discard(websocket)

    async with websockets.serve(handler, host, port):
        LOG.info("CARLA status WebSocket listening on ws://%s:%d/ws/carla_status", host, port)
        frame = 0
        while True:
            if mock:
                frame += 1
                state.replace(mock_payload(frame))
            message = json.dumps(state.snapshot(), ensure_ascii=False)
            if clients:
                await asyncio.gather(*(client.send(message) for client in tuple(clients)), return_exceptions=True)
            await asyncio.sleep(0.05 if mock else 0.1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", default="sandbox-v29")
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8766)
    parser.add_argument("--mock", action="store_true", help="publish deterministic data without connecting to CARLA")
    parser.add_argument("--spawn-demo-ego", action="store_true", help="spawn a hero vehicle and RGB camera when none exists")
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args()
    state = BridgeState({"carla_connected": False, "endpoint": f"{args.host}:{args.port}", "map": args.map})
    observer = None
    if args.mock:
        state.replace(mock_payload(0))
    else:
        observer = CarlaObserver(args.host, args.port, args.map, state, spawn_demo_ego=args.spawn_demo_ego)
        threading.Thread(target=observer.run, name="carla-observer", daemon=True).start()
    try:
        asyncio.run(serve(state, args.listen, args.web_port, args.mock))
    except KeyboardInterrupt:
        return 0
    finally:
        if observer:
            observer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
