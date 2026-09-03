"""NEXUS WebSocket gateway for the CARLA sandbox dashboard."""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
NEXUS_WEB = ROOT.parent / "ros2_ws" / "src" / "nexus_viz_dashboard" / "web"

app = FastAPI(title="NEXUS CARLA gateway", version="1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://127.0.0.1", "http://localhost",
        "http://127.0.0.1:8765", "http://localhost:8765",
    ],
    allow_methods=["*"],
    allow_headers=["*"],
)

if (NEXUS_WEB / "src").is_dir():
    app.mount("/src", StaticFiles(directory=NEXUS_WEB / "src"), name="nexus-src")
if (NEXUS_WEB / "public" / "assets").is_dir():
    app.mount("/assets", StaticFiles(directory=NEXUS_WEB / "public" / "assets"), name="nexus-assets")

clients: set[WebSocket] = set()
status_clients: set[WebSocket] = set()
carla_socket: WebSocket | None = None
latest_scene: dict[str, Any] | None = None
latest_targets: list[dict[str, Any]] = []
lock = asyncio.Lock()
carla_send_lock = asyncio.Lock()
SEND_TIMEOUT_S = 0.25


def _command_authorized(websocket: WebSocket) -> bool:
    """Require an explicit shared token before forwarding simulator commands."""
    token = os.environ.get("NEXUS_CARLA_TOKEN", "")
    return bool(token) and websocket.query_params.get("token") == token


async def _send_with_timeout(client: WebSocket, message: dict[str, Any]) -> bool:
    try:
        await asyncio.wait_for(client.send_json(message), timeout=SEND_TIMEOUT_S)
        return True
    except Exception:
        return False


@app.get("/")
async def index() -> FileResponse:
    page = NEXUS_WEB / "public" / "index.html"
    return FileResponse(page if page.exists() else WEB / "index.html")


@app.get("/health")
async def health() -> dict[str, Any]:
    return {
        "ok": True,
        "carla_connected": carla_socket is not None,
        "has_scene": latest_scene is not None,
    }


@app.get("/api/scene")
async def scene() -> dict[str, Any]:
    return latest_scene or {
        "type": "scene",
        "map": {"name": "waiting", "roads": []},
        "actors": [],
    }


async def broadcast(message: dict[str, Any]) -> None:
    snapshot = tuple(clients)
    results = await asyncio.gather(*(_send_with_timeout(client, message) for client in snapshot))
    dead = [client for client, ok in zip(snapshot, results) if not ok]
    for client in dead:
        clients.discard(client)


async def broadcast_status(message: dict[str, Any]) -> None:
    snapshot = tuple(status_clients)
    results = await asyncio.gather(*(_send_with_timeout(client, message) for client in snapshot))
    dead = [client for client, ok in zip(snapshot, results) if not ok]
    for client in dead:
        status_clients.discard(client)


def status_from_scene(scene_message: dict[str, Any]) -> dict[str, Any]:
    actors = scene_message.get("actors", [])
    vehicles = [a for a in actors if a.get("type") == "vehicle"]
    walkers = [a for a in actors if a.get("type") == "walker"]
    hero = next((a for a in actors if a.get("role") == "hero"), None)
    ego = hero.get("position", {}) if hero else None
    if hero:
        ego = {
            **ego,
            **hero.get("rotation", {}),
            "speed": hero.get("speed"),
        }
    simulation = scene_message.get("simulation", {})
    return {
        "type": "carla_status",
        "protocol": scene_message.get("protocol"),
        "carla_connected": True,
        "connected": True,
        "endpoint": "127.0.0.1:2000",
        "map": scene_message.get("map", {}).get("name"),
        "actor_count": len(actors),
        "vehicle_count": len(vehicles),
        "walker_count": len(walkers),
        "traffic_light_count": 0,
        "ego_pose": ego,
        "frame": simulation.get("frame"),
        "timestamp": scene_message.get("timestamp"),
        "sync_mode": simulation.get("synchronous"),
        f"actor_summary": f"{len(vehicles)} veh · {len(walkers)} ped",
    }


@app.websocket("/ws/carla")
async def carla_feed(websocket: WebSocket) -> None:
    global carla_socket, latest_scene
    if not _command_authorized(websocket):
        await websocket.close(code=1008, reason="NEXUS_CARLA_TOKEN is required")
        return
    await websocket.accept()
    carla_socket = websocket
    try:
        while True:
            message = json.loads(await websocket.receive_text())
            if message.get("type") != "scene":
                continue
            async with lock:
                latest_scene = message
            await broadcast(message)
            await broadcast_status(status_from_scene(message))
    except (WebSocketDisconnect, json.JSONDecodeError):
        pass
    finally:
        if carla_socket is websocket:
            carla_socket = None
        await broadcast({"type": "status", "carla_connected": False})
        await broadcast_status({"type": "carla_status", "carla_connected": False, "connected": False})


@app.websocket("/ws")
async def browser_feed(websocket: WebSocket) -> None:
    await websocket.accept()
    clients.add(websocket)
    if latest_scene:
        await websocket.send_json(latest_scene)
    for target in latest_targets:
        await websocket.send_json(target)
    if carla_socket:
        await websocket.send_json({"type": "status", "carla_connected": True})
    try:
        while True:
            command = await websocket.receive_json()
            if command.get("type") != "command" or carla_socket is None or not _command_authorized(websocket):
                continue
            try:
                async with carla_send_lock:
                    await asyncio.wait_for(carla_socket.send_json(command), timeout=SEND_TIMEOUT_S)
            except Exception:
                continue
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(websocket)


@app.websocket("/ws/carla_status")
async def browser_status_feed(websocket: WebSocket) -> None:
    await websocket.accept()
    status_clients.add(websocket)
    if latest_scene:
        await websocket.send_json(status_from_scene(latest_scene))
    if carla_socket:
        await websocket.send_json({"type": "carla_status", "carla_connected": True, "connected": True})
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        status_clients.discard(websocket)


@app.websocket("/ws/nexus")
async def nexus_feed(websocket: WebSocket) -> None:
    global latest_targets
    if not _command_authorized(websocket):
        await websocket.close(code=1008, reason="NEXUS_CARLA_TOKEN is required")
        return
    await websocket.accept()
    try:
        while True:
            message = await websocket.receive_json()
            if message.get("type") != "target":
                continue
            latest_targets = [t for t in latest_targets if t.get("target_id") != message.get("target_id")]
            latest_targets.append(message)
            await broadcast(message)
    except WebSocketDisconnect:
        return
