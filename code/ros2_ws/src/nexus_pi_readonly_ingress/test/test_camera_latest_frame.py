import json
import struct
from types import SimpleNamespace

from nexus_pi_readonly_ingress.camera_file_node import CameraFileNode


def packet(sequence):
    metadata = json.dumps({"frame_sequence": sequence}).encode()
    return struct.pack("!II", len(metadata), 3) + metadata + b"jpg"


def test_latest_complete_frame_is_published_and_partial_packet_retained():
    pending = packet(3)
    chunks = [packet(1) + packet(2) + pending[:10]]

    class Socket:
        def recv(self, size):
            if not chunks:
                raise BlockingIOError()
            return chunks.pop(0)

    received = []
    harness = SimpleNamespace(socket=Socket(), buffer=b"", connect_tcp=lambda: None,
                              publish_frame=lambda metadata, image: received.append(metadata["frame_sequence"]))
    CameraFileNode.poll_tcp(harness)
    assert received == [2]
    assert harness.buffer == pending[:10]
    chunks.append(pending[10:])
    CameraFileNode.poll_tcp(harness)
    assert received == [2, 3]
    assert harness.buffer == b""


def test_command_gates_camera_connection_until_takeoff():
    events = []

    class Logger:
        def info(self, message):
            events.append(message)

    class Socket:
        def close(self):
            events.append("closed")

    harness = SimpleNamespace(
        capture_enabled=False,
        start_command=3,
        stop_commands={2, 4},
        next_connect_ns=99,
        socket=None,
        buffer=b"stale",
        get_logger=lambda: Logger(),
    )

    CameraFileNode.command_callback(harness, SimpleNamespace(data=3))
    assert harness.capture_enabled is True
    assert harness.next_connect_ns == 0

    harness.socket = Socket()
    CameraFileNode.command_callback(harness, SimpleNamespace(data=4))
    assert harness.capture_enabled is False
    assert harness.socket is None
    assert harness.buffer == b""
    assert "closed" in events


def test_poll_is_silent_before_takeoff_command():
    called = []
    harness = SimpleNamespace(
        capture_enabled=False,
        transport="tcp_client",
        poll_tcp=lambda: called.append("tcp"),
    )
    CameraFileNode.poll(harness)
    assert called == []
