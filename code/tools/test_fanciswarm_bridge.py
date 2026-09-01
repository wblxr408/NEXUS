import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[2] / "tools" / "fanciswarm_bridge.py"
SPEC = importlib.util.spec_from_file_location("fanciswarm_bridge", MODULE_PATH)
bridge_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge_module)


class FakeSerial:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendto(self, data, endpoint):
        self.sent.append((bytes(data), endpoint))


def test_udp_control_datagram_is_forwarded_to_uart_and_endpoint_registered():
    serial = FakeSerial()
    socket = FakeSocket()
    bridge = bridge_module.TransparentBridge(serial, socket, ("192.168.1.140", 14550))

    bridge.handle_udp_datagram(b"control", ("192.168.1.140", 14550))

    assert serial.writes == [b"control"]
    assert ("192.168.1.140", 14550) in bridge.control_endpoints


def test_uart_bytes_fan_out_to_configured_and_registered_udp_endpoints():
    serial = FakeSerial()
    socket = FakeSocket()
    bridge = bridge_module.TransparentBridge(serial, socket, ("192.168.1.140", 14550))
    bridge.control_endpoints.add(("192.168.1.141", 14550))

    bridge.handle_uart_bytes(b"telemetry")

    assert {endpoint for _, endpoint in socket.sent} == {
        ("192.168.1.140", 14550), ("192.168.1.141", 14550)
    }
    assert all(data == b"telemetry" for data, _ in socket.sent)
