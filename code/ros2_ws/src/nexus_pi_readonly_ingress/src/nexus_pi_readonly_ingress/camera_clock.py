"""Measure Pi sensor-clock to edge Unix time; never set either system clock."""

import json
import socket
import time
import uuid


def estimate_exchange(t1, t2, t3, t4):
    rtt = (t4 - t1) - (t3 - t2)
    if t3 < t2 or t4 < t1 or rtt < 0:
        raise ValueError("invalid clock exchange or clock step")
    return ((t1 - t2) + (t4 - t3)) // 2, rtt


class CameraClockClient:
    def __init__(self, host, port=14553, clock="CLOCK_BOOTTIME"):
        if clock not in {"CLOCK_BOOTTIME", "CLOCK_REALTIME"}:
            raise ValueError("unsupported remote clock")
        self.clock = clock
        self.address = (host, port)
        self.samples = []
        self.boot_id = None

    def probe(self):
        nonce = uuid.uuid4().hex
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
            channel.settimeout(0.15)
            channel.connect(self.address)
            request = json.dumps(dict(nonce=nonce, clock=self.clock)).encode()
            before = time.monotonic_ns()
            t1 = time.time_ns()
            channel.send(request)
            raw = channel.recv(4096)
            t4 = time.time_ns()
            after = time.monotonic_ns()
        if abs((t4 - t1) - (after - before)) > 1_000_000:
            self.samples = []
            raise ValueError("edge Unix clock stepped during exchange")
        response = json.loads(raw)
        if response.get("nonce") != nonce or response.get("clock") != self.clock:
            raise ValueError("clock response mismatch")
        offset, rtt = estimate_exchange(t1, response["t2_ns"], response["t3_ns"], t4)
        if self.boot_id != response["boot_id"]:
            self.samples = []
            self.boot_id = response["boot_id"]
        sample = dict(offset_ns=offset, rtt_ns=rtt, edge_unix_ns=t4,
                      monotonic_ns=after, boot_id=self.boot_id)
        self.samples = [s for s in self.samples if after - s["monotonic_ns"] < 5_000_000_000]
        self.samples.append(sample)
        return sample

    def align(self, sensor_ns, boot_id):
        now = time.monotonic_ns()
        fresh = [s for s in self.samples if now - s["monotonic_ns"] < 5_000_000_000]
        if not fresh or boot_id != self.boot_id:
            raise ValueError("camera clock unsynchronized or Pi rebooted")
        sample = min(fresh, key=lambda s: s["rtt_ns"])
        if abs((time.time_ns() - sample["edge_unix_ns"]) - (now - sample["monotonic_ns"])) > 1_000_000:
            self.samples = []
            raise ValueError("edge Unix clock stepped since probe")
        if not isinstance(sensor_ns, int) or sensor_ns <= 0:
            raise ValueError("missing positive SensorTimestamp")
        metadata = dict(sample)
        metadata["probe_age_ms"] = (now - sample["monotonic_ns"]) / 1e6
        metadata["network_asymmetry_bound_at_probe_ms"] = sample["rtt_ns"] / 2e6
        return sensor_ns + sample["offset_ns"], metadata
