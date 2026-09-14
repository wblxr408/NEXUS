"""Four-timestamp read-only clock service, sharing libcamera CLOCK_BOOTTIME."""

import json
from pathlib import Path
import socket
import threading
import time


class CameraClockServer:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.socket.settimeout(0.2)
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.socket.close()

    def serve(self):
        while not self.stop.is_set():
            try:
                request, address = self.socket.recvfrom(512)
                t2_boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
                t2_unix = time.time_ns()
                value = json.loads(request)
                nonce = value.get("nonce")
                if not isinstance(nonce, str) or len(nonce) != 32:
                    continue
                clock = value.get("clock", "CLOCK_BOOTTIME")
                if clock not in {"CLOCK_BOOTTIME", "CLOCK_REALTIME"}:
                    continue
                t2 = t2_unix if clock == "CLOCK_REALTIME" else t2_boot
                reply = dict(schema_version=1, nonce=nonce, boot_id=self.boot_id,
                             clock=clock, t2_ns=t2)
                reply["t3_ns"] = (time.time_ns() if clock == "CLOCK_REALTIME" else
                                   time.clock_gettime_ns(time.CLOCK_BOOTTIME))
                self.socket.sendto(json.dumps(reply).encode(), address)
            except socket.timeout:
                continue
            except (ValueError, TypeError, AttributeError):
                continue
