"""Measure clock probes; optionally verify live sensor-frame timestamps."""

import argparse
import json
from pathlib import Path
import time

import numpy as np

from nexus_pi_readonly_ingress.camera_clock import CameraClockClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.1.143")
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    clock = CameraClockClient(args.host)
    samples, errors = [], []
    for _ in range(args.samples):
        try:
            samples.append(clock.probe())
        except (ValueError, OSError) as error:
            errors.append(str(error))
        time.sleep(0.5)
    if len(samples) < 10:
        raise RuntimeError(f"insufficient clock exchanges: {len(samples)}, {errors}")
    rtt = np.array([s["rtt_ns"] for s in samples]) / 1e6
    offsets = np.array([s["offset_ns"] - samples[0]["offset_ns"] for s in samples]) / 1e6
    report = dict(samples=samples, errors=errors, rtt_ms_min=float(rtt.min()),
                  rtt_ms_median=float(np.median(rtt)), rtt_ms_p95=float(np.percentile(rtt, 95)),
                  offset_spread_ms=float(np.ptp(offsets)),
                  scope="network_clock_mapping_only_not_camera_IMU_sync",
                  limitation="RTT/2 bounds network asymmetry at each probe; oscillator drift and timestamp read latency are not independently calibrated")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))


if __name__ == "__main__":
    main()
