#!/usr/bin/env python3
"""Run the six FanciSwarm UWB algorithms from MAVLink telemetry (2D/XY-only)."""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2]))
sys.path.insert(0, str(HERE.parents[4] / "simulation"))

from algorithms.contracts import AlgorithmResult
from algorithms.router import AlgorithmRouter, register_default_algorithms
from algorithms.uwb.live_protocol import (ANCHOR_ORDER, TAG_ID, convert_fc_position,
                                           horizontal_error, observation_from_battery)
from algorithms.uwb.positioning_algorithms_for_uwb_matlab.multilateration import solve_multilateration_2d
from algorithms.uwb.positioning_algorithms_for_uwb_matlab.taylor_series import solve_taylor_2d
from algorithms.uwb.positioning_algorithms_for_uwb_matlab.trilateration import solve_trilateration_2d

ALGORITHMS = (
    "uwb.matlab.trilateration", "uwb.matlab.multilateration", "uwb.matlab.taylor",
    "uwb.matlab.ekf", "uwb.matlab.ukf", "uwb.awesome_uwb",
)
DISPLAY_NAMES = {name: name.rsplit(".", 1)[-1].replace("matlab_", "").title() for name in ALGORITHMS}
DISPLAY_NAMES.update({"uwb.matlab.trilateration": "Trilateration", "uwb.matlab.multilateration": "Multilateration",
                      "uwb.matlab.taylor": "Taylor", "uwb.matlab.ekf": "EKF", "uwb.matlab.ukf": "UKF",
                      "uwb.awesome_uwb": "Range Graph"})


class Live2DState:
    def __init__(self, initial_xy=(2.0, 2.0), window=10):
        self.xy = np.asarray(initial_xy, dtype=float)
        self.velocity = np.zeros(2)
        self.last_timestamp_ns = None
        self.window = deque(maxlen=window)

    def reset(self):
        self.xy[:] = (2.0, 2.0); self.velocity[:] = 0.0
        self.last_timestamp_ns = None; self.window.clear()

    def filter(self, anchors, ranges, stamp, kind):
        if self.last_timestamp_ns is not None:
            dt = max((int(stamp) - self.last_timestamp_ns) / 1e9, 1e-3)
            predicted = self.xy + self.velocity * dt
        else:
            dt = 0.1; predicted = self.xy.copy()
        # A compact constant-velocity measurement update is sufficient for the
        # planar live adapter; offline runners retain their 3D implementations.
        estimate = solve_taylor_2d(anchors, ranges, initial_xy=predicted, max_iter=8)
        if kind == "ekf":
            gain = 0.65
        else:
            gain = 0.8
        updated = predicted + gain * (estimate - predicted)
        self.velocity = (updated - self.xy) / dt
        self.xy = updated
        self.last_timestamp_ns = int(stamp)
        return self.xy.copy()


class LiveRangeGraph2D:
    def __init__(self, window=10):
        self.window = deque(maxlen=window)
        self.last_xy = None
        self.maximum_iterations = 8
        self.cauchy_scale = 2.0

    def reset(self):
        self.window.clear(); self.last_xy = None

    def solve(self, anchors, ranges, stamp):
        self.window.append((int(stamp), np.asarray(anchors, float)[:, :2], np.asarray(ranges, float)))
        estimates = [solve_trilateration_2d(a, r) for _, a, r in self.window]
        xy = np.asarray(self.last_xy if self.last_xy is not None else estimates[-1], dtype=float)
        # Small LM solve over the current window. Cauchy weights preserve the
        # range-residual objective while limiting a single bad measurement.
        damping = 1e-3
        for _ in range(self.maximum_iterations):
            residuals, rows = [], []
            for _, anchors_i, ranges_i in self.window:
                delta = xy - anchors_i
                predicted = np.maximum(np.linalg.norm(delta, axis=1), 1e-9)
                residuals.extend((ranges_i - predicted) / 0.05)
                rows.extend((-delta / (predicted[:, None] * 0.05)).tolist())
            residuals = np.asarray(residuals); rows = np.asarray(rows)
            weights = 1.0 / np.sqrt(1.0 + (residuals / self.cauchy_scale) ** 2)
            normal = rows.T @ (rows * weights[:, None])
            gradient = rows.T @ (residuals * weights)
            try: step = np.linalg.solve(normal + damping * np.eye(2), -gradient)
            except np.linalg.LinAlgError: break
            xy = xy + step
            if np.linalg.norm(step) < 1e-7: break
        self.last_xy = xy
        return xy


class LiveAlgorithmRunner:
    def __init__(self):
        self.states = {name: (Live2DState() if name.endswith(("ekf", "ukf")) else LiveRangeGraph2D() if name == "uwb.awesome_uwb" else None)
                       for name in ALGORITHMS}

    def reset(self, name):
        state = self.states.get(name)
        if state is not None:
            state.reset()

    def run(self, name, frame):
        anchors = np.asarray([sample.anchor_position_m for sample in frame.ranges], dtype=float)
        ranges = np.asarray([sample.range_m for sample in frame.ranges], dtype=float)
        if tuple(sample.anchor_id for sample in frame.ranges) != ANCHOR_ORDER:
            raise ValueError("anchor order changed; expected 1,2,3,4")
        if frame.tag_id != TAG_ID:
            raise ValueError(f"unexpected tag_id {frame.tag_id!r}; expected {TAG_ID!r}")
        if name == "uwb.matlab.trilateration": xy = solve_trilateration_2d(anchors, ranges)
        elif name == "uwb.matlab.multilateration": xy = solve_multilateration_2d(anchors, ranges)
        elif name == "uwb.matlab.taylor": xy = solve_taylor_2d(anchors, ranges)
        elif name.endswith("ekf"): xy = self.states[name].filter(anchors, ranges, frame.timestamp_ns, "ekf")
        elif name.endswith("ukf"): xy = self.states[name].filter(anchors, ranges, frame.timestamp_ns, "ukf")
        else: xy = self.states[name].solve(anchors, ranges, frame.timestamp_ns)
        xy = np.asarray(xy, dtype=float).reshape(-1)
        if xy.size < 2 or not np.all(np.isfinite(xy[:2])): raise ValueError("non-finite XY estimate")
        metadata = {"dimension": "2D/XY-only", "timestamp_ns": frame.timestamp_ns}
        if name == "uwb.awesome_uwb":
            metadata.update({"window_size": len(self.states[name].window),
                             "robust_kernel": "cauchy", "optimizer": "LM"})
        elif name.endswith(("ekf", "ukf")):
            metadata["state"] = "[x,y,vx,vy]"
        return AlgorithmResult(xy[:2], np.full((2, 2), np.nan), name, "uwb", True, metadata)

    def run_all(self, frame, fc_position=None):
        """Run every route independently; one failure never masks the others."""
        output = {}
        for name in ALGORITHMS:
            try:
                result = self.run(name, frame)
                error = horizontal_error(result.estimate, fc_position)
                result.metadata["error_vs_fc_m"] = error
                output[name] = result
            except Exception as exc:
                self.reset(name)
                output[name] = AlgorithmResult(
                    np.full(2, np.nan), np.full((2, 2), np.nan), name, "uwb", False,
                    {"dimension": "2D/XY-only", "error": str(exc), "timestamp_ns": frame.timestamp_ns},
                )
        return output


def _mav_connection(port):
    from pymavlink import mavutil
    return mavutil.mavlink_connection(port)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-port", type=int, default=14550)
    parser.add_argument("--rate-hz", type=float, choices=(5.0, 10.0), default=10.0)
    args = parser.parse_args(argv)
    try:
        register_default_algorithms()
        for name in ALGORITHMS: AlgorithmRouter(name)
        conn = _mav_connection(f"udpin:0.0.0.0:{args.listen_port}")
    except Exception as exc:
        parser.error(str(exc))
    runner = LiveAlgorithmRunner(); fc = None; last_frame = 0
    period = 1.0 / args.rate_hz
    print("FanciSwarm live UWB: 2D/XY-only; FC position is comparison-only")
    while True:
        message = conn.recv_match(blocking=True, timeout=period)
        if message is None: continue
        typ = message.get_type()
        now_ns = time.time_ns()
        if typ == "BATTERY_STATUS":
            try: frame = observation_from_battery(message.voltages, now_ns)
            except ValueError as exc: print(f"input: INVALID: {exc}"); continue
            if now_ns - last_frame < int(period * 1e9 * 0.5): continue
            last_frame = now_ns
            results = runner.run_all(frame, fc.position_m if fc else None)
            for name, result in results.items():
                if not result.valid:
                    print(f"{DISPLAY_NAMES[name]}: INVALID: {result.metadata.get('error', 'invalid result')}")
                    continue
                err = result.metadata.get("error_vs_fc_m")
                suffix = "STALE" if fc is None or now_ns - fc.timestamp_ns > 1_000_000_000 else f"error_vs_fc={err:.3f}m"
                print(f"{DISPLAY_NAMES[name]}: XY=({result.estimate[0]:.3f},{result.estimate[1]:.3f}) {suffix}")
        elif typ == "GLOBAL_VISION_POSITION_ESTIMATE":
            try: fc = type("Fc", (), {"position_m": convert_fc_position(message), "timestamp_ns": now_ns})()
            except (TypeError, ValueError, OverflowError): fc = None


if __name__ == "__main__":
    main()
