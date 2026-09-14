"""Numerical checks for experimental summaries, using explicitly synthetic data."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "platform_acceptance", Path(__file__).parents[1] / "uwb" / "platform_acceptance.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_allan_white_rate_noise_decreases_as_sqrt_tau():
    rng = np.random.default_rng(729)
    values = rng.normal(0, 2, (40000, 3))
    result = module.overlapping_allan(values, .01)
    for row in result[:10]:
        expected = 2 / np.sqrt(row["tau_s"] / .01)
        np.testing.assert_allclose(row["deviation"], expected, rtol=.1)


def test_clock_drift_and_vendor_si_conversion(tmp_path):
    rows = []
    for i in range(200):
        rows.append(dict(receive_unix_ns=1_700_000_000_000_000_000 + i * 5_001_000,
                         message=dict(mavpackettype="SCALED_IMU", time_boot_ms=1000 + i * 5,
                                      xacc=0, yacc=0, zacc=-9800, xgyro=0, ygyro=-100, zgyro=0)))
    p = tmp_path / "synthetic.jsonl"
    p.write_text("".join(json.dumps(row) + "\n" for row in rows))
    result = module.analyze(p, "synthetic_rotation")
    assert result["imu"]["relative_clock_drift_ppm"] == pytest.approx(200)
    np.testing.assert_allclose(result["imu"]["accel_mean_mps2"], [0, 0, 9.8])
    np.testing.assert_allclose(result["imu"]["gyro_integral_rad"], [0, .0995, 0])
    assert "allan_uniform_resampled" not in result["imu"]
    rows[100]["message"]["time_boot_ms"] = rows[99]["message"]["time_boot_ms"]
    p.write_text("".join(json.dumps(row) + "\n" for row in rows))
    with pytest.raises(ValueError, match="duplicate/reset"):
        module.analyze(p, "synthetic_rotation")


def test_motion_timing_recovers_known_synthetic_attitude_delay(tmp_path):
    rows = []
    for i in range(1000):
        t = i * .005
        common = dict(role="axis_yaw", receive_unix_ns=1_700_000_000_000_000_000 + i * 5_000_000)
        rows.append(dict(common, message=dict(mavpackettype="SCALED_IMU", time_boot_ms=1000 + i * 5,
                                             xgyro=0, ygyro=0, zgyro=-1000 * np.sin(2 * np.pi * .4 * t))))
        if i % 20 == 0:
            yaw = (1 - np.cos(2 * np.pi * .4 * (t - .04))) / (2 * np.pi * .4)
            rows.append(dict(common, message=dict(mavpackettype="GLOBAL_VISION_POSITION_ESTIMATE",
                                                 usec=1000 + i * 5, roll=0, pitch=0, yaw=-yaw)))
    source, stationary, output = (tmp_path / name for name in ("synthetic.jsonl", "bias.json", "result.json"))
    source.write_text("".join(json.dumps(r) + "\n" for r in rows))
    stationary.write_text(json.dumps({"imu": {"gyro_mean_rps": [0, 0, 0]}}))
    script = Path(__file__).parents[1] / "uwb" / "analyze_platform_motion.py"
    subprocess.run([sys.executable, str(script), "--input", str(source), "--stationary-summary",
                    str(stationary), "--output", str(output)], check=True)
    result = json.loads(output.read_text())["phases"]["axis_yaw"]["imu_vs_vendor_attitude"][2]
    assert result["correlation"] > .99
    assert result["pose_lag_relative_to_imu_s"] == pytest.approx(.04, abs=.01)
