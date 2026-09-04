"""Generate explicitly synthetic IMU observations from recorded CARLA poses.

Recorded platform poses supply motion, never vehicle commands. Hardware notes
constrain rate and wire units only; unmeasured noise/mount parameters remain
declared assumptions. Ground truth is stored separately from IMU observations.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, RotationSpline
import yaml

from .rrd_episode import CARLA_TO_MAP, carla_rotation_matrix, location_to_map, read_jsonl


def motion_samples(times, positions, rotations, rate_hz, gravity_mps2=9.80665, sample_times=None):
    times, positions, rotations = map(np.asarray, (times, positions, rotations))
    if (len(times) < 4 or positions.shape != (len(times), 3)
            or rotations.shape != (len(times), 3, 3)
            or not all(np.isfinite(x).all() for x in (times, positions, rotations))
            or np.any(np.diff(times) <= 0) or not np.isfinite(rate_hz) or rate_hz <= 0):
        raise ValueError("invalid pose sequence or sample rate")
    if (not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-6)
            or not np.allclose(np.linalg.det(rotations), 1., atol=1e-6)):
        raise ValueError("poses must contain right-handed proper rotations")
    if sample_times is None:
        count = int(np.floor((times[-1] - times[0]) * rate_hz)) + 1
        sample_times = times[0] + np.arange(count) / rate_hz
    translation = CubicSpline(times, positions, axis=0, extrapolate=False)
    orientation = RotationSpline(times, Rotation.from_matrix(rotations))
    attitudes = orientation(sample_times)
    acceleration = translation(sample_times, 2)
    gravity = np.array([0., 0., -gravity_mps2])
    specific_force = attitudes.inv().apply(acceleration - gravity)
    # R(t-h)^T R(t+h) gives body angular rate. A small symmetric difference
    # also handles endpoints with one-sided intervals, without extrapolation.
    epsilon = min(1e-4, .01 / rate_hz)
    left = np.maximum(times[0], sample_times - epsilon)
    right = np.minimum(times[-1], sample_times + epsilon)
    angular_rate = (orientation(left).inv() * orientation(right)).as_rotvec() / (right - left)[:, None]
    return {"time": sample_times, "position": translation(sample_times),
            "velocity": translation(sample_times, 1), "acceleration": acceleration,
            "rotation": attitudes.as_matrix(), "specific_force": specific_force, "angular_rate": angular_rate}


def validated_motion_samples(times, positions, rotations, rate_hz, gravity_mps2=9.80665):
    """Do not interpret held-then-jumped simulator poses as physical impulses.

    Exclude the held pose and its two neighbours; interpolate each remaining
    contiguous segment separately. Actual stationary sequences remain valid.
    This conservative rule flags ambiguous stop/restart intervals too.
    """
    positions, rotations = np.asarray(positions), np.asarray(rotations)
    raw = motion_samples(times, positions, rotations, rate_hz, gravity_mps2)
    moving = (np.linalg.norm(np.diff(positions, axis=0), axis=1) > 1e-9
              ) | np.any(np.abs(np.diff(rotations, axis=0)) > 1e-9, axis=(1, 2))
    held = [i for i in range(1, len(times)) if not moving[i - 1]
            and ((i > 1 and moving[i - 2]) or (i < len(times) - 1 and moving[i]))]
    pose_valid = np.ones(len(times), dtype=bool)
    for i in held:
        pose_valid[max(0, i - 1):min(len(times), i + 2)] = False
    valid = np.zeros(len(raw['time']), dtype=bool)
    segments = np.full(len(raw['time']), -1, dtype=int)
    indices = np.flatnonzero(pose_valid)
    blocks = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    raw_peak = float(np.max(np.linalg.norm(raw['acceleration'], axis=1)))
    for segment_id, block in enumerate(blocks):
        if len(block) < 4:
            continue
        select = (raw['time'] >= times[block[0]]) & (raw['time'] <= times[block[-1]])
        if not np.any(select):
            continue
        part = motion_samples(np.asarray(times)[block], positions[block], rotations[block], rate_hz,
                              gravity_mps2, raw['time'][select])
        for key in raw:
            raw[key][select] = part[key]
        valid[select] = True
        segments[select] = segment_id
    if not np.any(valid):
        raise ValueError('no continuous pose segment with four knots')
    raw.update(valid=valid, segment_id=segments)
    return raw, held, raw_peak


def sensor_samples(motion, profile, seed, ideal=False):
    sensor = profile["sensor_assumptions"]
    mounting = np.asarray(sensor["rotation_body_imu"], dtype=float)
    if (mounting.shape != (3, 3) or not np.allclose(mounting.T @ mounting, np.eye(3))
            or not np.isclose(np.linalg.det(mounting), 1)):
        raise ValueError("invalid virtual IMU mounting")
    # IMU is at body origin; translated mounts require rotational lever-arm terms.
    if not np.allclose(sensor["lever_arm_body_m"], 0.):
        raise ValueError("this simulator requires a body-origin virtual IMU")
    n = len(motion["time"])
    dt = 1. / profile["rate_hz"]
    rng = np.random.default_rng(seed)
    measured, biases, variances = {}, {}, {}
    for name, truth, key in (("accel", motion["specific_force"], "accel"),
                             ("gyro", motion["angular_rate"], "gyro")):
        params = sensor[key]
        density = np.broadcast_to(params["noise_density"], (3,)).astype(float)
        walk = np.broadcast_to(params["bias_random_walk_density"], (3,)).astype(float)
        bias = np.broadcast_to(params["initial_bias"], (3,)).astype(float)
        quantization = float(params["quantization_step"])
        if (not np.isfinite(np.r_[density, walk, bias, quantization]).all()
                or np.any(density < 0) or np.any(walk < 0) or quantization < 0):
            raise ValueError("invalid noise assumptions")
        history = np.zeros((n, 3)) if ideal else np.tile(bias, (n, 1))
        if not ideal and n > 1:
            history[1:] += np.cumsum(rng.normal(size=(n - 1, 3)) * walk * np.sqrt(dt), axis=0)
        value = truth @ mounting + history
        variance = np.zeros(3) if ideal else density ** 2 / dt
        if not ideal:
            value += rng.normal(size=(n, 3)) * np.sqrt(variance)
            if quantization:
                value = np.round(value / quantization) * quantization
        measured[name], biases[name], variances[name] = value, history, variance
    return measured, biases, variances


def generate(episode, profile_path, output, seed=20260904, ideal=False):
    episode, profile_path, output = map(Path, (episode, profile_path, output))
    if output.exists():
        raise FileExistsError(output)
    profile = yaml.safe_load(profile_path.read_text())
    if profile["source_kind"] != "hardware_rate_with_assumed_noise":
        raise ValueError("unmeasured noise must be explicitly identified")
    rows = read_jsonl(episode / "ground_truth/uav_pose.jsonl")
    times = np.array([r["simulation_elapsed"] for r in rows])
    positions = np.array([location_to_map(r["uav_platform_pose"]["location"]) for r in rows])
    rotations = []
    for row in rows:
        angles = row["uav_platform_pose"]["rotation"]
        carla = carla_rotation_matrix(angles["pitch"], angles["yaw"], angles["roll"])
        rotations.append(CARLA_TO_MAP @ carla @ CARLA_TO_MAP)
    motion, held, raw_peak = validated_motion_samples(
        times, positions, np.array(rotations), profile["rate_hz"], profile["gravity_mps2"])
    measured, biases, variances = sensor_samples(motion, profile, seed, ideal)
    output.mkdir(parents=True, exist_ok=False)
    (output / "ground_truth").mkdir()
    mounting = np.array(profile["sensor_assumptions"]["rotation_body_imu"])
    with (output / "imu.jsonl").open("w") as observations, (output / "ground_truth/imu_motion.jsonl").open("w") as truth:
        for i, time in enumerate(motion["time"]):
            valid = bool(motion['valid'][i])
            common = {"sequence": i, "simulation_elapsed": float(time), "sample_timestamp_ns": int(round(time * 1e9)),
                      "source_kind": "SIMULATION", "frame_id": "imu_sim_link", "valid": valid,
                      "segment_id": int(motion['segment_id'][i]),
                      "invalid_reason": '' if valid else 'source_pose_hold_jump_or_short_segment'}
            observations.write(json.dumps({**common, "linear_acceleration_mps2": measured["accel"][i].tolist() if valid else None,
                                           "angular_velocity_rps": measured["gyro"][i].tolist() if valid else None,
                                           "accel_white_noise_covariance": np.diag(variances["accel"]).tolist(),
                                           "gyro_white_noise_covariance": np.diag(variances["gyro"]).tolist(),
                                           "orientation_available": False}) + "\n")
            truth.write(json.dumps({**common, "position_map_m": motion["position"][i].tolist(),
                                    "velocity_map_mps": motion["velocity"][i].tolist(),
                                    "acceleration_map_mps2": motion["acceleration"][i].tolist(),
                                    "quaternion_map_body_xyzw": Rotation.from_matrix(motion["rotation"][i]).as_quat().tolist(),
                                    "specific_force_imu_mps2": (motion["specific_force"][i] @ mounting).tolist(),
                                    "angular_velocity_imu_rps": (motion["angular_rate"][i] @ mounting).tolist(),
                                    "accel_bias_mps2": biases["accel"][i].tolist(),
                                    "gyro_bias_rps": biases["gyro"][i].tolist()}) + "\n")
    copied = output / "profile.yaml"
    copied.write_bytes(profile_path.read_bytes())
    summary = {"source_kind": "SIMULATION", "mode": "ideal" if ideal else "assumed_noise",
               "hardware_noise_calibrated": False, "sensor_mount_measured": False,
               "rate_hz": profile["rate_hz"], "samples": len(motion["time"]), "seed": seed,
               "valid_samples": int(motion['valid'].sum()), "invalid_samples": int((~motion['valid']).sum()),
               "possible_held_pose_frames": [rows[i]['frame'] for i in held],
               "invalid_interval_policy": "exclude held pose and neighbours; no interpolation across resulting gaps",
               "raw_global_spline_peak_acceleration_mps2": raw_peak,
               "episode_id": rows[0]["episode_id"], "time_range_s": [float(motion["time"][0]), float(motion["time"][-1])],
               "position_interpolation": "CubicSpline of stored poses; derived velocity/acceleration",
               "rotation_interpolation": "RotationSpline; angular rate from relative rotations",
               "frames": "CARLA world/body y flipped to right-handed map/body; virtual body-origin IMU",
               "specific_force": "R_map_imu.T @ (a_map - gravity_map)",
               "noise_convention": "per-sample sigma = density / sqrt(dt); bias increments = walk_density * sqrt(dt)",
               "covariance_scope": "white noise only; excludes bias and quantization",
               "motion_accel_max_mps2": float(np.max(np.linalg.norm(motion["acceleration"][motion['valid']], axis=1))),
               "angular_rate_max_rps": float(np.max(np.linalg.norm(motion["angular_rate"][motion['valid']], axis=1))),
               "limitations": ["No measured noise density or mounting available", "No motor vibration, thermal drift or real clock jitter",
                               "Interpolating 20 Hz poses does not recover missing high-frequency motion",
                               "Ground-truth sidecar must not be used as estimator input or learned input features"],
               "source_pose_sha256": hashlib.sha256((episode / "ground_truth/uav_pose.jsonl").read_bytes()).hexdigest()}
    summary["file_sha256"] = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in output.rglob("*") if p.is_file()}
    (output / "manifest.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260904)
    parser.add_argument("--ideal", action="store_true")
    args = parser.parse_args()
    generate(args.episode, args.profile, args.output, args.seed, args.ideal)
