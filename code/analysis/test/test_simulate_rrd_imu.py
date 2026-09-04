import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from localization.simulate_rrd_imu import motion_samples, sensor_samples, validated_motion_samples


def test_specific_force_stationary_rotated_sensor_and_right_handed_pose():
    times = np.arange(5, dtype=float)
    r = Rotation.from_euler('x', 90, degrees=True).as_matrix()
    motion = motion_samples(times, np.zeros((5, 3)), np.tile(r, (5, 1, 1)), 100)
    assert np.allclose(motion['specific_force'], [0, 9.80665, 0], atol=1e-10)
    assert np.allclose(motion['angular_rate'], 0, atol=1e-10)
    with pytest.raises(ValueError, match='proper rotations'):
        motion_samples(times, np.zeros((5, 3)), np.tile(np.diag([1, -1, 1]), (5, 1, 1)), 100)


def test_accelerating_translating_and_rotating_motion_uses_trajectory():
    times = np.arange(0, 2.01, .1)
    positions = np.column_stack([.5 * .7 * times ** 2, times * 1.3, times * 0])
    rotations = Rotation.from_euler('z', times * .4).as_matrix()
    motion = motion_samples(times, positions, rotations, 188.9)
    assert np.allclose(motion['acceleration'], [.7, 0, 0], atol=1e-10)
    assert np.allclose(motion['angular_rate'], [0, 0, .4], atol=1e-7)
    world_force = np.einsum('nij,nj->ni', motion['rotation'], motion['specific_force'])
    assert np.allclose(world_force, [.7, 0, 9.80665], atol=1e-10)
    assert np.allclose(np.diff(motion['time']), 1 / 188.9)


def test_noise_seed_density_and_ideal_mode():
    times = np.arange(0., 101., 1.)
    motion = motion_samples(times, np.zeros((101, 3)), np.tile(np.eye(3), (101, 1, 1)), 100)
    param = {'noise_density': .02, 'bias_random_walk_density': 0., 'initial_bias': [0, 0, .1], 'quantization_step': 0.}
    profile = {'rate_hz': 100, 'sensor_assumptions': {'rotation_body_imu': np.eye(3),
               'lever_arm_body_m': [0, 0, 0], 'accel': param, 'gyro': param}}
    measured, _, variances = sensor_samples(motion, profile, 12)
    repeated, _, _ = sensor_samples(motion, profile, 12)
    assert np.array_equal(measured['accel'], repeated['accel'])
    residual = measured['accel'] - motion['specific_force']
    assert np.allclose(residual.mean(axis=0), [0, 0, .1], atol=.008)
    assert np.allclose(residual.var(axis=0), variances['accel'], rtol=.04)
    ideal, bias, variance = sensor_samples(motion, profile, 12, ideal=True)
    assert np.array_equal(ideal['accel'], motion['specific_force'])
    assert not np.any(bias['accel']) and not np.any(variance['accel'])


def test_held_then_jumped_pose_is_not_a_valid_acceleration_spike():
    times = np.arange(0., 2.01, .05)
    positions = np.column_stack([times, 0 * times, 0 * times])
    positions[20] = positions[19]
    rotations = np.tile(np.eye(3), (len(times), 1, 1))
    motion, held, raw_peak = validated_motion_samples(times, positions, rotations, 188.9)
    assert held == [20] and raw_peak > 10
    assert not motion['valid'][(motion['time'] >= times[19]) & (motion['time'] <= times[21])].any()
    assert np.allclose(motion['acceleration'][motion['valid']], 0, atol=1e-9)
    stationary, held, _ = validated_motion_samples(times, 0 * positions, rotations, 188.9)
    assert not held and stationary['valid'].all()
