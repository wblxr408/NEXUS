"""Platform dynamics and graph tests; synthetic software checks, not flight accuracy."""

import numpy as np
import pytest

from nexus_fusion_localization.imu_preintegration import ImuBuffer, ImuNoise, ImuReading, PreintegratedImu
from nexus_fusion_localization.marginalization import schur_square_root
from nexus_fusion_localization.platform_measurements import (
    PoseMeasurement, PositionMeasurement, RangeMeasurement, RelativeMotionMeasurement,
)
from nexus_fusion_localization.platform_state import (
    BodyState, exp_so3, pose_covariance_from_ros, pose_covariance_to_ros,
)
from nexus_fusion_localization.platform_window import PlatformConfig, PlatformWindow, robust_weight


GRAVITY = np.array([0., 0., -9.80665])


def state(stamp=1_000_000_000, position=(0., 0., 1.), velocity=(0., 0., 0.), rotation=None, ba=None, bg=None):
    return BodyState(stamp, position, velocity, np.eye(3) if rotation is None else rotation,
                     np.zeros(3) if ba is None else ba, np.zeros(3) if bg is None else bg)


def buffer(duration=.6, acceleration=(0., 0., 9.80665), omega=(0., 0., 0.), step=.005):
    result = ImuBuffer()
    for index in range(round(duration / step) + 1):
        result.add(ImuReading(1_000_000_000 + round(index * step * 1e9), acceleration, omega))
    return result


def make_window(size=3, **kwargs):
    window = PlatformWindow(PlatformConfig(window_size=size, kernel="linear", irls_iterations=1, **kwargs))
    window.initialize(state(), np.diag([.01] * 3 + [.04] * 3 + [.01] * 3 + [.01] * 6))
    window.imu = buffer(duration=1.)
    return window


def test_preintegration_stationary_gravity_and_positive_covariance():
    initial = state()
    integrated = PreintegratedImu.from_buffer(
        buffer(), initial.stamp_ns, 1_600_000_000, np.zeros(3), np.zeros(3), ImuNoise())
    predicted = integrated.predict(initial, GRAVITY)
    np.testing.assert_allclose(predicted.position_m, initial.position_m, atol=1e-12)
    np.testing.assert_allclose(predicted.velocity_mps, 0., atol=1e-12)
    np.testing.assert_allclose(integrated.residual(initial, predicted, GRAVITY), 0., atol=1e-10)
    assert np.all(np.linalg.eigvalsh(integrated.covariance) > 0)
    covariance = integrated.predict_covariance(initial, np.eye(15) * .01)
    assert np.all(np.linalg.eigvalsh(covariance) > 0)
    assert covariance[0, 0] > .01


def test_constant_acceleration_and_angular_velocity_have_physical_units():
    initial = state()
    integrated = PreintegratedImu.from_buffer(
        buffer(acceleration=(2., 0., 9.80665)), initial.stamp_ns, 1_500_000_000,
        np.zeros(3), np.zeros(3), ImuNoise())
    predicted = integrated.predict(initial, GRAVITY)
    np.testing.assert_allclose(predicted.position_m, [.25, 0., 1.], atol=1e-10)
    np.testing.assert_allclose(predicted.velocity_mps, [1., 0., 0.], atol=1e-10)
    rotating = PreintegratedImu.from_buffer(
        buffer(omega=(0., 0., 1.)), initial.stamp_ns, 1_500_000_000, np.zeros(3), np.zeros(3), ImuNoise())
    np.testing.assert_allclose(rotating.predict(initial, GRAVITY).rotation_map_body,
                               exp_so3([0., 0., .5]), atol=1e-10)


def test_bias_correction_matches_reintegration_for_small_bias_updates():
    ba, bg = np.array([.01, -.02, .03]), np.array([.001, -.002, .003])
    readings = buffer(acceleration=np.array([0., 0., 9.80665]) + ba, omega=bg)
    cached = PreintegratedImu.from_buffer(
        readings, 1_000_000_000, 1_200_000_000, np.zeros(3), np.zeros(3), ImuNoise())
    recomputed = PreintegratedImu.from_buffer(readings, 1_000_000_000, 1_200_000_000, ba, bg, ImuNoise())
    corrected = cached.predict(state(ba=ba, bg=bg), GRAVITY)
    direct = recomputed.predict(state(ba=ba, bg=bg), GRAVITY)
    np.testing.assert_allclose(corrected.local(direct), 0., atol=3e-6)


def test_imu_boundary_interpolation_never_extrapolates_or_hides_gaps():
    readings = buffer(duration=.2, step=.01)
    selected = readings.interval(1_005_000_000, 1_195_000_000, .02)
    assert selected[0].stamp_ns == 1_005_000_000 and selected[-1].stamp_ns == 1_195_000_000
    with pytest.raises(ValueError, match="bracket"):
        readings.interval(1_000_000_000, 1_300_000_000, .02)
    with pytest.raises(ValueError, match="nonmonotonic"):
        readings.add(readings.samples[-1])
    with pytest.raises(ValueError, match="gap"):
        buffer(duration=.2, step=.1).interval(1_025_000_000, 1_075_000_000, .02)


def test_schur_preserves_mean_and_marginal_covariance_of_linear_problem():
    rng = np.random.default_rng(44)
    jacobian = rng.normal(size=(80, 45))
    residual = rng.normal(size=80)
    matrix, offset, keep = schur_square_root(jacobian, residual, np.arange(15))
    full_mean = -np.linalg.lstsq(jacobian, residual, rcond=None)[0]
    remaining_mean = -np.linalg.lstsq(matrix, offset, rcond=None)[0]
    np.testing.assert_allclose(remaining_mean, full_mean[keep], atol=1e-10)
    covariance = np.linalg.inv(jacobian.T @ jacobian)
    np.testing.assert_allclose(np.linalg.inv(matrix.T @ matrix), covariance[np.ix_(keep, keep)], atol=1e-10)


def test_schur_does_not_invent_information_along_a_gauge_direction():
    matrix, _, _ = schur_square_root(np.array([[1., 1.]]), np.array([0.]), [0])
    assert matrix.shape == (0, 1)


def test_monocular_factor_is_scale_invariant_but_metric_factor_is_not():
    initial, second = state(), state(stamp=1_100_000_000, position=(2., 0., 1.))
    direction = RelativeMotionMeasurement(initial.stamp_ns, second.stamp_ns, np.eye(3), [1., 0., 0.], np.eye(6))
    metric = RelativeMotionMeasurement(initial.stamp_ns, second.stamp_ns, np.eye(3), [1., 0., 0.], np.eye(6), True)
    values = {initial.stamp_ns: initial, second.stamp_ns: second}
    np.testing.assert_allclose(direction.raw_residual(values), 0.)
    assert np.linalg.norm(metric.raw_residual(values)) == 1.


def test_uwb_lever_arm_and_raw_ranges_measure_the_platform_tag():
    body = state(rotation=exp_so3([0, 0, np.pi / 2]))
    lever = np.array([.2, 0., 0.])
    tag = body.position_m + body.rotation_map_body @ lever
    position = PositionMeasurement(body.stamp_ns, tag, np.eye(3), lever)
    np.testing.assert_allclose(position.raw_residual({body.stamp_ns: body}), 0., atol=1e-10)
    anchors = np.array([[-1., -1., 0.], [-1., 1., 0.], [1., -1., 0.], [1., 1., .2]])
    measurement = RangeMeasurement(body.stamp_ns, anchors, np.linalg.norm(anchors - tag, axis=1), np.eye(4), lever)
    np.testing.assert_allclose(measurement.raw_residual({body.stamp_ns: body}), 0.)
    with pytest.raises(ValueError, match="four"):
        RangeMeasurement(body.stamp_ns, anchors[:3], [1., 1., 1.], np.eye(3))


def test_monocular_camera_lever_does_not_turn_body_rotation_into_body_translation():
    initial = state()
    second = state(stamp=1_100_000_000, rotation=exp_so3([0., 0., .5]))
    lever = np.array([.2, .1, .03])
    camera_shift_body_i = second.rotation_map_body @ lever - lever
    observation = RelativeMotionMeasurement(initial.stamp_ns, second.stamp_ns, second.rotation_map_body,
                                            camera_shift_body_i, np.eye(6), sensor_body_m=lever)
    values = {initial.stamp_ns: initial, second.stamp_ns: second}
    np.testing.assert_allclose(observation.raw_residual(values), 0., atol=1e-12)


def test_ros_pose_covariance_round_trip_includes_position_angle_cross_terms():
    body = state(rotation=exp_so3([.2, -.5, .8]))
    rng = np.random.default_rng(2)
    a = rng.normal(size=(15, 15))
    covariance = a @ a.T + np.eye(15)
    ros = pose_covariance_to_ros(body, covariance)
    restored = pose_covariance_from_ros(body.rotation_map_body, ros)
    indices = [0, 1, 2, 6, 7, 8]
    np.testing.assert_allclose(restored, covariance[np.ix_(indices, indices)], atol=1e-12)


def test_window_uwb_jump_rejected_without_moving_platform_and_duplicate_not_reused():
    window = make_window()
    good = PositionMeasurement(1_100_000_000, [0., 0., 1.], np.eye(3) * .0025)
    result = window.step(good.stamp_ns, [good])
    assert result.valid and not result.prediction_only
    before = window.joint_covariance.copy()
    duplicate = window.step(good.stamp_ns, [good])
    assert not duplicate.valid and duplicate.diagnostics[0]["reason"] == "duplicate_measurement"
    np.testing.assert_array_equal(window.joint_covariance, before)
    jump = PositionMeasurement(1_200_000_000, [100., 0., 1.], np.eye(3) * .0025, covariance_scale=100.)
    rejected = window.step(jump.stamp_ns, [jump])
    assert rejected.prediction_only and rejected.diagnostics[0]["decision"] == "rejected"
    assert abs(rejected.state.position_m[0]) < .01


def test_imu_gap_leaves_window_and_prior_unchanged():
    window = make_window()
    before = window.prior
    failed = window.step(2_500_000_000, [])
    assert not failed.valid and "bracket" in failed.reason
    assert window.prior is before and len(window.states) == 1


def test_short_window_matches_batch_information_without_recounting_retained_factors():
    small, batch = make_window(3), make_window(10)
    for index in range(1, 6):
        stamp = 1_000_000_000 + index * 100_000_000
        observation = PositionMeasurement(stamp, [0., 0., 1.], np.eye(3) * .0025)
        limited = small.step(stamp, [observation])
        full = batch.step(stamp, [observation])
        assert limited.valid and full.valid, (limited.reason, full.reason)
    assert len(small.states) == 3 and small.marginalization_count == 3
    np.testing.assert_allclose(limited.state.position_m, full.state.position_m, atol=1e-8)
    np.testing.assert_allclose(limited.covariance[:9, :9], full.covariance[:9, :9], atol=2e-5, rtol=.02)
    assert all(set(factor.stamps).issubset(small.states) for factor in small.factors)


def test_external_constraint_loss_is_explicit_even_when_imu_continues():
    window = make_window(prediction_horizon_s=.2)
    assert window.step(1_100_000_000).valid
    result = window.step(1_400_000_000)
    assert not result.valid and result.prediction_only and result.reason == "external_constraints_lost"


def test_moving_rotating_body_estimates_accelerometer_and_gyro_bias(record_testsuite_property):
    ba, bg = np.array([.04, -.02, .03]), np.array([.001, -.002, .01])
    window = PlatformWindow(PlatformConfig(window_size=4, kernel="huber", max_nfev=80, irls_iterations=2))
    initial = state(velocity=(.3, 0., 0.))
    window.initialize(initial, np.diag([.001] * 9 + [.04] * 6))
    for index in range(161):
        t = index * .005
        rotation = exp_so3([0., 0., .3 * t])
        force = rotation.T @ (np.array([.2, .05, 0.]) - GRAVITY)
        window.add_imu(ImuReading(1_000_000_000 + round(t * 1e9), force + ba, np.array([0., 0., .3]) + bg))
    runtimes = []
    for index in range(1, 9):
        t = index * .1
        stamp = 1_000_000_000 + index * 100_000_000
        position = np.array([.3 * t + .1 * t ** 2, .025 * t ** 2, 1.])
        orientation = exp_so3([0., 0., .3 * t])
        observation = PoseMeasurement(stamp, position, orientation, np.eye(6) * 1e-6)
        result = window.step(stamp, [observation])
        runtimes.append(result.runtime_ms)
        assert result.valid and not result.prediction_only, result.reason
    assert np.linalg.norm(result.state.position_m - position) < .005
    assert np.linalg.norm(result.state.velocity_mps - [.46, .04, 0.]) < .02
    assert np.linalg.norm(result.state.accel_bias_mps2 - ba) < np.linalg.norm(ba) * .5
    assert np.linalg.norm(result.state.gyro_bias_rps - bg) < .002
    record_testsuite_property("synthetic_update_mean_ms", float(np.mean(runtimes)))
    record_testsuite_property("synthetic_update_p95_ms", float(np.percentile(runtimes, 95)))


def test_delayed_observation_inserts_its_time_and_splits_imu_edge():
    chronological, delayed = make_window(6), make_window(6)
    early = PositionMeasurement(1_100_000_000, [0., 0., 1.], np.eye(3) * .0025)
    middle = PositionMeasurement(1_150_000_000, [0., 0., 1.], np.eye(3) * .0025)
    late = PositionMeasurement(1_200_000_000, [0., 0., 1.], np.eye(3) * .0025)
    for item in (early, middle, late):
        assert chronological.step(item.stamp_ns, [item]).valid
    for item in (early, late, middle):
        result = delayed.step(item.stamp_ns, [item])
        assert result.valid, result.reason
    assert tuple(delayed.states) == (1_000_000_000, early.stamp_ns, middle.stamp_ns, late.stamp_ns)
    assert result.state.stamp_ns == late.stamp_ns
    np.testing.assert_allclose(delayed.joint_covariance, chronological.joint_covariance, rtol=.03, atol=2e-5)
    assert delayed.last_source_stamps["uwb_position"] == late.stamp_ns


def test_visual_edge_inserts_both_camera_times_between_uwb_updates():
    window = PlatformWindow(PlatformConfig(window_size=6, kernel="linear", irls_iterations=1))
    initial = state(velocity=(.3, 0., 0.))
    window.initialize(initial, np.eye(15) * .01)
    window.imu = buffer(duration=.3)
    uwb = PositionMeasurement(1_200_000_000, [.06, 0., 1.], np.eye(3) * .0025)
    assert window.step(uwb.stamp_ns, [uwb]).valid
    edge = RelativeMotionMeasurement(1_070_000_000, 1_170_000_000, np.eye(3), [1., 0., 0.], np.eye(6) * .01)
    result = window.step(edge.stamp_ns, [edge])
    assert result.valid and not result.prediction_only, result.reason
    assert result.diagnostics[0]["decision"] == "accepted"
    assert result.state.stamp_ns == uwb.stamp_ns
    assert tuple(window.states) == (initial.stamp_ns, edge.previous_stamp_ns, edge.stamp_ns, uwb.stamp_ns)
    np.testing.assert_allclose(window.states[edge.previous_stamp_ns].position_m, [.021, 0., 1.], atol=1e-8)
    assert all(set(factor.stamps).issubset(window.states) for factor in window.factors)


def test_bad_units_are_rejected_before_imu_can_contaminate_the_buffer():
    window = make_window()
    count = len(window.imu.samples)
    with pytest.raises(ValueError, match="SI units"):
        window.add_imu(ImuReading(2_005_000_000, [0., 0., 9806.65], [0., 0., 0.]))
    assert len(window.imu.samples) == count


def test_default_window_tracks_through_tag_loss_and_rejects_uwb_jump(record_testsuite_property):
    window = PlatformWindow()  # Exercise the deployed six-state/two-IRLS default.
    velocity = np.array([.3, .05, .01])
    initial = state(velocity=velocity)
    window.initialize(initial, np.eye(15) * .01)
    for index in range(301):
        t = index * .005
        rotation = exp_so3([0., 0., .2 * t])
        window.add_imu(ImuReading(initial.stamp_ns + round(t * 1e9),
                                  rotation.T @ -GRAVITY, [0., 0., .2]))
    runtimes = []
    saw_jump = False
    for index in range(1, 31):
        t = index * .05
        stamp = initial.stamp_ns + index * 50_000_000
        previous_stamp = stamp - 50_000_000
        previous_rotation = exp_so3([0., 0., .2 * (t - .05)])
        rotation = exp_so3([0., 0., .2 * t])
        position = initial.position_m + velocity * t
        observations = [RelativeMotionMeasurement(previous_stamp, stamp,
                                                  previous_rotation.T @ rotation,
                                                  previous_rotation.T @ velocity * .05,
                                                  np.diag([.01] * 3 + [.001] * 3))]
        if index % 2 == 0:
            uwb_position = position + (np.array([4., -2., 0.]) if index == 16 else 0.)
            observations.append(PositionMeasurement(stamp, uwb_position, np.eye(3) * .0025))
        if index % 5 == 0 and not .4 <= t <= 1.1:
            observations.append(PoseMeasurement(stamp, position, rotation, np.eye(6) * .0001))
        result = window.step(stamp, observations)
        assert result.valid and not result.prediction_only, result.reason
        assert len(window.states) <= 6
        assert np.all(np.linalg.eigvalsh(result.covariance) > 0)
        assert np.linalg.norm(result.state.position_m - position) < .005
        if index == 16:
            saw_jump = any(item["source"] == "uwb_position" and item["decision"] == "rejected"
                           for item in result.diagnostics)
        runtimes.append(result.runtime_ms)
    assert saw_jump and window.last_source_stamps["fixed_tag"] == stamp
    assert window.marginalization_count == 25
    record_testsuite_property("synthetic_default_window_mean_ms", float(np.mean(runtimes)))
    record_testsuite_property("synthetic_default_window_p95_ms", float(np.percentile(runtimes, 95)))


@pytest.mark.parametrize("kernel", ["huber", "cauchy", "tukey", "switchable"])
def test_robust_kernels_downweight_large_factor_blocks(kernel):
    assert robust_weight(0., kernel, 3.) == 1.
    assert 0 <= robust_weight(10., kernel, 3.) < robust_weight(1., kernel, 3.) <= 1.
