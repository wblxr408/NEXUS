"""Check analytic manifold derivatives independently, at nonzero residuals."""

import numpy as np
import pytest

from nexus_fusion_localization.imu_preintegration import ImuBuffer, ImuNoise, ImuReading, PreintegratedImu
from nexus_fusion_localization.marginalization import MarginalPrior, numerical_jacobian
from nexus_fusion_localization.platform_measurements import (
    PoseMeasurement, PositionMeasurement, RangeMeasurement, RelativeMotionMeasurement,
)
from nexus_fusion_localization.platform_state import BodyState, exp_so3, right_jacobian_so3
from nexus_fusion_localization.platform_window import PlatformWindow, _ImuFactor


def fixture(seed):
    rng = np.random.default_rng(seed)
    states = {}
    for stamp in (1_000_000_000, 1_100_000_000):
        states[stamp] = BodyState(
            stamp, rng.normal(size=3), rng.normal(size=3), exp_so3(rng.normal(size=3) * .8),
            rng.normal(size=3) * .02, rng.normal(size=3) * .01)
    readings = ImuBuffer()
    for index in range(21):
        readings.add(ImuReading(1_000_000_000 + index * 5_000_000,
                                [1., -.3, 9.7], [.1, -.2, .3]))
    integrated = PreintegratedImu.from_buffer(
        readings, *states, np.zeros(3), np.zeros(3), ImuNoise())
    random_matrix = rng.normal(size=(6, 6))
    covariance = random_matrix @ random_matrix.T + np.eye(6)
    first, second = tuple(states)
    factors = [
        _ImuFactor(integrated, np.array([0., 0., -9.80665]), 2.5),
        PositionMeasurement(second, [1., .4, -.5], covariance[:3, :3], [.3, -.1, .1], 4.),
        PoseMeasurement(second, [1., .5, .3], exp_so3([.3, -.2, .8]), covariance, 2.),
        RangeMeasurement(second, [[-2., -2., 0.], [-2., 2., 0.], [2., -2., 0.], [2., 2., 1.]],
                         [3., 3., 3., 3.], covariance[:4, :4], [.2, .1, -.3]),
        RelativeMotionMeasurement(first, second, exp_so3([.1, .3, -.2]), [1., .2, -.1], covariance,
                                  sensor_body_m=np.array([.2, -.1, .05])),
        RelativeMotionMeasurement(first, second, exp_so3([.1, .3, -.2]), [1., .2, -.1], covariance, True,
                                  sensor_body_m=np.array([.2, -.1, .05])),
    ]
    references = tuple(state.retract(rng.normal(size=15) * .1) for state in states.values())
    prior = MarginalPrior(references, rng.normal(size=(25, 30)), rng.normal(size=25))
    return states, factors, prior


@pytest.mark.parametrize("seed", [4, 17, 39])
def test_each_factor_matches_central_differences_with_bias_and_cross_covariance(seed):
    states, factors, prior = fixture(seed)
    for factor in [prior] + factors:
        _, numerical = numerical_jacobian(factor.residual, states, factor.stamps)
        blocks = factor.jacobians(states)
        analytic = np.hstack([blocks[stamp] for stamp in factor.stamps])
        np.testing.assert_allclose(analytic, numerical, atol=2e-5, rtol=2e-6,
                                   err_msg=type(factor).__name__)


def test_full_graph_jacobian_uses_optimizer_rotation_coordinates_and_local_covariance():
    references, factors, prior = fixture(23)
    weights = [1., .4, .7, 1., .6, .3]
    rng = np.random.default_rng(8)
    delta = rng.normal(size=30) * .25

    def decode(values):
        return {stamp: state.retract(values[index * 15:(index + 1) * 15])
                for index, (stamp, state) in enumerate(references.items())}

    states = decode(delta)
    residual, local = PlatformWindow._linearize(states, prior, factors, weights)
    oracle_residual, oracle_local = numerical_jacobian(
        lambda values: PlatformWindow._stack(values, prior, factors, weights), states)
    np.testing.assert_allclose(residual, oracle_residual)
    np.testing.assert_allclose(local, oracle_local, atol=2e-5, rtol=2e-6)
    optimizer = local.copy()
    for index in range(2):
        angle_slice = slice(index * 15 + 6, index * 15 + 9)
        optimizer[:, angle_slice] = optimizer[:, angle_slice] @ right_jacobian_so3(delta[angle_slice])
    oracle = np.empty_like(optimizer)
    for axis in range(30):
        offset = np.zeros(30)
        offset[axis] = 1e-6
        plus = PlatformWindow._stack(decode(delta + offset), prior, factors, weights)
        minus = PlatformWindow._stack(decode(delta - offset), prior, factors, weights)
        oracle[:, axis] = (plus - minus) / 2e-6
    np.testing.assert_allclose(optimizer, oracle, atol=2e-5, rtol=2e-6)


def test_zero_angle_jacobian_and_anchor_singularity():
    np.testing.assert_array_equal(right_jacobian_so3(np.zeros(3)), np.eye(3))
    states, factors, _ = fixture(3)
    measurement = factors[3]
    state = states[measurement.stamp_ns]
    shift = np.zeros(15)
    shift[:3] = (measurement.anchors_map_m[0] - state.rotation_map_body @ measurement.tag_body_m
                 - state.position_m)
    states[state.stamp_ns] = state.retract(shift)
    with pytest.raises(ValueError, match="at an anchor"):
        measurement.raw_jacobians(states)
