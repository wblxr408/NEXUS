from dataclasses import replace

import numpy as np
import pytest

from nexus_fusion_localization.fusion import Observation
from nexus_fusion_localization.robust_filter import (
    RobustFilterConfig, RobustTargetFilter, innovation_gate,
)


def sample(t, position=(0, 0, 0), source=2, target="target", variance=.001):
    stamp = int(1e9 + t * 1e9)
    return Observation(target, "map", stamp, np.array(position, dtype=float),
                       np.eye(3) * variance, .9, source, np.array([0., 0., 0., 1.]),
                       stamp + 1)


def update(estimator, item, **kwargs):
    return estimator.update(item, item.stamp_ns + 2, **kwargs)


def test_mahalanobis_uses_correlated_covariance_and_three_decisions():
    covariance = np.array([[2., 1.], [1., 2.]])
    normal = innovation_gate([1., 1.], covariance, 1., 3.)
    assert normal[0] == "accepted" and normal[1] == pytest.approx(2 / 3)
    assert innovation_gate([2., 2.], covariance, 1., 3.)[0] == "suspect"
    assert innovation_gate([3., 3.], covariance, 1., 3.)[0] == "rejected"


def test_constant_motion_survives_outlier_without_contaminating_state():
    estimator = RobustTargetFilter()
    for i in range(12):
        t = i * .1
        assert update(estimator, sample(t, (t, 0, 0))) is not None
    before = estimator.tracks["target"].x.copy()
    assert update(estimator, sample(1.2, (100, 0, 0))) is None
    assert estimator.last_diagnostic["reason"] == "innovation_outlier"
    np.testing.assert_array_equal(estimator.tracks["target"].x, before)
    result = update(estimator, sample(1.3, (1.3, 0, 0)))
    assert result["position"][0] == pytest.approx(1.3, abs=.025)
    assert result["velocity"][0] == pytest.approx(1., abs=.1)
    assert np.all(np.linalg.eigvalsh(estimator.tracks["target"].covariance) > 0)


def test_soft_gate_inflates_covariance_and_learning_cannot_rescue_hard_outlier():
    estimator = RobustTargetFilter(RobustFilterConfig(initial_velocity_sigma_mps=.01))
    update(estimator, sample(0))
    result = update(estimator, sample(.01, (.14, 0, 0)))
    assert result is not None and estimator.last_diagnostic["decision"] == "suspect"
    assert result["track_status"] == "degraded"
    assert estimator.last_diagnostic["covariance_scale"] > 1
    assert update(estimator, sample(.02, (50, 0, 0)), covariance_scale=100) is None


def test_duplicates_old_and_out_of_sequence_measurements_are_not_reused():
    estimator = RobustTargetFilter()
    first = sample(0)
    update(estimator, first)
    covariance = estimator.tracks["target"].covariance.copy()
    assert update(estimator, first) is None
    np.testing.assert_array_equal(estimator.tracks["target"].covariance, covariance)
    update(estimator, sample(.2))
    assert update(estimator, sample(.1, source=1)) is None
    assert estimator.last_diagnostic["reason"] == "out_of_sequence_observation"


def test_same_time_independent_sources_and_separate_target_states():
    estimator = RobustTargetFilter()
    update(estimator, sample(0))
    result = update(estimator, sample(0, (.01, 0, 0), source=1))
    assert result["source_mode"] == 4
    assert result["covariance"][0, 0] < .001
    update(estimator, sample(0, (20, 0, 0), target="another"))
    assert estimator.tracks["target"].x[0] < .02
    assert estimator.tracks["another"].x[0] == 20


def test_loss_prediction_and_recovery_need_multiple_consistent_observations():
    estimator = RobustTargetFilter()
    update(estimator, sample(0))
    assert estimator.status("target", int(1.3e9)) == "lost"
    predicted = estimator.predict("target", int(1.4e9))
    assert predicted["predicted"] and not predicted["valid_observation"]
    assert predicted["covariance"][0, 0] > .001
    assert estimator.predict("target", int(2e9)) is None
    assert update(estimator, sample(1., (4., 0, 0))) is None
    assert update(estimator, sample(1.1, (4.01, 0, 0))) is None
    result = update(estimator, sample(1.2, (4.02, 0, 0)))
    assert result["track_status"] == "re-associated"
    assert result["position"][0] == pytest.approx(4.02)


@pytest.mark.parametrize("changes", [
    {"unit": "cm"}, {"frame_id": "base_link"}, {"position": np.array([np.nan, 0, 0])},
    {"orientation_xyzw": np.zeros(4)}, {"covariance": -np.eye(3)},
    {"validity": 2, "invalid_reason": "occluded"},
])
def test_invalid_input_cannot_initialize_filter(changes):
    estimator = RobustTargetFilter()
    assert update(estimator, replace(sample(0), **changes)) is None
    assert not estimator.tracks


def test_old_receive_future_and_bad_weight_are_rejected():
    estimator = RobustTargetFilter()
    item = sample(0)
    assert estimator.update(item, item.stamp_ns + int(1e9)) is None
    assert estimator.update(item, item.stamp_ns - 1) is None
    assert update(estimator, replace(item, receive_timestamp_ns=item.stamp_ns + 3)) is None
    assert update(estimator, item, covariance_scale=float("nan")) is None
    assert not estimator.tracks
