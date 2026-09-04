"""Correlated errors, partial observability, SO(3), rejection and recovery."""

from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from nexus_fusion_localization.kinematic_filter import (
    KinematicEstimate, KinematicFilterConfig, KinematicTargetFilter, _coordinates, intersect_estimates,
)


def observation(stamp=1_000_000_000, position=(1., 2., 3.), velocity=None, rotation=None,
                covariance=None, reference="reference_feature:test:0", source="superpoint_multiview_static", target_id="one"):
    indices = np.r_[np.arange(3), np.arange(3, 6) if velocity is not None else [], np.arange(6, 9) if rotation is not None else []].astype(int)
    full = np.full((9, 9), np.nan)
    full[np.ix_(indices, indices)] = np.eye(len(indices)) * .01 if covariance is None else covariance
    return KinematicEstimate(target_id, stamp, stamp, source, reference, np.array(position), full,
                             None if velocity is None else np.array(velocity), rotation, state="confirmed")


@pytest.mark.parametrize("velocity,rotation", [(None, None), ((.2, -.1, 0.), None), (None, np.eye(3)), ((.2, -.1, 0.), np.eye(3))])
def test_identical_correlated_estimates_do_not_accumulate_false_information(velocity, rotation):
    item = observation(velocity=velocity, rotation=rotation)
    result = item
    for _ in range(30):
        result, _ = intersect_estimates(result, item)
    np.testing.assert_allclose(result.covariance, item.covariance, rtol=1e-10, atol=1e-12)
    np.testing.assert_array_equal(result.position, item.position)
    assert (result.velocity is None) == (velocity is None)
    assert (result.rotation is None) == (rotation is None)


def test_ci_matches_independent_information_formula_and_keeps_pv_cross_terms():
    rng = np.random.default_rng(12)
    left_root, right_root = rng.normal(size=(6, 6)), rng.normal(size=(6, 6))
    left = observation(velocity=(.2, -.1, 0.), covariance=(left_root @ left_root.T + np.eye(6)) * .01)
    right = observation(position=(1.04, 1.98, 3.02), velocity=(.23, -.12, .01), covariance=(right_root @ right_root.T + np.eye(6)) * .01)
    result, weight = intersect_estimates(left, right)
    il, ir = np.linalg.inv(left.covariance[:6, :6]), np.linalg.inv(right.covariance[:6, :6])
    expected_cov = np.linalg.inv(weight * il + (1 - weight) * ir)
    expected = expected_cov @ (weight * il @ np.r_[left.position, left.velocity] + (1 - weight) * ir @ np.r_[right.position, right.velocity])
    np.testing.assert_allclose(result.covariance[:6, :6], expected_cov, rtol=1e-10, atol=1e-12)
    np.testing.assert_allclose(np.r_[result.position, result.velocity], expected, atol=1e-11)
    assert np.linalg.norm(result.covariance[:3, 3:6]) > .001
    # Discrete independent grid bounds the scalar optimizer's reported optimum.
    costs = [np.linalg.slogdet(np.linalg.inv(w * il + (1 - w) * ir))[1] for w in np.linspace(0., 1., 101)]
    assert np.linalg.slogdet(expected_cov)[1] <= min(costs) + 1e-7


def test_map_angle_covariance_conversion_matches_independent_rotation_differences():
    origin = observation(velocity=(.2, -.1, 0.), rotation=Rotation.from_euler("xyz", [.3, -.2, .4]).as_matrix())
    rotation = Rotation.from_rotvec([.15, -.25, .1]).as_matrix() @ origin.rotation
    rng = np.random.default_rng(91)
    root = rng.normal(size=(9, 9)) * .02
    item = observation(velocity=(.2, -.1, 0.), rotation=rotation, covariance=root @ root.T + np.eye(9) * .001)
    mean, cov = _coordinates(item, origin)
    jacobian = np.eye(9)
    for i in range(3):
        delta = np.eye(3)[i] * 1e-6
        plus = Rotation.from_matrix(Rotation.from_rotvec(delta).as_matrix() @ rotation @ origin.rotation.T).as_rotvec()
        minus = Rotation.from_matrix(Rotation.from_rotvec(-delta).as_matrix() @ rotation @ origin.rotation.T).as_rotvec()
        jacobian[6:, 6 + i] = (plus - minus) / 2e-6
    np.testing.assert_allclose(mean[6:], [.15, -.25, .1], atol=1e-12)
    np.testing.assert_allclose(cov, jacobian @ item.covariance @ jacobian.T, rtol=1e-8, atol=1e-10)
    result, weight = intersect_estimates(origin, item)
    reset = np.eye(9)
    angle = Rotation.from_matrix(result.rotation @ origin.rotation.T).as_rotvec()
    for i in range(3):
        delta = np.eye(3)[i] * 1e-6
        plus = Rotation.from_matrix(Rotation.from_rotvec(angle + delta).as_matrix() @ origin.rotation @ result.rotation.T).as_rotvec()
        minus = Rotation.from_matrix(Rotation.from_rotvec(angle - delta).as_matrix() @ origin.rotation @ result.rotation.T).as_rotvec()
        reset[6:, 6 + i] = (plus - minus) / 2e-6
    expected = np.linalg.inv(weight * np.linalg.inv(origin.covariance) + (1 - weight) * np.linalg.inv(cov))
    np.testing.assert_allclose(result.covariance, reset @ expected @ reset.T, rtol=1e-8, atol=1e-10)


def test_partial_observations_add_only_observed_components_and_preserve_existing_cross_blocks():
    first = observation(velocity=None)
    second = observation(velocity=(.2, -.1, 0.), rotation=np.eye(3), covariance=np.eye(9) * .02)
    result, weight = intersect_estimates(first, second)
    assert result.velocity is not None and result.rotation is not None and weight < 1.
    result.validate("map")
    positions_only = observation(position=(1.01, 2., 3.), velocity=None, covariance=np.eye(3) * .02)
    retained, _ = intersect_estimates(result, positions_only)
    assert retained.velocity is not None and retained.rotation is not None
    retained.validate("map")


def test_static_history_never_moves_or_gains_process_noise_even_after_long_occlusion():
    estimator = KinematicTargetFilter()
    first = observation()
    assert estimator.update(first, first.stamp_ns) is not None
    for dt in (300_000_000, 6_000_000_000, 600_000_000_000):
        future = first.stamp_ns + dt
        estimator.expire(future)
        history = estimator.history(first.target_id, future)
        assert history["historical"] and not history["valid"]
        assert history["last_observed_stamp_ns"] == first.stamp_ns
        np.testing.assert_array_equal(history["estimate"].position, first.position)
        np.testing.assert_allclose(history["estimate"].covariance, first.covariance)
        assert history["estimate"].velocity is None
    moving = observation(first.stamp_ns + 1, velocity=(.2, 0., 0.))
    assert estimator.update(moving, moving.stamp_ns) is None
    assert estimator.last_diagnostic["reason"] == "static_target_velocity_must_be_unobserved"


def test_hard_outlier_cannot_be_rescued_by_learned_inflation_and_valid_data_recovers():
    estimator = KinematicTargetFilter()
    first = observation()
    estimator.update(first, first.stamp_ns)
    outlier = observation(first.stamp_ns + 10_000_000, position=(30., 2., 3.))
    assert estimator.update(outlier, outlier.stamp_ns, covariance_scale=100.) is None
    assert estimator.last_diagnostic["reason"] == "kinematic_innovation_outlier"
    assert estimator.tracks[first.target_id].estimate.stamp_ns == first.stamp_ns
    good = observation(first.stamp_ns + 20_000_000, position=first.position + [.001, 0., 0.])
    assert estimator.update(good, good.stamp_ns) is not None
    assert estimator.last_diagnostic["decision"] == "accepted"


def test_suspect_observation_is_downweighted_with_dimension_appropriate_gate():
    estimator = KinematicTargetFilter()
    first = observation(velocity=None)
    estimator.update(first, first.stamp_ns)
    suspect = observation(first.stamp_ns + 1, position=(1.65, 2., 3.), velocity=None)
    result = estimator.update(suspect, suspect.stamp_ns)
    assert result is not None and estimator.last_diagnostic["decision"] == "suspect"
    assert result.state == "degraded" and estimator.last_diagnostic["covariance_scale"] > 1.


@pytest.mark.parametrize("reason", ["outage", "reference_change", "identity_ambiguous"])
def test_static_recovery_requires_consistent_identity_and_same_reference_position(reason):
    estimator = KinematicTargetFilter()
    first = observation()
    estimator.update(first, first.stamp_ns)
    start = first.stamp_ns + (700_000_000 if reason == "outage" else 10_000_000)
    if reason == "identity_ambiguous":
        assert estimator.invalidate(first.target_id, start, "target_identity_ambiguous", start)
        assert estimator.association_block_reason(first.target_id) == "target_identity_ambiguous"
    reference = "reference_feature:changed:2" if reason == "reference_change" else first.position_reference
    for i in range(3):
        item = observation(start + (i + 1) * 10_000_000, position=((9. if reason == "reference_change" else 1.) + .002 * i, 2., 3.), reference=reference)
        result = estimator.update(item, item.stamp_ns)
        if i < 2:
            assert result is None and estimator.tracks[first.target_id].estimate.stamp_ns == first.stamp_ns
            assert estimator.history(first.target_id, item.stamp_ns)["last_observed_stamp_ns"] == first.stamp_ns
        else:
            assert result is not None and result.state == "re-associated"
            assert abs(result.position[0] - (9. if reason == "reference_change" else 1.)) < .01
            assert result.position_reference == reference


def test_future_old_duplicate_and_invalid_component_inputs_never_replace_current_state():
    estimator = KinematicTargetFilter()
    first = observation()
    estimator.update(first, first.stamp_ns)
    original = estimator.tracks[first.target_id]
    assert estimator.update(first, first.stamp_ns) is None
    future = observation(first.stamp_ns + 10_000_000)
    assert estimator.update(future, first.stamp_ns) is None
    assert not estimator.invalidate(first.target_id, first.stamp_ns - 1, "target_identity_ambiguous", first.stamp_ns)
    assert not estimator.invalidate(first.target_id, first.stamp_ns + 1, "target_identity_ambiguous", first.stamp_ns)
    bad = observation(first.stamp_ns + 10_000_000, velocity=None)
    invalid_covariance = bad.covariance.copy()
    invalid_covariance[3:, 3:] = 0.
    assert estimator.update(replace(bad, covariance=invalid_covariance), bad.stamp_ns) is None
    assert estimator.tracks[first.target_id] is original
    assert estimator.history(first.target_id, first.stamp_ns + 20_000_000)["historical"]


def test_stale_optional_components_expire_even_with_continuing_position_updates():
    estimator = KinematicTargetFilter()
    first = observation(rotation=np.eye(3))
    estimator.update(first, first.stamp_ns)
    for step in range(1, 5):
        item = observation(first.stamp_ns + step * 150_000_000, position=first.position, velocity=None)
        result = estimator.update(item, item.stamp_ns)
        assert result is not None
    assert result.velocity is None and result.rotation is None
    assert np.all(np.isnan(result.covariance[3:, :]))


def test_target_and_source_memory_is_bounded_and_static_history_is_retained():
    estimator = KinematicTargetFilter(KinematicFilterConfig(maximum_tracks=2, maximum_sources=2))
    one, two = observation(), observation(target_id="two")
    estimator.update(one, one.stamp_ns)
    estimator.update(two, two.stamp_ns)
    three = observation(target_id="three")
    assert estimator.update(three, three.stamp_ns) is None
    assert estimator.update(replace(one, source="tag_reference"), one.stamp_ns) is not None
    assert estimator.update(replace(one, source="third_source"), one.stamp_ns) is None
    assert len(estimator.tracks) == 2 and len(estimator.source_stamps[one.target_id]) == 2
    estimator.expire(one.stamp_ns + estimator.config.retention_ns + 1)
    assert len(estimator.tracks) == 2 and not estimator.recovery
    assert estimator.history("one", one.stamp_ns + 10_000_000_000)["historical"]


def test_fusion_confirmation_does_not_override_tentative_upstream_identity():
    estimator = KinematicTargetFilter()
    for i in range(6):
        item = replace(observation(1_000_000_000 + i * 10_000_000), state="tentative")
        result = estimator.update(item, item.stamp_ns)
        assert result is not None and result.state == "tentative"
    item = observation(1_100_000_000)
    assert estimator.update(item, item.stamp_ns).state == "confirmed"


def test_long_occlusion_does_not_allow_consistent_wrong_location_to_replace_static_anchor():
    estimator = KinematicTargetFilter()
    first = observation()
    estimator.update(first, first.stamp_ns)
    for i in range(5):
        bad = observation(first.stamp_ns + 60_000_000_000 + i * 10_000_000, position=(9., 2., 3.))
        assert estimator.update(bad, bad.stamp_ns) is None
        assert estimator.last_diagnostic["reason"] == "static_reference_position_outlier"
    np.testing.assert_array_equal(estimator.history(first.target_id, bad.stamp_ns)["estimate"].position, first.position)
