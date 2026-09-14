import numpy as np
import pytest

from nexus_vision_localization.adaptive_policy import (
    ActiveObservationPlanner, AdaptiveComputeScheduler, gyro_vibration,
    rotational_homography, rolling_shutter_homographies,
)


def test_scheduler_escalates_ambiguous_identity_then_respects_latency_budget():
    scheduler = AdaptiveComputeScheduler(base_keypoints=128)
    hard = scheduler.decide(identity_confidence=.2, inlier_ratio=.3, reprojection_error_px=4.,
                            blur_metric=.2, vibration_quality=.2, identity_ambiguous=True)
    assert hard.high_accuracy and hard.robust_model and hard.detector_period_s == .5 and hard.transformer_tokens == 64
    late = scheduler.decide(identity_confidence=.2, latency_ms=200.)
    assert (not late.high_accuracy and not late.robust_model and late.transformer_tokens == 4 and late.transformer_history == 1
            and late.image_scale == .5 and late.reason == "latency_budget_exceeded")


def test_scheduler_treats_low_visibility_as_additional_observability_risk():
    common = dict(identity_confidence=.3, inlier_ratio=.5, reprojection_error_px=4.,
                  blur_metric=.65, vibration_quality=.7, occlusion_ratio=.25)
    visible = AdaptiveComputeScheduler(base_keypoints=128).decide(**common, visibility=1.)
    hidden = AdaptiveComputeScheduler(base_keypoints=128).decide(**common, visibility=0.)
    assert not visible.high_accuracy and hidden.high_accuracy


def test_scheduler_escalates_when_online_outlier_estimate_is_high():
    common = dict(identity_confidence=.2, inlier_ratio=.4, reprojection_error_px=4.,
                  blur_metric=.6, vibration_quality=.7, occlusion_ratio=.3, visibility=.7)
    normal = AdaptiveComputeScheduler(base_keypoints=128).decide(**common, outlier_probability=0.)
    outlier = AdaptiveComputeScheduler(base_keypoints=128).decide(**common, outlier_probability=1.)
    assert not normal.high_accuracy and outlier.high_accuracy


def test_scheduler_accepts_a_fitted_profile_with_explicit_hysteresis():
    profile = {"risk_weights": {"identity": 1., "inliers": 0., "reprojection": 0., "sharpness": 0.,
                                "vibration": 0., "occlusion": 0., "visibility": 0., "outlier": 0.},
               "enter_threshold": .4, "exit_threshold": .2, "fast_latency_ms": 80., "late_latency_ms": 120.}
    scheduler = AdaptiveComputeScheduler(base_keypoints=128, profile=profile)
    assert scheduler.decide(identity_confidence=.2, latency_ms=50.).high_accuracy
    assert not scheduler.decide(identity_confidence=.2, latency_ms=130.).high_accuracy


def test_active_planner_respects_forbidden_circle_and_never_emits_control():
    planner = ActiveObservationPlanner(minimum_radius_m=2., maximum_radius_m=2.)
    recommendation = planner.recommend([0., 0., 2.], [2., 2., 0.], previous_view_map_m=[0., 2., 2.],
                                       forbidden_circles=[[2. + 2. * np.cos(np.deg2rad(35)),
                                                          2. + 2. * np.sin(np.deg2rad(35)), .5]])
    assert np.linalg.norm(recommendation.position_map_m[:2] - np.array([2., 2.])) == pytest.approx(2.)
    assert recommendation.score > 0


def test_active_planner_uses_reprojection_error_to_seek_a_closer_legal_view():
    planner = ActiveObservationPlanner(minimum_radius_m=1., maximum_radius_m=6.)
    nominal = planner.recommend([0., 0., 2.], [3., 0., 0.], target_sigma_m=.5, reprojection_error_px=.2)
    residual = planner.recommend([0., 0., 2.], [3., 0., 0.], target_sigma_m=.5, reprojection_error_px=4.)
    assert residual.orbit_radius_m < nominal.orbit_radius_m
    assert residual.reason == "reduce_reprojection_error"


def test_active_planner_returns_reachable_collision_free_trajectory():
    planner = ActiveObservationPlanner(minimum_radius_m=1., maximum_radius_m=2., maximum_speed_mps=5.,
                                       maximum_acceleration_mps2=5., planning_horizon_s=5., trajectory_steps=4)
    recommendation = planner.recommend([0., 0., 1.], [2., 0., 1.],
                                       forbidden_spheres=[[1., 0., 1., .25]])
    assert recommendation.trajectory_map_m.shape == (4, 3)
    assert np.all(np.linalg.norm(recommendation.trajectory_map_m - np.array([1., 0., 1.]), axis=1) > .25)
    assert recommendation.travel_time_s <= 5.


def test_gyro_vibration_and_derotation_are_finite():
    result = gyro_vibration(np.array([[0., 0., 0.], [.1, 0., 0.], [-.1, 0., 0.], [0., 0., 0.]]),
                            np.array([.01, .01, .01]))
    assert 0 < result["vibration_quality"] <= 1
    homography = rotational_homography(np.diag([300., 300., 1.]), np.eye(3))
    assert np.allclose(homography, np.eye(3))
    row_maps, exposure_sigma = rolling_shutter_homographies(np.diag([300., 300., 1.]), [0., 0., 2.], 32, .01, .004)
    assert row_maps.shape == (32, 3, 3) and exposure_sigma > 0
    assert np.allclose(row_maps[0] @ np.linalg.inv(row_maps[0]), np.eye(3))
