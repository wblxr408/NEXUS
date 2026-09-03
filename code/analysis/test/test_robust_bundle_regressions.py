"""Failures/degeneracies must not become apparently precise target poses."""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from localization.bundle_solver import (
    BundleConfig, BundleProblem, PoseState, _Layout, _reject_outliers, solve_bundle,
)
from localization.factors import BearingFactor, RelativePoseFactor, SupportPlaneFactor
from localization.online_solver import OnlineFrame, OnlineLocalizer


def config(**kwargs):
    return BundleConfig(chain="regression", enabled_factors=("F1", "F3", "F5"), factor_weights={},
                        loss="linear", second_pass=False, **kwargs)


def center_problem():
    target = PoseState.identity([0., 0., 3.])
    cameras = {i: PoseState.identity([x, 0., 0.]) for i, x in enumerate((-.5, .5))}
    bearings = [BearingFactor(1, i, np.zeros(3), np.array([-pose.translation_m[0] / 3., 0.]), .001)
                for i, pose in cameras.items()]
    return BundleProblem({1: target}, cameras, bearings)


def test_unobservable_rotation_does_not_have_zero_covariance():
    solution = solve_bundle(center_problem(), config())
    assert solution.valid
    assert solution.unobservable_target_dofs[1] == (0, 1, 2)
    diagonal = np.diag(solution.target_covariances[1])
    assert np.all(np.isinf(diagonal[:3]))
    assert np.all(np.isfinite(diagonal[3:])) and np.all(diagonal[3:] > 0)


def test_support_plane_alone_cannot_locate_a_target_in_three_dimensions():
    problem = center_problem()
    problem.factors = [SupportPlaneFactor(1, .1, 2.9)]
    solution = solve_bundle(problem, config())
    assert not solution.valid
    assert solution.degraded_reason == "unconstrained_targets:1"


def test_other_targets_cannot_supply_missing_views():
    problem = center_problem()
    problem.target_poses[2] = PoseState.identity([1., 0., 3.])
    problem.factors.append(BearingFactor(2, 0, np.zeros(3), np.array([.5, 0]), .001))
    solution = solve_bundle(problem, config())
    assert not solution.valid and solution.degraded_reason == "unconstrained_targets:2"


def test_all_rejected_factors_stay_rejected():
    problem = center_problem()
    layout = _Layout(problem, config(), problem.factors)
    wrong_state = layout.initial(problem)
    wrong_state[3:6] += np.array([10., 10., 0.])
    result = SimpleNamespace(x=wrong_state)
    _, retained, rejected = _reject_outliers(problem, config(), problem.factors, layout, result)
    assert retained == [] and rejected == 4


def test_forward_motion_with_zero_parallax_is_unobservable():
    problem = center_problem()
    problem.camera_poses = {0: PoseState.identity(), 1: PoseState.identity([0, 0, .5])}
    problem.factors = [BearingFactor(1, i, np.zeros(3), np.zeros(2), .001) for i in (0, 1)]
    solution = solve_bundle(problem, config())
    assert not solution.valid and solution.degraded_reason == "unobservable_target_positions:1"


def test_online_solver_drops_factors_referencing_evicted_cameras(monkeypatch):
    import localization.online_solver as module

    problem = center_problem()
    base_solution = solve_bundle(problem, config())
    seen = []

    def solver(candidate, settings):
        for factor in candidate.factors:
            for kind, identifier in factor.dependencies():
                if kind == "camera":
                    assert identifier in candidate.camera_poses
        seen.append(candidate)
        return base_solution

    monkeypatch.setattr(module, "solve_bundle", solver)
    online = OnlineLocalizer(config(), problem.target_poses, window_size=2, carry_prior=False)
    for i in range(4):
        factors = [BearingFactor(1, i, np.zeros(3), np.zeros(2), .001)]
        if i:
            factors.append(RelativePoseFactor(i - 1, i, np.eye(3), np.zeros(3)))
        online.update(OnlineFrame(i, PoseState.identity(), tuple(factors)))
    assert set(seen[-1].camera_poses) == {2, 3}
    assert [factor.camera_i for factor in seen[-1].factors if factor.kind == "F5"] == [2]


def test_numerical_failure_rolls_back_online_window_and_carried_state(monkeypatch):
    import localization.online_solver as module

    problem = center_problem()
    solution = solve_bundle(problem, config())
    online = OnlineLocalizer(config(), problem.target_poses, window_size=2, carry_prior=False)
    monkeypatch.setattr(module, "solve_bundle", lambda *args: solution)
    first = OnlineFrame(0, problem.camera_poses[0], (problem.factors[0],))
    online.update(first)
    before = online.target_poses[1].translation_m.copy()
    monkeypatch.setattr(module, "solve_bundle", lambda *args: replace(solution, valid=False, degraded_reason="solver_failed"))
    second = OnlineFrame(1, problem.camera_poses[1], (problem.factors[1],))
    failed = online.update(second)
    assert not failed.valid and len(online.window) == 1
    np.testing.assert_array_equal(online.target_poses[1].translation_m, before)


def test_invalid_factor_sigma_does_not_enter_online_window():
    online = OnlineLocalizer(config(), {1: PoseState.identity([0, 0, 3])}, carry_prior=False)
    factor = BearingFactor(1, 0, np.zeros(3), np.zeros(2), np.nan)
    failed = online.update(OnlineFrame(0, PoseState.identity(), (factor,)))
    assert not failed.valid and failed.degraded_reason.startswith("invalid_window:")
    assert not online.window
