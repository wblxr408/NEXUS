"""Independent Gaussian elimination and real bearing-window regression checks."""

from dataclasses import dataclass, replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from localization.bundle_marginalization import (
    GeometryEvidence, MarginalFactor, StateReference, marginalize_factors, robust_rho,
)
from localization.bundle_solver import (
    BundleConfig, BundleProblem, PoseState, _Layout, _least_squares, _State, solve_bundle,
)
from localization.factors import (
    BearingFactor, CatalogPriorFactor, ContourFactor, PlatformPoseFactor,
    RelativePoseFactor, SegmentUwbFactor, SupportPlaneFactor,
)
from localization.online_solver import OnlineFrame, OnlineLocalizer


def settings(**kwargs):
    return BundleConfig(chain="marginal_test", enabled_factors=("F1", "F3", "F5", "F6", "F7", "F8", "linear_test"),
                        factor_weights={}, loss="linear", second_pass=False, **kwargs)


def state_of(problem, config):
    layout = _Layout(problem, config, problem.factors)
    return _State(layout.initial(problem), layout, problem)


@dataclass(frozen=True)
class LinearTestFactor:
    """Test-only local Gaussian oracle, never a production measurement."""

    references: tuple
    matrix: np.ndarray
    offset: np.ndarray
    kind: str = "linear_test"

    def dependencies(self):
        return tuple(reference.key for reference in self.references)

    def residual(self, state):
        return self.matrix @ np.concatenate([reference.local(state) for reference in self.references]) + self.offset


def gaussian_graph(matrix, offset):
    config = settings(optimize_camera_poses=True, optimize_segment_alignment=True, outlier_sigma=100.)
    poses = {i: PoseState(Rotation.from_rotvec([.2 * i, -.3, .15]).as_matrix(), np.array([i, .3, 4.])) for i in (1, 2)}
    problem = BundleProblem(poses, {8: PoseState.identity([.5, 1., .2])}, [], 1.2, np.array([.1, -.2, .3]))
    # Capture fixed values before dependencies create the optimized layout.
    base = state_of(problem, config)
    keys = (("camera", 8), ("segment", None), ("target", 1), ("target", 2))
    refs = tuple(StateReference.capture(key, base) for key in keys)
    factor = LinearTestFactor(refs, matrix, offset)
    problem.factors = [factor]
    return config, problem, state_of(problem, config), factor


def test_full_cross_information_matches_independent_schur_complement():
    rng = np.random.default_rng(51)
    matrix, offset = rng.normal(size=(48, 22)), rng.normal(size=48) * .1
    config, _, state, factor = gaussian_graph(matrix, offset)
    prior = marginalize_factors([factor], state, config, {("camera", 8)})
    hessian, gradient = matrix.T @ matrix, matrix.T @ offset
    expected_h = hessian[6:, 6:] - hessian[6:, :6] @ np.linalg.solve(hessian[:6, :6], hessian[:6, 6:])
    expected_g = gradient[6:] - hessian[6:, :6] @ np.linalg.solve(hessian[:6, :6], gradient[:6])
    assert prior.dependencies() == (("segment", None), ("target", 1), ("target", 2))
    np.testing.assert_allclose(prior.matrix.T @ prior.matrix, expected_h, rtol=2e-8, atol=2e-8)
    np.testing.assert_allclose(prior.matrix.T @ prior.offset, expected_g, rtol=2e-8, atol=2e-8)
    # Nonzero inter-target / segment blocks must survive, not separate sigmas.
    assert np.linalg.norm(expected_h[:4, 4:]) > 1.
    assert np.linalg.norm(expected_h[4:10, 10:]) > 1.
    for _ in range(5):
        delta = rng.normal(size=16) * .1
        removed = np.linalg.lstsq(matrix[:, :6], -(matrix[:, 6:] @ delta + offset), rcond=None)[0]
        profiled = np.linalg.norm(matrix @ np.r_[removed, delta] + offset) ** 2
        zero_removed = np.linalg.lstsq(matrix[:, :6], -offset, rcond=None)[0]
        profiled_zero = np.linalg.norm(matrix[:, :6] @ zero_removed + offset) ** 2
        compressed = np.linalg.norm(prior.matrix @ delta + prior.offset) ** 2 - np.linalg.norm(prior.offset) ** 2
        assert compressed == pytest.approx(profiled - profiled_zero, abs=1e-8)


def test_rank_deficient_elimination_does_not_invent_orientation_information():
    rng = np.random.default_rng(17)
    matrix = np.zeros((16, 22))
    matrix[:, [0, 1, 13, 14, 15]] = rng.normal(size=(16, 5))
    config, _, state, factor = gaussian_graph(matrix, np.zeros(16))
    prior = marginalize_factors([factor], state, config, {("camera", 8)})
    projected = matrix[:, 6:] - matrix[:, :6] @ np.linalg.lstsq(matrix[:, :6], matrix[:, 6:], rcond=None)[0]
    np.testing.assert_allclose(prior.matrix.T @ prior.matrix, projected.T @ projected, atol=1e-8)
    assert prior.matrix.shape[0] == 3
    np.testing.assert_allclose(prior.matrix[:, 4:7], 0., atol=1e-10)


def test_fully_explained_rows_leave_no_spurious_prior_at_large_scale():
    rng = np.random.default_rng(71)
    camera = rng.normal(size=(6, 6)) * 1e7
    matrix = np.c_[camera, camera @ rng.normal(size=(6, 16))]
    config, _, state, factor = gaussian_graph(matrix, np.zeros(6))
    assert marginalize_factors([factor], state, config, {("camera", 8)}) is None


def center_frames(count=8, *, support=False, optimize=False):
    target = PoseState(Rotation.from_rotvec([.4, -.2, .3]).as_matrix(), np.array([.2, -.1, 3.]))
    frames = []
    for i in range(count):
        camera = PoseState.identity([-.8 + .23 * i, .2 * np.sin(i), 0.])
        ray = target.translation_m - camera.translation_m
        factors = [BearingFactor(1, i, np.zeros(3), ray[:2] / ray[2], .002)]
        if support:
            factors.append(SupportPlaneFactor(1, 0., 3., sigma_m=.02))
        if optimize:
            factors.append(PlatformPoseFactor(i, camera.rotation, camera.translation_m, .01, .02))
            if frames:
                translation = camera.translation_m - frames[-1].camera_pose.translation_m
                factors.append(RelativePoseFactor(i - 1, i, np.eye(3), translation, .01, .03))
        frames.append(OnlineFrame(i, camera, tuple(factors)))
    return {1: target}, frames


@pytest.mark.parametrize("optimize", [False, True])
def test_overlapping_windows_match_full_batch_information_without_f8(optimize):
    config = settings(optimize_camera_poses=optimize)
    targets, frames = center_frames(optimize=optimize)
    online = OnlineLocalizer(config, targets, window_size=2, max_nfev=100)
    raw = []
    for i, frame in enumerate(frames):
        raw.extend(frame.factors)
        update = online.update(frame)
        if i == 0:
            assert not update.valid
            continue
        assert update.valid, update.degraded_reason
        batch = solve_bundle(BundleProblem(targets, {f.camera_id: f.camera_pose for f in frames[:i + 1]}, raw), config)
        assert batch.valid
        np.testing.assert_allclose(update.target_poses[1].translation_m, batch.target_poses[1].translation_m, atol=1e-7)
        np.testing.assert_allclose(update.target_poses[1].rotation, targets[1].rotation, atol=1e-6)
        np.testing.assert_allclose(update.target_covariances[1][3:, 3:], batch.target_covariances[1][3:, 3:], rtol=2e-5, atol=1e-9)
        assert np.all(np.isinf(np.diag(update.target_covariances[1])[:3]))
        assert update.view_count == i + 1
        assert len(online.window) <= 2
        assert all(f.kind != "F8" for f in online._prior_factors())
    prior = online._marginal_prior
    assert prior is not None and ("target", 1) in prior.dependencies()
    assert all(kind != "camera" or identifier in update.window_camera_ids for kind, identifier in prior.dependencies())
    for entry in online.window:
        for factor in entry.frame.factors:
            assert all(kind != "camera" or identifier in update.window_camera_ids for kind, identifier in factor.dependencies())
    if optimize:
        assert any(kind == "camera" for kind, _ in prior.dependencies())
    else:
        assert prior.matrix.shape == (3, 6)


def test_static_support_prior_is_counted_once_including_after_eviction():
    config = settings()
    targets, frames = center_frames(support=True)
    online = OnlineLocalizer(config, targets, window_size=2)
    for frame in frames:
        update = online.update(frame)
        assert update.valid, update.degraded_reason
    raw = [f for frame in frames for f in frame.factors if f.kind != "F3"] + [frames[0].factors[1]]
    batch = solve_bundle(BundleProblem(targets, {f.camera_id: f.camera_pose for f in frames}, raw), config)
    np.testing.assert_allclose(update.target_covariances[1][3:, 3:], batch.target_covariances[1][3:, 3:], rtol=2e-5)
    assert len(online._persistent_priors) == 1
    assert all(f.kind != "F3" for entry in online.window for f in entry.frame.factors)
    assert not online._marginal_prior.geometry[1].support
    altered = replace(frames[-1].factors[1], support_z_m=3.1)
    incoming = OnlineFrame(8, frames[-1].camera_pose, (replace(frames[-1].factors[0], camera_id=8), altered))
    rejected = online.update(incoming)
    assert not rejected.valid and "fixed prior changed" in rejected.degraded_reason
    assert online._last_camera_id == 7


def test_full_pose_history_preserves_roll_pitch_and_off_diagonal_information():
    rotation = Rotation.from_euler("xyz", [.4, -.3, .7]).as_matrix()
    target = PoseState(rotation, np.array([.2, -.1, 3.]))
    points = np.array([[-.3, -.2, .1], [.2, -.2, -.15], [.1, .3, .2], [-.2, .2, -.1], [.3, .1, .3]])
    config = settings()
    online = OnlineLocalizer(config, {1: target}, window_size=2)
    raw, cameras = [], {}
    for i in range(6):
        camera = PoseState.identity([-.6 + .2 * i, .1 * np.sin(i), 0.])
        cameras[i] = camera
        factors = []
        for point in points:
            ray = rotation @ point + target.translation_m - camera.translation_m
            factors.append(BearingFactor(1, i, point, ray[:2] / ray[2], .002))
        raw.extend(factors)
        result = online.update(OnlineFrame(i, camera, tuple(factors)))
    batch = solve_bundle(BundleProblem({1: target}, cameras, raw), config)
    assert result.valid and batch.valid
    np.testing.assert_allclose(result.target_poses[1].rotation, rotation, atol=1e-7)
    np.testing.assert_allclose(result.target_covariances[1], batch.target_covariances[1], rtol=4e-5, atol=1e-8)
    assert online._marginal_prior.matrix.shape == (6, 6)
    assert np.linalg.norm(result.target_covariances[1][:3, 3:]) > 1e-6


def test_segment_history_is_retained_without_raw_measurement_duplication():
    config = settings(optimize_segment_alignment=True)
    targets, frames = center_frames(7, support=True)
    online = OnlineLocalizer(config, targets, window_size=2, max_nfev=150)
    for i, frame in enumerate(frames):
        slam = np.array([i * .2, .1 * np.sin(i), 0.])
        factor = SegmentUwbFactor(slam, slam.copy(), .04)
        update = online.update(replace(frame, factors=frame.factors + (factor,)))
        assert update.valid, update.degraded_reason
    assert ("segment", None) in online._marginal_prior.dependencies()
    assert sum(f.kind == "F6" for e in online.window for f in e.frame.factors) == 2
    assert update.segment_scale == pytest.approx(1.)
    # Departed F6 rows alone give exactly this 4x4 alignment information.
    jacobians = [np.c_[np.array([i * .2, .1 * np.sin(i), 0.]), np.eye(3)] / .04 for i in range(5)]
    expected = sum(j.T @ j for j in jacobians)
    index = 0
    for reference in online._marginal_prior.references:
        if reference.key == ("segment", None):
            break
        index += reference.size
    block = online._marginal_prior.matrix[:, index:index + 4]
    np.testing.assert_allclose(block.T @ block, expected, rtol=2e-6, atol=1e-7)


def test_hard_outliers_do_not_enter_prior_or_historical_geometry():
    targets, frames = center_frames(3)
    good = frames[0].factors[0]
    bad = replace(frames[1].factors[0], normalized_xy=np.array([100., 100.]))
    config = settings()
    problem = BundleProblem(targets, {f.camera_id: f.camera_pose for f in frames}, [good, bad])
    state = state_of(problem, config)
    prior = marginalize_factors([good, bad], state, config, set())
    reference = marginalize_factors([good], state, config, set())
    np.testing.assert_allclose(prior.matrix.T @ prior.matrix, reference.matrix.T @ reference.matrix, atol=1e-9)
    assert prior.geometry[1].view_count == 1
    assert marginalize_factors([bad], state, config, set()) is None


@pytest.mark.parametrize("loss", ["linear", "soft_l1", "huber", "cauchy", "arctan"])
def test_marginal_rows_are_linear_even_when_raw_rows_are_robust(loss):
    config = replace(settings(), loss=loss, f_scale=2.)
    problem = BundleProblem({1: PoseState.identity()}, {}, [])
    state = state_of(problem, config)
    refs = (StateReference.capture(("target", 1), state),)
    prior = MarginalFactor(refs, np.eye(6) * 10., np.array([0., 0., 0., 40., 0., 0.]))
    raw = LinearTestFactor(refs, np.eye(6), np.zeros(6))
    problem.factors = [prior, raw]
    layout = _Layout(problem, config, problem.factors)
    result = _least_squares(problem, config, problem.factors, layout, layout.initial(problem))
    assert result.success
    final_state = _State(result.x, layout, problem)
    prior_r, raw_r = prior.residual(final_state), raw.residual(final_state)
    expected_cost = .5 * (prior_r @ prior_r + config.f_scale ** 2 * robust_rho((raw_r / config.f_scale) ** 2, loss)[0].sum())
    assert result.cost == pytest.approx(expected_cost, rel=1e-9)
    gradient = 10. * prior_r + raw_r * robust_rho((raw_r / config.f_scale) ** 2, loss)[1]
    np.testing.assert_allclose(gradient, 0., atol=2e-5)
    # Re-marginalizing the prior never softens its large residuals or gates it.
    recompressed = marginalize_factors([prior], state, config, set())
    np.testing.assert_allclose(recompressed.matrix.T @ recompressed.matrix, prior.matrix.T @ prior.matrix, atol=1e-7)
    np.testing.assert_allclose(recompressed.matrix.T @ recompressed.offset, prior.matrix.T @ prior.offset, atol=1e-7)


def test_failed_marginalization_rolls_back_all_state_then_retry_succeeds(monkeypatch):
    import localization.online_solver as module

    targets, frames = center_frames(5, support=True, optimize=True)
    online = OnlineLocalizer(settings(optimize_camera_poses=True), targets, window_size=2)
    for frame in frames[:4]:
        assert online.update(frame).valid
    prior = online._marginal_prior
    pose = online.target_poses[1]
    cameras = online._camera_poses
    persistent = online._persistent_priors
    window = tuple(online.window)
    original = module.marginalize_factors

    def fail(*args, **kwargs):
        raise np.linalg.LinAlgError("synthetic factorization failure")

    monkeypatch.setattr(module, "marginalize_factors", fail)
    failed = online.update(frames[4])
    assert not failed.valid and "factorization failure" in failed.degraded_reason
    assert online._marginal_prior is prior and online.target_poses[1] is pose
    assert online._camera_poses is cameras and online._persistent_priors is persistent
    assert tuple(online.window) == window and online._last_camera_id == 3
    monkeypatch.setattr(module, "marginalize_factors", original)
    assert online.update(frames[4]).valid


def test_no_history_is_created_before_geometric_initialization():
    target = PoseState.identity([0., 0., 3.])
    online = OnlineLocalizer(settings(), {1: target}, window_size=2)
    for i in range(6):
        factor = BearingFactor(1, i, np.zeros(3), np.zeros(2), .001)
        update = online.update(OnlineFrame(i, PoseState.identity([0., 0., .1 * i]), (factor,)))
        assert not update.valid
        assert online._marginal_prior is None and len(online.window) <= 2
    assert online.target_covariances == {}


def test_fixed_catalog_alone_cannot_revalidate_an_old_track_and_bad_ownership_is_rejected():
    targets, frames = center_frames(4, support=True)
    online = OnlineLocalizer(settings(), targets, window_size=2)
    for frame in frames:
        assert online.update(frame).valid
    prior = online._marginal_prior
    fixed = CatalogPriorFactor(1, targets[1].translation_m, .3)
    rejected = online.update(OnlineFrame(4, PoseState.identity(), (fixed,)))
    assert not rejected.valid and rejected.degraded_reason == "no_new_factors"
    assert online._marginal_prior is prior and len(online._persistent_priors) == 1
    assert online.update(frames[0]).degraded_reason == "out_of_order_camera_id"
    bad = online.update(OnlineFrame(4, PoseState.identity(), (frames[3].factors[0],)))
    assert not bad.valid and "capture frame" in bad.degraded_reason


def test_contour_linearization_does_not_modify_observation_normal():
    problem = BundleProblem({1: PoseState.identity([0., 0., 3.])}, {0: PoseState.identity()}, [])
    normal = np.array([3., 4.])
    factor = ContourFactor(1, 0, np.zeros(3), np.zeros(2), normal, np.eye(3), 1.)
    factor.residual(state_of(problem, settings()))
    np.testing.assert_array_equal(normal, [3., 4.])


def test_geometry_witnesses_are_bounded_and_never_overstate_baseline():
    evidence = GeometryEvidence()
    points = np.random.default_rng(19).normal(size=(80, 3))
    for i, point in enumerate(points):
        evidence = evidence.merge(GeometryEvidence(1, (tuple(point),)))
        exact = np.max(np.linalg.norm(points[:i + 1, None] - points[None, :i + 1], axis=2))
        assert len(evidence.positions) <= 6 and evidence.view_count == i + 1
        assert evidence.baseline_m <= exact + 1e-12
