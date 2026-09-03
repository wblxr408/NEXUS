"""Verification for the markerless localization bundle (design section 14).

The depth test is the design's own stop condition: if bearing intersection plus
the support plane cannot bring depth error well under 10 mm at 1 px bearing
noise, the sensitivity derivation is wrong and the plan must be re-derived
instead of swapping networks.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
REPOSITORY = ROOT.parents[1]
sys.path.insert(0, str(ROOT))

from localization.bundle_solver import BundleConfig, BundleProblem, PoseState, load_chain_config, solve_bundle
from localization.bundle_io import load_problem
from localization.channel_consistency import ChannelPose, channel_consistency
from localization.factors import BearingFactor, SupportPlaneFactor, normalized_from_pixel
from localization.gating import QualityMetrics, select_chain
from localization.records import result_records, solve_records, write_records


FOCAL = 1978.892939
CAMERA_MATRIX = np.array([[FOCAL, 0.0, 1640.5], [0.0, FOCAL, 1232.5], [0.0, 0.0, 1.0]])
BEARING_SIGMA = 1.0 / FOCAL
TARGET_SIZE_M = np.array([0.190, 0.190, 0.130])
TARGET_TRANSLATION_M = np.array([1.320, 3.080, 0.115])
SUPPORT_Z_M = 0.050


def _rotation_z(degrees):
    radians = np.radians(degrees)
    return np.array([[np.cos(radians), -np.sin(radians), 0.0], [np.sin(radians), np.cos(radians), 0.0], [0.0, 0.0, 1.0]])


def _look_at(position, target):
    """R_map_camera for an optical frame with +Z forward, +X right, +Y down."""
    forward = target - position
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right = right / np.linalg.norm(right)
    return np.vstack((right, np.cross(forward, right), forward)).T


def _camera_poses(count=8, radius=1.0, height=3.55):
    poses = {}
    for index, angle in enumerate(np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)):
        position = TARGET_TRANSLATION_M + np.array([radius * np.cos(angle), radius * np.sin(angle), height])
        poses[index] = PoseState(_look_at(position, TARGET_TRANSLATION_M), position)
    return poses


def _object_points():
    half = TARGET_SIZE_M / 2.0
    return np.array([[sx * half[0], sy * half[1], sz * half[2]]
                     for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])


def _bearing_factors(cameras, rotation, translation, rng, pixel_sigma=1.0):
    factors = []
    for camera_id, pose in cameras.items():
        for point in _object_points():
            point_camera = pose.rotation.T @ ((rotation @ point + translation) - pose.translation_m)
            pixel = (CAMERA_MATRIX @ point_camera)[:2] / point_camera[2]
            noisy = pixel + rng.normal(0.0, pixel_sigma, 2)
            factors.append(BearingFactor(1, camera_id, point, normalized_from_pixel(noisy, CAMERA_MATRIX), BEARING_SIGMA * pixel_sigma))
    return factors


def _support_factor():
    return SupportPlaneFactor(1, float(TARGET_SIZE_M[2] / 2.0), SUPPORT_Z_M,
                              base_offset_m=float(TARGET_TRANSLATION_M[2] - TARGET_SIZE_M[2] / 2.0 - SUPPORT_Z_M), sigma_m=0.002)


def _problem(rng, pixel_sigma=1.0, position_offset=0.15, yaw_offset_deg=12.0):
    cameras = _camera_poses()
    rotation = _rotation_z(10.0)
    factors = _bearing_factors(cameras, rotation, TARGET_TRANSLATION_M, rng, pixel_sigma) + [_support_factor()]
    initial = PoseState(_rotation_z(10.0 + yaw_offset_deg), TARGET_TRANSLATION_M + position_offset)
    return BundleProblem(target_poses={1: initial}, camera_poses=cameras, factors=factors)


def _config(**overrides):
    settings = dict(chain="test", enabled_factors=("F1", "F3"), factor_weights={"F1": 1.0, "F3": 1.0},
                    loss="linear", second_pass=False, minimum_views=3, minimum_baseline_m=0.10, max_nfev=200)
    settings.update(overrides)
    return BundleConfig(**settings)


def test_bearing_intersection_and_support_plane_recover_metric_depth():
    solution = solve_bundle(_problem(np.random.default_rng(20260901)), _config())
    assert solution.valid and solution.depth_source == "triangulation"
    error = solution.target_poses[1].translation_m - TARGET_TRANSLATION_M
    # Depth here is the map z axis: the cameras look down at the sandbox deck.
    assert abs(float(error[2])) < 0.010
    assert float(np.linalg.norm(error)) < 0.010


def test_depth_error_shrinks_with_more_views():
    rng = np.random.default_rng(7)
    errors = {}
    for count in (3, 8):
        cameras = _camera_poses(count=count)
        factors = _bearing_factors(cameras, _rotation_z(10.0), TARGET_TRANSLATION_M, rng) + [_support_factor()]
        problem = BundleProblem(target_poses={1: PoseState(_rotation_z(22.0), TARGET_TRANSLATION_M + 0.15)},
                                camera_poses=cameras, factors=factors)
        solution = solve_bundle(problem, _config())
        errors[count] = float(np.linalg.norm(solution.target_poses[1].translation_m - TARGET_TRANSLATION_M))
    assert errors[8] <= errors[3]


def test_reported_covariance_is_statistically_consistent():
    """chi-square style check: mean normalized squared error should sit near 1."""
    normalized = []
    for trial in range(40):
        solution = solve_bundle(_problem(np.random.default_rng(1000 + trial)), _config())
        covariance = solution.target_covariances[1]
        sigma = np.sqrt(np.diag(covariance)[3:6])
        error = solution.target_poses[1].translation_m - TARGET_TRANSLATION_M
        normalized.append(np.square(error / sigma))
    statistic = float(np.mean(normalized))
    assert 0.3 < statistic < 3.0, f"covariance is not calibrated: mean e^2/sigma^2 = {statistic}"


def test_network_depth_is_never_an_input():
    """F1 carries bearings only, so no factor can accept a regressed depth."""
    factor = BearingFactor(1, 0, np.zeros(3), np.zeros(2), BEARING_SIGMA)
    assert not any("depth" in name or name == "translation_m" for name in factor.__dataclass_fields__)


def test_continuous_symmetry_absorbs_yaw_in_the_residual():
    rotations = tuple(_rotation_z(angle) for angle in np.linspace(0.0, 360.0, 24, endpoint=False))
    cameras = _camera_poses(count=4)
    point = np.array([0.06, 0.0, 0.0])
    truth = PoseState(_rotation_z(0.0), TARGET_TRANSLATION_M)
    yawed = PoseState(_rotation_z(45.0), TARGET_TRANSLATION_M)

    class _View:
        def __init__(self, pose):
            self.pose = pose

        def target_pose(self, target_id):
            return self.pose.rotation, self.pose.translation_m

        def camera_pose(self, camera_id):
            return cameras[camera_id].rotation, cameras[camera_id].translation_m

        def segment_alignment(self):
            return 1.0, np.zeros(3)

    point_camera = cameras[0].rotation.T @ ((truth.rotation @ point + truth.translation_m) - cameras[0].translation_m)
    observed = point_camera[:2] / point_camera[2]
    plain = BearingFactor(1, 0, point, observed, BEARING_SIGMA)
    symmetric = BearingFactor(1, 0, point, observed, BEARING_SIGMA, symmetries=rotations)
    assert np.linalg.norm(plain.residual(_View(yawed))) > 10.0
    assert np.linalg.norm(symmetric.residual(_View(yawed))) < np.linalg.norm(plain.residual(_View(yawed))) / 10.0


def test_chain_a_config_declares_the_two_round_schedule():
    config = load_chain_config(REPOSITORY / "config/chain_a_precision.yaml")
    assert config.stages and len(config.stages) == 2
    assert "F2" not in config.stages[0] and "F2" in config.stages[1]
    assert config.loss == "soft_l1" and config.second_pass


def test_stage_schedule_reports_one_cost_per_round():
    solution = solve_bundle(_problem(np.random.default_rng(3)), _config(stages=(("F1", "F3"), ("F1", "F3"))))
    assert len(solution.stage_costs) == 2


def test_missing_geometric_constraint_is_reported_not_guessed():
    problem = _problem(np.random.default_rng(4))
    problem.target_poses[2] = PoseState(np.eye(3), np.array([2.0, 2.0, 0.1]))
    solution = solve_bundle(problem, _config())
    assert not solution.valid and solution.degraded_reason.startswith("unconstrained_targets:2")


def test_short_baseline_falls_back_to_the_support_plane():
    cameras = _camera_poses(count=3, radius=0.01)
    rng = np.random.default_rng(5)
    factors = _bearing_factors(cameras, _rotation_z(10.0), TARGET_TRANSLATION_M, rng) + [_support_factor()]
    problem = BundleProblem(target_poses={1: PoseState(_rotation_z(10.0), TARGET_TRANSLATION_M)}, camera_poses=cameras, factors=factors)
    solution = solve_bundle(problem, _config(minimum_baseline_m=0.10))
    assert solution.depth_source == "support_plane"


def test_channel_consistency_needs_two_channels_and_names_the_outlier():
    single = channel_consistency([ChannelPose("dense", np.zeros(3))])
    assert single.disagreement_m == float("inf") and single.outlier_channel is None
    report = channel_consistency([
        ChannelPose("dense", np.array([1.0, 1.0, 1.0])),
        ChannelPose("contour", np.array([1.01, 1.0, 1.0])),
        ChannelPose("uwb", np.array([1.6, 1.0, 1.0])),
    ])
    assert report.outlier_channel == "uwb"
    assert report.disagreement_m == pytest.approx(0.6, abs=1e-6)


def test_gate_reports_unavailable_transform_instead_of_a_pose():
    metrics = QualityMetrics(12, 0.5, 20000.0, 0.9, 0.01, 0.01, 0.01, 0.001, 200.0, 0.5, transform_available=False)
    decision = select_chain(metrics, REPOSITORY / "config/chain_c_adaptive.yaml")
    assert decision.level == "unavailable" and decision.chain is None
    assert decision.degraded_reason == "transform_unavailable"


def test_records_report_missing_sigma_as_none(tmp_path):
    solution = solve_bundle(_problem(np.random.default_rng(6)), _config())
    records = solve_records(solution, frame_id="12", chain="chain_a_precision", class_names={1: "vertical_striped_tank"})
    assert len(records) == 1 and records[0].class_name == "vertical_striped_tank"
    written = write_records(tmp_path / "records.csv", records)
    assert written.is_file() and written.with_suffix(".json").is_file()
    broken = dict(solution.target_covariances)
    broken[1] = np.full((6, 6), np.nan)
    solution = type(solution)(**{**solution.__dict__, "target_covariances": broken})
    assert solve_records(solution, frame_id="12", chain="chain_a_precision")[0].sigma_z_m is None


def test_bundle_io_loads_the_reference_document():
    problem, document = load_problem(REPOSITORY / "simulation/markerless_bundle_example_v01.json")
    assert sorted(kind.kind for kind in problem.factors) == sorted(entry["kind"] for entry in document["factors"])
    solution = solve_bundle(problem, load_chain_config(REPOSITORY / "config/chain_a_precision.yaml"))
    assert solution.valid
    # The reference document places the target at 3 m along the optical axis.
    assert np.allclose(solution.target_poses[1].translation_m, document["truth"]["1"]["translation_m"], atol=1e-6)


def test_result_records_map_router_metadata():
    from algorithms.router import AlgorithmRouter

    problem, document = load_problem(REPOSITORY / "simulation/markerless_bundle_example_v01.json")
    result = AlgorithmRouter("vision.bundle_a").run(problem=problem)
    records = result_records(result, frame_id=document["frame_id"], chain="chain_a_precision")
    assert len(records) == 1 and records[0].depth_source == "triangulation" and records[0].n_views == 4
