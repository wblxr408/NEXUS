import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))

from localization.bundle_solver import PoseState, load_chain_config
from localization.chain_runner import run_chain_d
from localization.gating import QualityMetrics
from localization.online_solver import OnlineFrame, OnlineLocalizer
from localization.rrd_episode import EpisodeFrame, RrdEpisode, TargetPrior
from vision.online_frontend.detect_arbiter import Detection, DetectionArbiter
from vision.online_frontend.vio_bridge import VioBridge, scaled_imu_to_si


def test_vendor_imu_conversion_matches_design_units():
    acceleration, gyro = scaled_imu_to_si(-582, 44, -9873, 1000, -250, 11)
    assert np.allclose(acceleration, [-.582, .044, -9.873])
    assert np.allclose(gyro, [1.0, -.25, .011])


def test_uvio_pose_stream_produces_f5_and_f6_in_map_metres():
    bridge = VioBridge()
    identity = np.eye(3)
    first, relative_none = bridge.add_pose(0, {
        "timestamp_ns": 1, "rotation_map_camera": identity,
        "translation_map_camera_m": [0., 0., 2.],
    })
    second, relative = bridge.add_pose(1, {
        "timestamp_ns": 2, "rotation_map_camera": identity,
        "translation_map_camera_m": [1., 0., 2.],
    })
    assert np.allclose(first.translation_m, [0., 0., 2.])
    assert relative_none is None
    assert relative is not None and relative.camera_i == 0 and relative.camera_j == 1
    assert np.allclose(relative.translation_i_j_m, [1., 0., 0.])
    factor = bridge.add_uwb(1, [1.02, 0., 2.])
    assert factor.kind == "F6" and np.allclose(factor.uwb_position_m, [1.02, 0., 2.])


def test_detector_runs_at_low_rate_and_tracker_handles_intermediate_frame():
    detector_calls = []

    def detector(image):
        detector_calls.append(1)
        return [Detection(7, np.array([18., 18., 12., 12.]), .95, 1.0)]

    first = np.zeros((64, 64), dtype=np.uint8)
    cv2.circle(first, (24, 24), 4, 255, -1)
    second = np.zeros_like(first)
    cv2.circle(second, (28, 24), 4, 255, -1)
    arbiter = DetectionArbiter(detector, detector_period_s=.5)
    observations = arbiter.process(first, 0.0)
    assert arbiter.last_mode == "detector" and len(detector_calls) == 1
    tracked = arbiter.process(second, .1)
    assert arbiter.last_mode == "tracker" and len(detector_calls) == 1
    assert tracked and tracked[0].target_id == 7


def test_online_config_is_bounded_and_disables_mesh_only_factors():
    config = load_chain_config(Path(__file__).parents[3] / "config/chain_d_online_visual.yaml")
    assert config.max_nfev == 30 and not config.second_pass
    assert "F2" not in config.enabled_factors and "F4" not in config.enabled_factors


def test_chain_d_is_registered_contract_adapter():
    # A rejected first frame is still a valid contract result and must not
    # fabricate a target pose before the minimum geometric view count.
    config = load_chain_config(Path(__file__).parents[3] / "config/chain_d_online_visual.yaml")
    localizer = OnlineLocalizer(config, {})
    quality = QualityMetrics(0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, False)
    result = run_chain_d(localizer=localizer, frame=OnlineFrame(0, PoseState.identity(), tuple(), quality=quality))
    assert result.algorithm == "vision.bundle_d" and not result.valid


def test_rejected_or_empty_online_frames_never_enter_the_solver_window():
    config = load_chain_config(Path(__file__).parents[3] / "config/chain_d_online_visual.yaml")
    localizer = OnlineLocalizer(config, {})
    rejected_quality = QualityMetrics(0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, False)
    rejected = localizer.update(OnlineFrame(1, PoseState.identity(), tuple(), quality=rejected_quality))
    assert not rejected.valid and not localizer.window

    usable_quality = QualityMetrics(3, 0.2, 1.0, 0.9, 0.1, 0.0, 0.0, 0.0, 100.0, 0.5)
    empty = localizer.update(OnlineFrame(2, PoseState.identity(), tuple(), quality=usable_quality))
    assert not empty.valid and empty.degraded_reason == "no_factors" and not localizer.window


def test_rrd_frame_factors_do_not_turn_uwb_position_into_a_fake_f6_alignment():
    episode = object.__new__(RrdEpisode)
    episode.priors = {
        "target": TargetPrior("target", 7, 0.0, 0.2, 0.01, None),
    }
    frame = EpisodeFrame(
        1, np.eye(3), PoseState.identity([1.0, 2.0, 3.0]),
        "uwb_position_with_platform_attitude", np.array([1.0, 2.0, 3.0]),
        [{"target_id": "target", "valid": True,
          "bbox": {"bbox_xyxy": [10.0, 10.0, 20.0, 20.0]}}],
    )
    factors, _ = episode.frame_factors(frame, 1)
    assert {factor.kind for factor in factors} == {"F1", "F3", "F7"}
