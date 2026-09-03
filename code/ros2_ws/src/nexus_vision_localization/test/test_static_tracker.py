"""Reference-frame lifetime tests driven by synthetic calibrated features."""

import numpy as np
import pytest

from nexus_vision_localization.static_tracker import StaticMotionTracker
from test_visual_motion import CAMERA, scene


def test_low_parallax_accumulates_against_reference_then_recovers():
    previous, current, _, _ = scene(noise=0.)
    tracker = StaticMotionTracker(maximum_reference_age_s=.3)
    start = tracker.process(previous, 1_000_000_000, CAMERA, np.zeros(5))
    assert start.state == "initializing" and start.motion is None
    stationary = tracker.process(previous, 1_050_000_000, CAMERA, np.zeros(5))
    assert stationary.state == "degraded" and stationary.motion is None
    assert tracker.reference_stamp_ns == 1_000_000_000
    moving = tracker.process(current, 1_100_000_000, CAMERA, np.zeros(5))
    assert moving.state == "tracking" and moving.previous_stamp_ns == 1_000_000_000
    assert tracker.reference_stamp_ns == 1_100_000_000


def test_missing_texture_expires_reference_and_returns_to_initialization():
    previous, _, _, _ = scene(noise=0.)
    tracker = StaticMotionTracker(maximum_reference_age_s=.2)
    tracker.process(previous, 1_000_000_000, CAMERA, np.zeros(5))
    empty = previous.select(slice(0, 0))
    assert tracker.process(empty, 1_100_000_000, CAMERA, np.zeros(5)).state == "degraded"
    assert tracker.process(empty, 1_300_000_000, CAMERA, np.zeros(5)).state == "lost"
    recovered = tracker.process(previous, 1_400_000_000, CAMERA, np.zeros(5))
    assert recovered.motion is None and recovered.state == "initializing"


def test_camera_geometry_change_and_duplicate_time_do_not_create_motion_edges():
    previous, current, _, _ = scene(noise=0.)
    tracker = StaticMotionTracker()
    tracker.process(previous, 1_000_000_000, CAMERA, np.zeros(5))
    changed = CAMERA.copy()
    changed[0, 0] += 5.
    result = tracker.process(current, 1_100_000_000, changed, np.zeros(5))
    assert result.motion is None and result.reason == "camera_geometry_changed"
    with pytest.raises(ValueError, match="strictly increasing"):
        tracker.process(current, 1_100_000_000, changed, np.zeros(5))
