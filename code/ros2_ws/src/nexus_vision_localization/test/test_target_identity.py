"""Known synthetic identities, crossings and reference-only reacquisition."""

import numpy as np
import pytest

from nexus_vision_localization.superpoint_frontend import FeatureSet
from nexus_vision_localization.target_identity import (
    IdentityConfig, TargetCandidate, TargetIdentityTracker, appearance_descriptor,
)


def candidate(x, y=100., *, code=0, color=0, class_id=0, histogram=None):
    offsets = np.array([[dx, dy] for dy in (5., 20., 35.) for dx in (5., 15., 25., 35.)])
    descriptors = np.eye(256, dtype=np.float32)[code:code + len(offsets)]
    features = FeatureSet(offsets + [x, y], descriptors, np.ones(len(offsets)), (640, 480))
    if histogram is None:
        histogram = np.zeros(128)
        histogram[color] = 1.
    return TargetCandidate(np.array([x, y, 40., 40.]), .95, features, histogram, class_id)


def by_id(observations):
    return {item.track_id: item for item in observations}


def test_similar_class_instances_keep_ids_when_detection_order_changes():
    tracker = TargetIdentityTracker()
    first = tracker.update(1_000_000_000, [candidate(100, code=0), candidate(300, code=20)])
    a, b = first[0].track_id, first[1].track_id
    assert all(item.state == "tentative" for item in first)
    for i in range(1, 4):
        candidates = [candidate(300 - 6 * i, code=20), candidate(100 + 6 * i, code=0)]
        output = by_id(tracker.update(1_000_000_000 + i * 100_000_000, candidates))
    assert len(output) == 2 and output[a].state == output[b].state == "confirmed"
    assert output[a].observed and output[a].bbox_xywh_px[0] < output[b].bbox_xywh_px[0]
    assert output[a].velocity_px_s[0] > 0 and output[b].velocity_px_s[0] < 0
    assert output[a].feature_inliers == 12
    assert np.linalg.eigvalsh(output[a].covariance).min() > 0


def test_complete_identical_crossing_remains_ambiguous_after_positions_separate():
    tracker = TargetIdentityTracker(IdentityConfig(confirmation_hits=2))
    tracker.update(1_000_000_000, [candidate(100), candidate(130)])
    tracker.update(1_100_000_000, [candidate(110), candidate(120)])
    before = {key: track.latest for key, track in tracker.tracks.items()}
    crossing = tracker.update(1_200_000_000, [candidate(115), candidate(115)])
    assert len(crossing) == 2 and all(item.identity_ambiguous and not item.observed for item in crossing)
    assert all(item.identity_confidence <= .25 for item in crossing)
    assert all(tracker.tracks[key].latest is template for key, template in before.items())
    separated = tracker.update(1_300_000_000, [candidate(90), candidate(140)])
    assert len(separated) == 2 and all(item.identity_ambiguous and not item.observed for item in separated)
    assert tracker.last_diagnostics[0]["decision"] == "identity_ambiguous"


def test_distinct_appearance_restores_identity_after_ambiguous_occlusion():
    tracker = TargetIdentityTracker(IdentityConfig(confirmation_hits=2))
    initial = tracker.update(1_000_000_000, [candidate(100, color=0), candidate(130, color=1)])
    a, b = initial[0].track_id, initial[1].track_id
    tracker.update(1_100_000_000, [candidate(110, color=0), candidate(120, color=1)])
    mixed = np.zeros(128)
    mixed[:2] = .5
    tracker.update(1_200_000_000, [candidate(115, histogram=mixed), candidate(115, histogram=mixed)])
    output = by_id(tracker.update(1_300_000_000, [candidate(90, color=1), candidate(140, color=0)]))
    assert output[a].observed and output[b].observed
    assert output[a].state == output[b].state == "re-associated"
    assert output[a].bbox_xywh_px[0] == pytest.approx(140.)
    assert output[b].bbox_xywh_px[0] == pytest.approx(90.)
    assert not output[a].identity_ambiguous


def test_identical_targets_lost_between_frames_cannot_reassociate_by_old_position():
    tracker = TargetIdentityTracker(IdentityConfig(confirmation_hits=2))
    tracker.update(1_000_000_000, [candidate(100), candidate(300)])
    confirmed = tracker.update(1_100_000_000, [candidate(100), candidate(300)])
    assert all(item.state == "confirmed" for item in confirmed)
    # No intermediate images: two identical objects may have crossed while
    # hidden. A nearby old center does not establish identity on reappearance.
    output = tracker.update(1_500_000_000, [candidate(100), candidate(300)])
    assert len(output) == 2 and all(item.identity_ambiguous and not item.observed for item in output)


def test_registered_reference_is_located_without_any_category_detector():
    tracker = TargetIdentityTracker()
    reference = candidate(40, y=50, color=7, class_id=None)
    tracker.register_reference("chosen_object", reference)
    observed = candidate(240, y=180, color=7, class_id=None)
    distractor = candidate(400, y=260, code=30, color=7, class_id=None)
    combined = FeatureSet(np.vstack((observed.features.points_px, distractor.features.points_px)),
                          np.vstack((observed.features.descriptors, distractor.features.descriptors)), np.ones(24), (640, 480))
    image = np.zeros((480, 640, 3), np.uint8)
    image[:, :, 2] = 255
    proposals = tracker.reference_candidates(combined, image)
    assert len(proposals) == 1
    np.testing.assert_allclose(proposals[0].bbox_xywh_px, observed.bbox_xywh_px, atol=1e-4)
    output = tracker.update(1_000_000_000, proposals)
    assert len(output) == 1 and output[0].track_id == "chosen_object"
    assert output[0].observed and output[0].state == "tentative"
    tracker.update(1_100_000_000, proposals)
    assert tracker.update(1_200_000_000, proposals)[0].state == "confirmed"


def test_loss_prediction_is_bounded_and_reference_tracks_are_retained():
    tracker = TargetIdentityTracker(IdentityConfig(automatic_retention_s=1.))
    tracker.register_reference("selected", candidate(100), stamp_ns=1_000_000_000)
    tracker.update(1_000_000_000, [candidate(100)])
    assert tracker.tracks["selected"].hits == 1  # registering and processing one frame is not two hits
    tracker.update(1_100_000_000, [candidate(106)])
    tracker.update(1_200_000_000, [candidate(112)])
    lost = tracker.update(1_800_000_000, [])[0]
    later = tracker.update(2_800_000_000, [])[0]
    assert lost.state == later.state == "lost" and not lost.observed and not later.observed
    np.testing.assert_allclose(lost.bbox_xywh_px, later.bbox_xywh_px)
    assert later.last_observed_ns == 1_200_000_000
    assert np.trace(later.covariance) > np.trace(lost.covariance)


def test_failed_camera_warp_does_not_partially_mutate_tracks():
    tracker = TargetIdentityTracker()
    tracker.update(1_000_000_000, [candidate(100)])
    before = next(iter(tracker.tracks.values())).x.copy()
    homography = np.eye(3)
    homography[2] = [0., 1 / 120., -1.]
    with pytest.raises(ValueError, match="infinity"):
        tracker.update(1_100_000_000, [candidate(100)], camera_homography=homography)
    np.testing.assert_array_equal(next(iter(tracker.tracks.values())).x, before)
    assert tracker.last_stamp_ns == 1_000_000_000


def test_track_capacity_is_reported_and_color_histogram_uses_selected_region():
    tracker = TargetIdentityTracker(IdentityConfig(maximum_tracks=1))
    tracker.update(1_000_000_000, [candidate(100), candidate(300, code=30)])
    assert len(tracker.tracks) == 1 and tracker.last_diagnostics[0]["reason"] == "track_capacity"
    image = np.zeros((20, 20, 3), np.uint8)
    image[:10, :, 2] = 255
    image[10:, :, 0] = 255
    red = appearance_descriptor(image, [0., 0., 20., 10.])
    blue = appearance_descriptor(image, [0., 10., 20., 10.])
    assert red.sum() == blue.sum() == 1. and np.dot(red, blue) == 0.
