"""Reference-conditioned masks and transactional identity updates."""

import numpy as np
import pytest

from nexus_vision_localization.target_frontend import TargetVisualFrontend
from test_target_identity import candidate


class SyntheticDense:
    """Only the network boundary is synthetic; use real association geometry."""

    def __init__(self, features):
        self.value = features
        self.masks = []

    def features(self, allowed, **_):
        self.masks.append(allowed.copy())
        pixels = np.floor(self.value.points_px + .5).astype(int)
        return self.value.select(allowed[pixels[:, 1], pixels[:, 0]])


def image():
    output = np.zeros((480, 640, 3), np.uint8)
    output[:, :, 2] = 255
    return output


def test_reference_route_masks_target_and_does_not_commit_stale_work():
    frontend = TargetVisualFrontend()
    reference = candidate(40, y=50)
    frontend.register("chosen", SyntheticDense(reference.features), image(), reference.bbox_xywh_px, {})
    detected = candidate(240, y=180)
    pending, observations, boxes = frontend.prepare(SyntheticDense(detected.features), image(), 1_000_000_000, [], {})
    assert len(observations) == len(boxes) == 1
    assert observations[0].track_id == "chosen" and observations[0].observed
    np.testing.assert_allclose(boxes[0], detected.bbox_xywh_px, atol=1e-4)
    assert frontend.tracker.last_stamp_ns == 0 and frontend.tracker.tracks["chosen"].last_observed_ns is None
    frontend.tracker = pending
    empty = SyntheticDense(detected.features.select(np.zeros(12, bool)))
    _, missed, exclusions = frontend.prepare(empty, image(), 1_100_000_000, [], {})
    assert not missed[0].observed and len(exclusions) == 1
    assert exclusions[0][2] > detected.bbox_xywh_px[2]
    _, expired, exclusions = frontend.prepare(empty, image(), 2_000_000_000, [], {})
    assert expired[0].state == "lost" and not exclusions
    packet = frontend.packet(2_000_000_000, "camera_optical_frame", (640, 480), expired)
    assert packet["unit"] == "px" and not packet["tracks"][0]["world_position_observed"]


def test_explicit_binding_preserves_existing_track_instead_of_creating_duplicate():
    frontend = TargetVisualFrontend()
    item = candidate(100)
    detection = {"bbox_xywh_px": item.bbox_xywh_px, "confidence": .9, "class_id": 0}
    frontend.tracker, first, _ = frontend.prepare(SyntheticDense(item.features), image(), 1_000_000_000, [detection], {})
    previous = first[0].track_id
    frontend.register("selected", SyntheticDense(item.features), image(), item.bbox_xywh_px, {}, existing_track_id=previous)
    assert list(frontend.tracker.tracks) == ["selected"]
    assert frontend.tracker.tracks["selected"].last_observed_ns == 1_000_000_000
    pending, output, exclusions = frontend.prepare(SyntheticDense(item.features), image(), 1_100_000_000, [detection], {})
    assert len(output) == len(exclusions) == 1 and output[0].track_id == "selected"
    assert pending.tracks["selected"].hits == 2


def test_reference_with_insufficient_features_rejects_without_registering():
    frontend = TargetVisualFrontend()
    reference = candidate(40, y=50)
    with pytest.raises(ValueError, match="insufficient"):
        frontend.register("bad", SyntheticDense(reference.features), image(), [600., 400., 10., 10.], {})
    assert not frontend.tracker.tracks
