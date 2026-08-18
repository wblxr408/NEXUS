import numpy as np

from nexus_fusion_localization.fusion import Observation, fuse_observations, is_stale


def observation(position, source=2, confidence=1.0, stamp=1_000_000_000):
    return Observation(
        target_id="target_1",
        frame_id="map",
        stamp_ns=stamp,
        position=np.asarray(position, dtype=float),
        covariance=np.eye(3) * 0.01,
        confidence=confidence,
        source_mode=source,
        orientation_xyzw=np.array([0., 0., 0., 1.]),
    )


def test_single_observation_keeps_source_mode():
    result = fuse_observations([observation([1, 2, 3])], "map", 1_100_000_000, 250_000_000)
    assert result["source_mode"] == 2
    assert np.allclose(result["position"], [1, 2, 3])


def test_two_observations_are_information_weighted():
    first = observation([0, 0, 0], confidence=1.0)
    second = observation([1, 0, 0], source=1, confidence=1.0)
    second = Observation(**{**second.__dict__, "covariance": np.eye(3) * 0.04})
    result = fuse_observations([first, second], "map", 1_100_000_000, 250_000_000)
    assert 0.0 < result["position"][0] < 0.5
    assert result["source_mode"] == 4


def test_stale_observations_are_rejected():
    assert is_stale(1_000, 2_001, 1_000)
    assert not is_stale(1_001, 2_001, 1_000)
