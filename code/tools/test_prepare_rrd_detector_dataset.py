import numpy as np
import pytest

from prepare_rrd_detector_dataset import normalized_box, time_splits


def test_time_blocks_have_no_interleaving_and_preserve_gap():
    times = np.arange(500) * .05 + 5.318
    labels = np.array(time_splits(times))
    assert [sum(labels == s) for s in ("train", "val", "test", "gap")] == [300, 80, 80, 40]
    for first, second in (("train", "val"), ("val", "test")):
        assert times[labels == second].min() - times[labels == first].max() >= 1.


def test_bad_timestamps_and_empty_holdout_are_rejected():
    for times in ([0, 0, 1], [2, 1, 0], [0, float("nan"), 1], [0, .01, .02]):
        with pytest.raises(ValueError):
            time_splits(times)


def test_box_clipping_and_roundtrip():
    box = np.array(normalized_box([-10, 10, 50, 110], 100, 100))
    assert np.allclose(box, [.25, .55, .5, .9])
    for invalid in ([10, 0, 1, 5], [200, 0, 300, 20], [0, 0, float("nan"), 20]):
        with pytest.raises(ValueError):
            normalized_box(invalid, 100, 100)
