import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[1]
for directory in ("calibration", "evaluation", "fusion", "uwb", "vision"):
    sys.path.insert(0, str(ROOT / directory))

from geometry_utils import umeyama_alignment
from metrics import position_metrics
from time_alignment import nearest_time_pairs
from gdop import gdop


def test_metrics_have_explicit_units_and_count():
    result = position_metrics([[0, 0, 0], [1, 0, 0]], [[0, 0, 0], [0, 0, 0]])
    assert result["samples"] == 2
    assert result["unit"] == "m"
    assert result["dimension"] == "3D"
    assert result["max_error_m"] == 1.0


def test_alignment_and_gdop_accept_geometry():
    points = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    rotation, translation = umeyama_alignment(points, points + [1, 2, 3])
    assert np.allclose(rotation, np.eye(3))
    assert np.allclose(translation, [1, 2, 3])
    assert gdop(points, [0.2, 0.2, 0.2]) > 0


def test_time_alignment_rejects_far_pairs():
    assert nearest_time_pairs([100], [1_000], 10) == []
