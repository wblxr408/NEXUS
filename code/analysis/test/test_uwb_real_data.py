import csv
import hashlib
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent.parent / "simulation"))

from algorithms.router import AlgorithmRouter, register_default_algorithms
from algorithms.uwb.real_data import (
    load_observation_frames,
    load_truth_points,
    normalize_observations,
    reset_runner_state,
    sha256_file,
)
from algorithms.uwb.run_real_data import main as run_real_data_main
from generators.range_simulation import Anchor, simulate_ranges


OBSERVATION_COLUMNS = [
    "frame_seq", "sample_timestamp_ns", "receive_timestamp_ns", "tag_id",
    "anchor_id", "anchor_x_m", "anchor_y_m", "anchor_z_m", "range_m",
    "stddev_m", "point_id", "repeat_id",
]


def _write_observations(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OBSERVATION_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def _row(frame_seq=1, sample_ns=1_000, receive_ns=1_100, anchor_id="A1",
        range_value="250", stddev="2", point_id="p01", repeat_id="2"):
    positions = {
        "A1": (0.0, 0.0, 2.0),
        "A2": (4.0, 0.0, 2.0),
        "A3": (4.0, 4.0, 2.0),
        "A4": (0.0, 4.0, 2.0),
    }
    x, y, z = positions[anchor_id]
    return {
        "frame_seq": frame_seq,
        "sample_timestamp_ns": sample_ns,
        "receive_timestamp_ns": receive_ns,
        "tag_id": "target",
        "anchor_id": anchor_id,
        "anchor_x_m": x,
        "anchor_y_m": y,
        "anchor_z_m": z,
        "range_m": range_value,
        "stddev_m": stddev,
        "point_id": point_id,
        "repeat_id": repeat_id,
    }


def test_observation_import_groups_rows_and_converts_units(tmp_path):
    rows = [
        _row(anchor_id=anchor, receive_ns=1_100 + index * 10)
        for index, anchor in enumerate(("A1", "A2", "A3", "A4"))
    ]
    rows += [
        _row(frame_seq=2, sample_ns=2_000, receive_ns=2_100,
             anchor_id=anchor, range_value="300")
        for anchor in ("A1", "A2", "A3", "A4")
    ]
    path = tmp_path / "observations.csv"
    _write_observations(path, rows)

    frames = load_observation_frames(path, range_unit="cm")

    assert len(frames) == 2
    assert frames[0].frame_seq == 1
    assert frames[0].timestamp_ns == 1_000
    assert frames[0].receive_timestamp_ns == 1_130
    assert frames[0].tag_id == "target"
    assert frames[0].point_id == "p01"
    assert frames[0].repeat_id == "2"
    assert [sample.anchor_id for sample in frames[0].ranges] == ["A1", "A2", "A3", "A4"]
    assert frames[0].ranges[0].range_m == pytest.approx(2.5)
    # The normalized schema names this field ``stddev_m``; only the raw
    # range column is converted by the explicit source-unit argument.
    assert frames[0].ranges[0].stddev_m == pytest.approx(2.0)
    assert frames[0].ranges[0].anchor_position_m == (0.0, 0.0, 2.0)
    assert frames[1].timestamp_ns == 2_000


def test_normalize_raw_csv_uses_explicit_unit_and_anchor_file(tmp_path):
    raw = tmp_path / "raw.csv"
    raw.write_text(
        "timestamp_ns,receive_timestamp_ns,tag_id,anchor_id,range,stddev,point_id,repeat_id\n"
        "1000,1100,target,A1,250,2,p01,1\n", encoding="utf-8")
    anchors = tmp_path / "anchors.yaml"
    anchors.write_text("anchors:\n  - id: A1\n    position_m: [1, 2, 3]\n", encoding="utf-8")
    normalized = tmp_path / "observations.csv"
    normalize_observations(raw, normalized, range_unit="cm", anchors_path=anchors)
    frames = load_observation_frames(normalized)
    assert frames[0].ranges[0].range_m == pytest.approx(2.5)
    assert frames[0].ranges[0].anchor_position_m == (1.0, 2.0, 3.0)


def test_observation_import_retains_incomplete_frame(tmp_path):
    path = tmp_path / "observations.csv"
    _write_observations(path, [_row(anchor_id="A1"), _row(anchor_id="A2")])

    frames = load_observation_frames(path)

    assert len(frames) == 1
    assert len(frames[0].ranges) == 2


@pytest.mark.parametrize("bad_range", ["0", "-1", "nan", "inf", "-inf"])
def test_observation_import_rejects_non_positive_or_non_finite_range(tmp_path, bad_range):
    path = tmp_path / "observations.csv"
    _write_observations(path, [_row(range_value=bad_range)])

    with pytest.raises(ValueError, match="range|non-finite"):
        load_observation_frames(path)


def test_observation_import_rejects_duplicate_anchor_in_frame(tmp_path):
    path = tmp_path / "observations.csv"
    _write_observations(path, [_row(anchor_id="A1"), _row(anchor_id="A1")])

    with pytest.raises(ValueError, match="duplicate anchor"):
        load_observation_frames(path)


@pytest.mark.parametrize(
    "rows",
    [
        [_row(sample_ns=2_000), _row(frame_seq=2, sample_ns=1_000)],
        [_row(), _row(frame_seq=2)],
        [_row(receive_ns=900)],
    ],
)
def test_observation_import_rejects_bad_timestamp_order(tmp_path, rows):
    path = tmp_path / "observations.csv"
    _write_observations(path, rows)

    with pytest.raises(ValueError, match="timestamp"):
        load_observation_frames(path)


def test_observation_import_rejects_missing_columns_and_bad_unit(tmp_path):
    path = tmp_path / "observations.csv"
    row = _row()
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[column for column in OBSERVATION_COLUMNS if column != "stddev_m"])
        writer.writeheader()
        writer.writerow({key: value for key, value in row.items() if key != "stddev_m"})

    with pytest.raises(ValueError, match="missing columns"):
        load_observation_frames(path)
    _write_observations(path, [_row()])
    with pytest.raises(ValueError, match="unsupported range unit"):
        load_observation_frames(path, range_unit="feet")


def test_truth_import_validates_and_preserves_provenance(tmp_path):
    path = tmp_path / "truth_points.csv"
    path.write_text(
        "point_id,x_m,y_m,z_m,zone,truth_method,truth_uncertainty_m\n"
        "p01,1.0,2.0,0.5,core,total_station,0.01\n",
        encoding="utf-8",
    )

    points = load_truth_points(path)

    assert points == {
        "p01": {
            "x_m": 1.0,
            "y_m": 2.0,
            "z_m": 0.5,
            "zone": "core",
            "truth_method": "total_station",
            "truth_uncertainty_m": 0.01,
        }
    }


@pytest.mark.parametrize(
    "content, error",
    [
        (
            "point_id,x_m,y_m,z_m,zone,truth_method,truth_uncertainty_m\n"
            "p01,1,2,0.5,core,total_station,-0.01\n",
            "invalid truth point",
        ),
        (
            "point_id,x_m,y_m,z_m,zone,truth_method,truth_uncertainty_m\n"
            "p01,1,2,0.5,core,total_station,0.01\n"
            "p01,1,2,0.5,core,total_station,0.01\n",
            "duplicate truth point",
        ),
    ],
)
def test_truth_import_rejects_invalid_points(tmp_path, content, error):
    path = tmp_path / "truth_points.csv"
    path.write_text(content, encoding="utf-8")

    with pytest.raises(ValueError, match=error):
        load_truth_points(path)


def test_sha256_file_matches_standard_digest(tmp_path):
    path = tmp_path / "raw.log"
    path.write_bytes(b"measured uwb\n")

    assert sha256_file(path) == hashlib.sha256(b"measured uwb\n").hexdigest()


def test_imported_frame_runs_all_five_algorithms(tmp_path):
    anchors = {
        "A1": (0.0, 0.0, 2.0),
        "A2": (4.0, 0.0, 2.0),
        "A3": (4.0, 4.0, 2.0),
        "A4": (0.0, 4.0, 2.0),
    }
    target = np.array([2.0, 2.0, 2.0])
    rows = []
    for anchor_id, position in anchors.items():
        row = _row(anchor_id=anchor_id, range_value=str(np.linalg.norm(target - position)))
        rows.append(row)
    path = tmp_path / "observations.csv"
    _write_observations(path, rows)
    frame = load_observation_frames(path)[0]

    register_default_algorithms()
    algorithms = [
        "uwb.matlab.trilateration",
        "uwb.matlab.multilateration",
        "uwb.matlab.taylor",
        "uwb.matlab.ekf",
        "uwb.matlab.ukf",
    ]
    for algorithm in algorithms:
        runner = AlgorithmRouter(algorithm).spec.runner
        reset_runner_state(runner)
        result = AlgorithmRouter(algorithm).run(observation_frame=frame)
        assert result.valid, f"{algorithm} failed: {result.metadata}"
        assert result.estimate.shape == (2,)
        assert result.metadata["timestamp_ns"] == frame.timestamp_ns


def test_real_data_runner_writes_measured_outputs(tmp_path):
    anchors = {
        "A1": (0.0, 0.0, 2.0),
        "A2": (4.0, 0.0, 2.0),
        "A3": (4.0, 4.0, 2.0),
        "A4": (0.0, 4.0, 2.0),
    }
    target = np.array([2.0, 2.0, 2.0])
    observation_path = tmp_path / "observations.csv"
    _write_observations(observation_path, [
        _row(anchor_id=anchor_id, range_value=str(np.linalg.norm(target - position)))
        for anchor_id, position in anchors.items()
    ])
    truth_path = tmp_path / "truth_points.csv"
    truth_path.write_text(
        "point_id,x_m,y_m,z_m,zone,truth_method,truth_uncertainty_m\n"
        "p01,2.0,2.0,2.0,core,total_station,0.01\n",
        encoding="utf-8",
    )
    output_path = tmp_path / "metrics"

    run_real_data_main([
        "--input", str(observation_path), "--truth", str(truth_path),
        "--output", str(output_path),
    ])

    assert (output_path / "results.csv").exists()
    summary = (output_path / "summary.json").read_text(encoding="utf-8")
    assert '"data_type": "measured"' in summary
    assert '"is_measured_result": true' in summary
    for algorithm in ("trilateration", "multilateration", "taylor", "ekf", "ukf"):
        assert (output_path / f"{algorithm}.json").exists()


def test_real_data_runner_marks_missing_anchor_frame_invalid(tmp_path):
    observation_path = tmp_path / "observations.csv"
    _write_observations(observation_path, [_row(anchor_id="A1"), _row(anchor_id="A2")])
    truth_path = tmp_path / "truth_points.csv"
    truth_path.write_text(
        "point_id,x_m,y_m,z_m,zone,truth_method,truth_uncertainty_m\n"
        "p01,1,1,1,core,total_station,0.01\n", encoding="utf-8")
    output_path = tmp_path / "metrics"
    run_real_data_main(["--input", str(observation_path), "--truth", str(truth_path), "--output", str(output_path)])
    result = (output_path / "results.csv").read_text(encoding="utf-8")
    assert result.count("insufficient anchors") == 5


@pytest.mark.parametrize("algorithm", ["uwb.matlab.ekf", "uwb.matlab.ukf"])
def test_filter_state_reset_reproduces_first_frame(algorithm):
    anchors = [
        Anchor("A1", (0.30, 0.30, 2.20)),
        Anchor("A2", (3.70, 0.30, 2.20)),
        Anchor("A3", (3.70, 4.40, 2.20)),
        Anchor("A4", (0.30, 4.40, 2.20)),
    ]
    frame = simulate_ranges(anchors, [[2.0, 2.35, 1.2]], [1_000])[0]
    register_default_algorithms()
    runner = AlgorithmRouter(algorithm).spec.runner
    reset_runner_state(runner)
    first = AlgorithmRouter(algorithm).run(observation_frame=frame)
    AlgorithmRouter(algorithm).run(observation_frame=frame)
    reset_runner_state(runner)
    reset = AlgorithmRouter(algorithm).run(observation_frame=frame)

    assert first.valid and reset.valid
    np.testing.assert_allclose(reset.estimate, first.estimate)


@pytest.mark.parametrize("algorithm", ["uwb.matlab.ekf", "uwb.matlab.ukf"])
def test_filter_state_is_continuous_across_imported_frames(tmp_path, algorithm):
    anchors = {
        "A1": (0.0, 0.0, 2.0),
        "A2": (4.0, 0.0, 2.0),
        "A3": (4.0, 4.0, 2.0),
        "A4": (0.0, 4.0, 2.0),
    }
    positions = (np.array([1.0, 1.0, 2.0]), np.array([3.0, 3.0, 2.0]))
    rows = []
    for frame_seq, (sample_ns, target) in enumerate(zip((1_000, 2_000), positions), start=1):
        for anchor_id, anchor_position in anchors.items():
                rows.append(_row(
                    frame_seq=frame_seq,
                    sample_ns=sample_ns,
                    receive_ns=sample_ns + 100,
                    anchor_id=anchor_id,
                range_value=str(np.linalg.norm(target - anchor_position)),
            ))
    path = tmp_path / "observations.csv"
    _write_observations(path, rows)
    frames = load_observation_frames(path)

    register_default_algorithms()
    runner = AlgorithmRouter(algorithm).spec.runner
    reset_runner_state(runner)
    assert AlgorithmRouter(algorithm).run(observation_frame=frames[0]).valid
    continuous = AlgorithmRouter(algorithm).run(observation_frame=frames[1])
    reset_runner_state(runner)
    second_only = AlgorithmRouter(algorithm).run(observation_frame=frames[1])

    assert continuous.valid and second_only.valid
    assert not np.allclose(continuous.estimate, second_only.estimate)
