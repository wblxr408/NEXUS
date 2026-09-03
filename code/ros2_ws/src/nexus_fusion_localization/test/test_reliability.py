import csv
import json

import numpy as np
import pytest

from nexus_fusion_localization.reliability import (
    FEATURE_NAMES, OUTPUT_NAMES, ReliabilityMLP, covariance_scales, feature_row, split_groups,
)
from nexus_fusion_localization.train_reliability import main


def labelled_test_data():
    rng = np.random.default_rng(23)
    values = rng.normal(size=(360, 20))
    labels = np.column_stack((values[:, 0] > 0, values[:, 1] > 0,
                              values[:, 2] > 0, values[:, 3] < 0)).astype(float)
    values[:, 18] = np.nan
    groups = [f"synthetic_flight_{index // 30}" for index in range(len(values))]
    return values, labels, groups


def test_group_split_prevents_adjacent_frame_leakage_and_is_reproducible():
    _, _, groups = labelled_test_data()
    split = split_groups(groups, seed=8)
    assignments = [{groups[index] for index in indices} for indices in split.values()]
    for i, first in enumerate(assignments):
        for second in assignments[i + 1:]:
            assert first.isdisjoint(second)
    assert sum(map(len, split.values())) == len(groups)
    np.testing.assert_array_equal(split["train"], split_groups(groups, seed=8)["train"])
    with pytest.raises(ValueError, match="three"):
        split_groups(["one"] * 20)


def test_network_trains_on_groups_and_roundtrips_checkpoint(tmp_path):
    values, labels, groups = labelled_test_data()
    model = ReliabilityMLP(seed=4)
    report = model.fit(values, labels, groups, epochs=80, seed=4,
                       label_provenance="synthetic functional test only")
    assert report["metrics"]["validation"]["binary_cross_entropy"] < report["initial_validation"]["binary_cross_entropy"]
    assert report["metrics"]["test"]["brier_score"] < .16
    assert np.isfinite(model.predict({})).all()
    assert report["architecture"] == [40, 64, 32, 4]
    # Normalization may only depend on training groups.
    training = split_groups(groups, seed=4)["train"]
    assert model.mean[0] == pytest.approx(values[training, 0].mean())
    assert model.scale[18] == 1 and model.mean[18] == 0
    path = tmp_path / "quality.npz"
    model.save(path)
    restored = ReliabilityMLP.load(path)
    np.testing.assert_allclose(restored.predict(values), model.predict(values), atol=1e-12)
    with pytest.raises(FileExistsError):
        model.save(path)


def test_random_weights_cannot_be_used_as_learned_reliability():
    with pytest.raises(RuntimeError, match="trained"):
        ReliabilityMLP().predict({})


def test_covariance_mapping_is_bounded_monotonic_and_source_specific():
    reliable = covariance_scales([1, 1, 1, 0])
    unreliable = covariance_scales([0, 0, 0, 1])
    np.testing.assert_array_equal(reliable, [1, 1, 1])
    np.testing.assert_array_equal(unreliable, [100, 100, 100])
    assert covariance_scales([.9, .1, .9, 0])[1] > covariance_scales([.9, .1, .9, 0])[0]
    assert np.all(covariance_scales([.8, .8, .8, .8]) >= covariance_scales([.8, .8, .8, .1]))
    with pytest.raises(ValueError):
        covariance_scales([1, np.nan, 1, 0])


def test_missing_features_are_explicit_and_unknown_features_rejected():
    row = feature_row({"blur_metric": 50})
    assert np.isnan(row).sum() == 19
    with pytest.raises(ValueError):
        feature_row({"absolute_target_x": 3})
    with pytest.raises(ValueError):
        feature_row({"blur_metric": np.inf})


def test_training_cli_requires_group_labels_and_records_input_hash(tmp_path, capsys):
    source = tmp_path / "quality.csv"
    values, labels, groups = labelled_test_data()
    with source.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["collection_group", *FEATURE_NAMES, *OUTPUT_NAMES])
        writer.writeheader()
        for features, outputs, group in zip(values, labels, groups):
            writer.writerow({"collection_group": group, **dict(zip(FEATURE_NAMES, features)),
                             **dict(zip(OUTPUT_NAMES, outputs))})
    output = tmp_path / "model.npz"
    main(["--input", str(source), "--output", str(output), "--epochs", "3",
          "--label-provenance", "synthetic functional CLI test"])
    report = json.loads(capsys.readouterr().out)
    assert len(report["input_sha256"]) == 64
    assert ReliabilityMLP.load(output).metadata["split_groups"] == report["split_groups"]
