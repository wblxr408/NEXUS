"""Synthetic head/descriptor checks; these do not exercise pretrained weights."""

import hashlib
import json

import numpy as np
import pytest

from nexus_vision_localization.superpoint_frontend import (
    DenseSuperPoint, FeatureSet, SuperPointOnnx, feature_quality,
    match_features, roi_mask, static_background_mask,
)


def dense(points, image_size=(32, 32)):
    logits = np.full((1, 65, 4, 4), -20., np.float32)
    logits[:, 64] = 20.
    for x, y, score in points:
        logits[0, 64, y // 8, x // 8] = -20.
        logits[0, (y % 8) * 8 + x % 8, y // 8, x // 8] = score
    descriptors = np.zeros((1, 256, 4, 4), np.float32)
    for y in range(4):
        for x in range(4):
            descriptors[0, y * 4 + x, y, x] = 1.
    return DenseSuperPoint(logits, descriptors, image_size)


def test_dense_cell_decode_resize_and_descriptor_normalization():
    heads = dense([(9, 10, 20.), (23, 22, 20.)], (64, 48))
    features = heads.features(threshold=.5)
    np.testing.assert_allclose(features.points_px, [[18.5, 15.25], [46.5, 33.25]])
    np.testing.assert_allclose(np.linalg.norm(features.descriptors, axis=1), 1., atol=1e-6)
    assert features.image_size_wh == (64, 48)
    assert np.all(features.scores > .99)


def test_descriptor_sampling_respects_cell_coordinates_and_zero_padding():
    heads = dense([])
    values, valid = heads._sample_descriptors(np.array([[3.5, 3.5], [31., 31.], [17.25, 17.25]]))
    assert valid.all()
    np.testing.assert_allclose(values[0], np.eye(256)[0])
    np.testing.assert_allclose(values[1], np.eye(256)[15])
    expected = np.zeros(256)
    expected[[5, 6, 9, 10]] = .5
    np.testing.assert_allclose(values[2], expected)


def test_dynamic_mask_is_applied_before_nms_and_point_budget():
    heads = dense([(8, 8, 20.), (10, 8, 19.), (24, 24, 15.)])
    full = heads.features(threshold=.05, maximum_points=1)
    assert full.points_px.tolist() == [[24., 24.]]  # single-point cell has larger softmax score
    allowed = static_background_mask((32, 32), [(7., 7., 2., 2.), (23., 23., 3., 3.)], margin_px=0)
    background = heads.features(allowed, threshold=.05, maximum_points=1)
    assert background.points_px.tolist() == [[10., 8.]]
    assert heads.features(np.zeros((32, 32), bool)).points_px.shape == (0, 2)


def test_original_image_roi_and_mask_margin_are_not_network_coordinates():
    mask = static_background_mask((64, 48), [(18., 15., 1., 1.)], margin_px=1)
    assert not mask[14:17, 17:20].any()
    features = dense([(9, 10, 20.), (23, 22, 20.)], (64, 48)).features(mask)
    assert len(features.points_px) == 1
    assert features.points_px[0, 0] > 40
    assert features.in_roi([40., 30., 10., 10.]).points_px.shape == (1, 2)
    assert not roi_mask((64, 48), [-20., -20., 5., 5.]).any()
    with pytest.raises(ValueError, match="same-image"):
        static_background_mask((64, 48), dynamic_mask=np.zeros((32, 32), bool))


def features(descriptors, points=None):
    count = len(descriptors)
    points = np.column_stack((np.arange(count) + 10, np.arange(count) + 10)) if points is None else points
    return FeatureSet(points, descriptors, np.ones(count), (64, 64))


def test_mutual_ratio_matching_preserves_permutation_and_rejects_duplicate_appearance():
    reference = features(np.eye(256, dtype=np.float32)[:5])
    order = [3, 0, 4, 1, 2]
    current = features(reference.descriptors[order])
    matches = match_features(reference, current)
    assert matches.previous_indices.tolist() == list(range(5))
    assert matches.current_indices.tolist() == [1, 3, 4, 0, 2]
    np.testing.assert_allclose(matches.distances, 0.)
    duplicated = features(np.repeat(reference.descriptors[:1], 3, axis=0))
    assert len(match_features(reference, duplicated).distances) == 0
    assert len(match_features(reference.select(slice(0, 1)), current).distances) == 0


def test_ratio_and_distance_reject_uncorrelated_descriptors():
    reference = features(np.eye(256, dtype=np.float32)[:4])
    disjoint = features(np.eye(256, dtype=np.float32)[4:8])
    assert len(match_features(reference, disjoint).distances) == 0
    with pytest.raises(ValueError, match="ratio"):
        match_features(reference, disjoint, ratio=1.5)


def test_feature_distribution_distinguishes_line_from_spatial_coverage():
    line = features(np.eye(256, dtype=np.float32)[:4])
    square = features(line.descriptors, [[8., 8.], [56., 8.], [56., 56.], [8., 56.]])
    assert feature_quality(line)["feature_coverage"] == 0.
    assert feature_quality(square)["feature_coverage"] == .5625


def test_corrupt_head_nonunit_descriptors_and_invalid_mask_are_explicit_errors():
    with pytest.raises(ValueError, match="65-channel"):
        DenseSuperPoint(np.zeros((1, 64, 4, 4)), np.zeros((1, 256, 4, 4)), (32, 32))
    with pytest.raises(ValueError, match="normalized"):
        features(np.zeros((3, 256)))
    with pytest.raises(ValueError, match="boolean"):
        dense([]).features(np.zeros((32, 32), np.uint8))
    with pytest.raises(ValueError, match="positive"):
        roi_mask((32, 32), [0, 0, -1, 2])


def test_manifest_rejects_missing_model_hash_mismatch_and_unsupported_layout(tmp_path):
    manifest = {"schema_version": 1, "architecture": "superpoint_dense_v1", "model_file": "missing.onnx",
                "sha256": "0" * 64, "input_name": "image", "detector_output": "logits",
                "descriptor_output": "descriptors", "input_size_wh": [32, 32],
                "descriptor_sampling": "lightglue_v1", "source": "synthetic_contract_test", "license": "test_only"}
    path = tmp_path / "model.json"
    path.write_text(json.dumps(manifest))
    with pytest.raises(FileNotFoundError):
        SuperPointOnnx(path)
    model = tmp_path / "missing.onnx"
    model.write_bytes(b"not_a_network; test hash rejection before model parsing")
    with pytest.raises(ValueError, match="SHA256"):
        SuperPointOnnx(path)
    manifest["sha256"] = hashlib.sha256(model.read_bytes()).hexdigest()
    manifest["descriptor_sampling"] = "unspecified"
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="unsupported"):
        SuperPointOnnx(path)


def test_mit_descriptor_sampling_uses_half_pixel_align_corners_false():
    logits = np.zeros((1, 65, 2, 2), np.float32)
    descriptors = np.zeros((1, 256, 2, 2), np.float32)
    descriptors[0, 0] = [[1, 0], [1, 0]]
    descriptors[0, 1] = [[0, 1], [0, 1]]
    model = DenseSuperPoint(logits, descriptors, (16, 16), "superpoint_mit_v1")
    sampled, valid = model._sample_descriptors(np.array([[3.5, 3.5], [7.5, 3.5], [11.5, 3.5]]))
    assert valid.all()
    np.testing.assert_allclose(sampled[:, :2], [[1, 0], [2 ** -.5, 2 ** -.5], [0, 1]], atol=1e-6)


def test_superpoint_scale_preserves_camera_coordinate_contract_after_downsampling():
    class Network:
        def run(self, blob):
            assert blob.shape == (1, 1, 32, 32)
            logits = np.zeros((1, 65, 4, 4), np.float32)
            descriptors = np.zeros((1, 256, 4, 4), np.float32)
            descriptors[:, 0] = 1.
            return [logits, descriptors]

    model = SuperPointOnnx.__new__(SuperPointOnnx)
    model.manifest = {"input_size_wh": [32, 32], "descriptor_sampling": "superpoint_mit_v1"}
    model.network = Network()
    dense = model.infer(np.zeros((64, 64), np.uint8), image_scale=.5)
    assert dense.image_size_wh == (64, 64)
    with pytest.raises(ValueError, match="scale"):
        model.infer(np.zeros((64, 64), np.uint8), image_scale=.2)


def test_dynamic_superpoint_scale_changes_network_tensor_shape_without_changing_camera_coordinates():
    class Network:
        def run(self, blob):
            assert blob.shape == (1, 1, 24, 32)
            return [np.zeros((1, 65, 3, 4), np.float32), np.ones((1, 256, 3, 4), np.float32)]

    model = SuperPointOnnx.__new__(SuperPointOnnx)
    model.manifest = {"input_size_wh": [64, 48], "descriptor_sampling": "superpoint_mit_v1", "dynamic_input": True}
    model.network = Network()
    dense = model.infer(np.zeros((96, 128), np.uint8), image_scale=.25)
    assert dense.image_size_wh == (128, 96)
    assert dense.logits.shape == (1, 65, 3, 4)
