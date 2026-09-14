"""Synthetic YOLO head checks, without claiming trained-model inference."""

import json

import numpy as np
import pytest

from nexus_vision_localization.object_detector import ObjectDetectorOnnx, decode_yolo, letterbox


def test_letterbox_preserves_aspect_ratio_rgb_and_padding():
    image = np.zeros((160, 320, 3), np.uint8)
    image[:, :, 0] = 255  # BGR blue becomes the third input channel.
    blob, transform = letterbox(image, (640, 640))
    assert blob.shape == (1, 3, 640, 640)
    assert transform == (2., 2., 0, 160)
    np.testing.assert_allclose(blob[0, :, 0, 0], 114 / 255., atol=1e-7)
    np.testing.assert_allclose(blob[0, :, 200, 100], [0., 0., 1.])


@pytest.mark.parametrize("architecture,layout", [("yolo_v11_detection", "channels_first"), ("yolo_v8_detection", "channels_first"), ("yolo_v5_detection", "anchors_first")])
def test_yolo_boxes_recover_original_pixels_and_nms_is_per_class(architecture, layout):
    rows = np.array([[160., 320., 160., 80., .9, .1],
                     [161., 320., 160., 80., .8, .1],
                     [160., 320., 160., 80., .1, .85]])
    if architecture == "yolo_v5_detection":
        rows = np.column_stack((rows[:, :4], np.ones(3), rows[:, 4:]))
    raw = rows.T[None] if layout == "channels_first" else rows[None]
    decoded = decode_yolo(raw, (320, 160), (2., 2., 0, 160), ["car", "person"], architecture=architecture, layout=layout)
    assert [item.class_id for item in decoded] == [0, 1]
    np.testing.assert_allclose(decoded[0].bbox_xywh_px, [40., 60., 80., 40.])
    assert decoded[1].label == "person"
    assert "track_id" not in decoded[0].as_dict()


def test_v5_objectness_is_multiplied_and_boxes_are_clipped():
    raw = np.array([[[0., 0., 40., 40., .5, .8], [50., 50., 20., 20., .1, .9]]])
    output = decode_yolo(raw, (100, 100), (1., 1., 0., 0.), ["target"],
                         architecture="yolo_v5_detection", layout="anchors_first")
    assert len(output) == 1 and output[0].confidence == pytest.approx(.4)
    assert output[0].bbox_xywh_px == (0., 0., 20., 20.)


def test_wrong_head_layout_and_unactivated_scores_are_rejected():
    with pytest.raises(ValueError, match="channels"):
        decode_yolo(np.zeros((1, 5, 10)), (100, 100), (1, 1, 0, 0), ["a", "b"],
                    architecture="yolo_v8_detection", layout="channels_first")
    raw = np.array([[[50., 50., 20., 20., -2.]]])
    with pytest.raises(ValueError, match="probabilities"):
        decode_yolo(raw, (100, 100), (1, 1, 0, 0), ["a"], architecture="yolo_v8_detection", layout="anchors_first")
    with pytest.raises(ValueError, match="8-bit BGR"):
        letterbox(np.zeros((10, 10)), (32, 32))


def test_model_manifest_does_not_allow_missing_or_mismatched_assets(tmp_path):
    document = {"schema_version": 1, "architecture": "yolo_v8_detection", "model_file": "detector.onnx",
                "sha256": "0" * 64, "input_name": "images", "output_name": "output0", "input_size_wh": [640, 640],
                "padding_value": 114, "layout": "channels_first", "class_names": ["target"],
                "source": "synthetic_contract_test", "license": "test_only"}
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(document))
    with pytest.raises(FileNotFoundError):
        ObjectDetectorOnnx(path)
    (tmp_path / "detector.onnx").write_bytes(b"synthetic hash-validation fixture, not an ONNX graph")
    with pytest.raises(ValueError, match="SHA256"):
        ObjectDetectorOnnx(path)


def test_detector_scale_reduces_source_image_and_maps_box_back_to_camera_pixels():
    class Network:
        def run(self, blob):
            assert blob.shape == (1, 3, 32, 32)
            return [np.array([[[16.], [16.], [8.], [8.], [.9]]], dtype=np.float32)]

    model = ObjectDetectorOnnx.__new__(ObjectDetectorOnnx)
    model.manifest = {"input_size_wh": [32, 32], "padding_value": 114, "class_names": ["target"],
                      "architecture": "yolo_v11_detection", "layout": "channels_first"}
    model.network = Network()
    output = model.infer(np.zeros((64, 64, 3), np.uint8), image_scale=.5)
    assert len(output) == 1
    np.testing.assert_allclose(output[0].bbox_xywh_px, [24., 24., 16., 16.])
