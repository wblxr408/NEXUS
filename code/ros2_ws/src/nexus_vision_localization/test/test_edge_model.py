from types import SimpleNamespace

import numpy as np
import pytest

from nexus_vision_localization.edge_model import OnnxRuntimeModel, create_model


def test_edge_manifest_requires_explicit_cpu_onnx():
    with pytest.raises(ValueError, match="explicit CPU"):
        OnnxRuntimeModel("unused", {"model_format": "onnx", "inference_device": "cuda:0", "precision": "int8"})
    with pytest.raises(ValueError, match="supported torchscript or ONNX"):
        create_model("unused", {"model_format": "ncnn"})


def test_edge_runner_binds_cpu_and_requires_float32(monkeypatch):
    class Session:
        def __init__(self, *_args, **_kwargs):
            pass
        def get_providers(self):
            return ["CPUExecutionProvider"]
        def get_inputs(self):
            return [SimpleNamespace(name="image")]
        def get_outputs(self):
            return [SimpleNamespace(name="output")]
        def run(self, names, feeds):
            assert names == ["output"] and feeds["image"].dtype == np.float32
            return [np.zeros((1, 1), np.float32)]
    monkeypatch.setitem(__import__("sys").modules, "onnxruntime",
                    SimpleNamespace(SessionOptions=lambda: SimpleNamespace(), InferenceSession=Session))
    runner = OnnxRuntimeModel("model.onnx", {"model_format": "onnx", "inference_device": "cpu",
                                               "precision": "int8", "input_name": "image"})
    assert runner.run(np.zeros((1, 1), np.float32))[0].shape == (1, 1)
    with pytest.raises(ValueError, match="float32"):
        runner.run(np.zeros((1, 1), np.uint8))
