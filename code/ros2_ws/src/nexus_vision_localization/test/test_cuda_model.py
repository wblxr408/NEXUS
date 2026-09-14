from types import SimpleNamespace

import pytest

from nexus_vision_localization.cuda_model import CudaModel


def test_cpu_manifest_is_rejected_before_any_model_execution():
    with pytest.raises(ValueError, match="CPU fallback is disabled"):
        CudaModel("unused", dict(model_format="torchscript", precision="float32", inference_device="cpu"))


def test_old_onnx_asset_does_not_silently_use_cpu():
    with pytest.raises(ValueError, match="GPU model assets"):
        CudaModel("unused", dict(model_format="onnx"))


def test_missing_gpu_fails_without_cpu_fallback(monkeypatch):
    monkeypatch.setitem(__import__("sys").modules, "torch",
                        SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False)))
    with pytest.raises(RuntimeError, match="CUDA unavailable; CPU fallback is disabled"):
        CudaModel("unused", dict(model_format="torchscript", precision="float32", inference_device="cuda:0"))
