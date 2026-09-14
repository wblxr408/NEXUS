"""Manifest-constrained model runners for GPU workstations and CPU edge hosts."""

import numpy as np

from .cuda_model import CudaModel


class OnnxRuntimeModel:
    """CPU ONNX Runtime runner; never silently selects a GPU provider."""
    def __init__(self, path, manifest):
        if manifest.get("model_format") != "onnx" or manifest.get("inference_device") not in {"cpu", "onnxruntime_cpu"}:
            raise ValueError("edge ONNX assets require model_format=onnx and explicit CPU inference_device")
        if manifest.get("precision") not in {"float32", "int8"}:
            raise ValueError("edge ONNX assets require float32 or int8 precision")
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError("ONNX Runtime is required for the configured edge model") from error
        options = ort.SessionOptions()
        options.intra_op_num_threads = int(manifest.get("intra_op_threads", 1))
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])
        if self.session.get_providers() != ["CPUExecutionProvider"]:
            raise RuntimeError("edge model did not bind exclusively to CPUExecutionProvider")
        inputs = self.session.get_inputs()
        if len(inputs) != 1 or inputs[0].name != manifest["input_name"]:
            raise ValueError("edge ONNX model input does not match manifest")
        self.input_name = inputs[0].name
        self.output_names = [value.name for value in self.session.get_outputs()]

    def run(self, blob):
        values = np.asarray(blob)
        if values.dtype != np.float32:
            raise ValueError("edge ONNX input must be float32")
        outputs = self.session.run(self.output_names, {self.input_name: np.ascontiguousarray(values)})
        if not all(isinstance(value, np.ndarray) and np.all(np.isfinite(value)) for value in outputs):
            raise RuntimeError("edge ONNX model produced invalid output")
        return outputs


def create_model(path, manifest):
    """Select only the backend explicitly declared by a verified manifest."""
    if manifest.get("model_format") == "torchscript":
        return CudaModel(path, manifest)
    if manifest.get("model_format") == "onnx":
        return OnnxRuntimeModel(path, manifest)
    raise ValueError("model manifest must declare supported torchscript or ONNX format")
