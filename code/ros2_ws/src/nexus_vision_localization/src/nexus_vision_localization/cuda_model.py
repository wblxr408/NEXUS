"""Strict CUDA model execution shared by the detector and SuperPoint frontend."""

import numpy as np


class CudaModel:
    def __init__(self, path, manifest):
        if manifest.get("model_format") != "torchscript" or manifest.get("precision") != "float32":
            raise ValueError("CUDA TorchScript FP32 manifest required; use the GPU model assets")
        device_name = manifest.get("inference_device", "")
        if not isinstance(device_name, str) or not device_name.startswith("cuda:"):
            raise ValueError("GPU inference requires an explicit cuda device; CPU fallback is disabled")
        try:
            import torch
        except ImportError as error:
            raise RuntimeError("CUDA PyTorch runtime missing; use run_with_camera_models.sh") from error
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA unavailable; CPU fallback is disabled")
        self.torch = torch
        self.device = torch.device(device_name)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        self.model = torch.jit.load(str(path), map_location=self.device).eval()
        self.device_name = torch.cuda.get_device_name(self.device)
        width, height = manifest["input_size_wh"]
        channels = 1 if manifest["architecture"] == "superpoint_dense_v1" else 3
        # TorchScript optimizes its early executions. Complete those before ROS
        # subscriptions begin, so initialization cannot make a live frame stale.
        with torch.inference_mode():
            sample = torch.zeros(1, channels, height, width, device=self.device)
            for _ in range(3):
                self.model(sample)
            torch.cuda.synchronize(self.device)

    def run(self, blob):
        if blob.dtype != np.float32:
            raise ValueError("GPU model input must be float32")
        torch = self.torch
        with torch.inference_mode():
            tensor = torch.from_numpy(np.ascontiguousarray(blob)).to(self.device)
            result = self.model(tensor)
            outputs = result if isinstance(result, tuple) else (result,)
            if not all(isinstance(value, torch.Tensor) and value.device == self.device for value in outputs):
                raise RuntimeError("model did not produce outputs on the requested CUDA device")
            # Synchronous copy includes GPU completion; downstream geometry uses NumPy.
            return [value.cpu().numpy() for value in outputs]
