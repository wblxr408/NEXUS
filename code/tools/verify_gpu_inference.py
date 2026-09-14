"""Check CUDA kernels, numerical agreement and synchronized GPU latency."""

import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np
import torch

from nexus_vision_localization.object_detector import ObjectDetectorOnnx, decode_yolo, letterbox
from nexus_vision_localization.superpoint_frontend import SuperPointOnnx


def measure(function, repeats=30):
    for _ in range(5):
        function()
    durations = []
    for _ in range(repeats):
        torch.cuda.synchronize()
        start = perf_counter()
        function()
        torch.cuda.synchronize()
        durations.append((perf_counter() - start) * 1000)
    return dict(samples=repeats, median_ms=float(np.median(durations)),
                p95_ms=float(np.percentile(durations, 95)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--references", type=Path, required=True)
    args = parser.parse_args()
    detector = ObjectDetectorOnnx(args.assets / "detector/detector_manifest.json")
    superpoint = SuperPointOnnx(args.assets / "superpoint/superpoint_manifest.json")
    report = dict(torch=torch.__version__, cuda=torch.version.cuda, cudnn=torch.backends.cudnn.version(),
                  gpu=detector.network.device_name, device=str(detector.network.device),
                  precision="FP32; TF32 disabled", cpu_fallback=False, parity=[])
    cases = sorted(args.references.glob("*.npz"))
    assert len(cases) >= 8
    for path in cases:
        data = np.load(path)
        actual = detector.network.run(data["detector_blob"])[0]
        np.testing.assert_allclose(actual, data["detector_output"], rtol=1e-4, atol=.002)
        image = data["image"]
        _, transform = letterbox(image, (640, 640))
        expected = decode_yolo(data["detector_output"], (image.shape[1], image.shape[0]), transform,
                               detector.manifest["class_names"], architecture="yolo_v11_detection", layout="channels_first")
        detected = detector.infer(image)
        assert [d.class_id for d in detected] == [d.class_id for d in expected]
        np.testing.assert_allclose([d.bbox_xywh_px for d in detected], [d.bbox_xywh_px for d in expected], atol=.01)
        case = dict(file=path.name, detector_max_abs=float(np.max(abs(actual - data["detector_output"]))),
                    detection_count=len(detected))
        if "superpoint_blob" in data:
            logits, descriptors = superpoint.network.run(data["superpoint_blob"])
            np.testing.assert_allclose(logits, data["logits"], atol=1e-4, rtol=1e-4)
            np.testing.assert_allclose(descriptors, data["descriptors"], atol=1e-4, rtol=1e-4)
            case["superpoint_logits_max_abs"] = float(np.max(abs(logits - data["logits"])))
            case["keypoints"] = len(superpoint.infer(image).features().points_px)
        report["parity"].append(case)

    data = np.load(cases[0])
    image = data["image"]
    detector_blob, superpoint_blob = data["detector_blob"], data["superpoint_blob"]
    report["detector_forward_and_transfer"] = measure(lambda: detector.network.run(detector_blob))
    report["superpoint_forward_and_transfer"] = measure(lambda: superpoint.network.run(superpoint_blob))
    report["detector_with_pre_postprocessing"] = measure(lambda: detector.infer(image))
    report["superpoint_with_pre_postprocessing"] = measure(lambda: superpoint.infer(image).features())
    with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA]) as profile:
        detector.infer(image)
        superpoint.infer(image).features()
        torch.cuda.synchronize()
    profile.export_chrome_trace(str(args.assets / "cuda_trace.json"))
    gpu_events = [event for event in profile.events() if str(event.device_type) == "DeviceType.CUDA"]
    kernels = [event.name for event in gpu_events if "memcpy" not in event.name.lower() and "memset" not in event.name.lower()]
    trace = json.loads((args.assets / "cuda_trace.json").read_text())
    launches = [event for event in trace["traceEvents"]
                if event.get("name") in {"cudaLaunchKernel", "cuLaunchKernel", "cudaLaunchKernelExC"}]
    assert launches, "No CUDA kernel launches were recorded"
    report["cuda_kernel_launches"] = len(launches)
    report["kernel_activity_trace_available"] = bool(kernels)
    report["profiling_limitation"] = (
        None if kernels else
        "WSL profiler records CUDA launch APIs but does not supply individual GPU activity events")
    report["cuda_compute_events"] = len(kernels)
    report["resident_input_cuda_event_ms"] = {}
    with torch.inference_mode():
        networks = [("detector", detector.network, detector_blob),
                    ("superpoint", superpoint.network, superpoint_blob)]
        for name, network, blob in networks:
            tensor = torch.from_numpy(blob).to(network.device)
            samples = []
            for _ in range(30):
                start = torch.cuda.Event(enable_timing=True)
                end = torch.cuda.Event(enable_timing=True)
                start.record()
                output = network.model(tensor)
                end.record()
                values = output if isinstance(output, tuple) else (output,)
                assert all(value.is_cuda for value in values)
                end.synchronize()
                samples.append(start.elapsed_time(end))
            assert min(samples) > 0
            report["resident_input_cuda_event_ms"][name] = float(np.median(samples))
    report["cuda_kernel_examples"] = list(dict.fromkeys(kernels))[:10]
    report["peak_allocated_mib"] = torch.cuda.max_memory_allocated() / (1024 ** 2)
    (args.assets / "gpu_validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k not in {"parity", "cuda_kernel_examples"}}, indent=2))


if __name__ == "__main__":
    main()
