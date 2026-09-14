"""Generate PyTorch reference tensors from real assets for WSL ONNX checks."""

import argparse
import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/nexus_vision_localization/src"))
from nexus_vision_localization.object_detector import letterbox


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--superpoint-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from ultralytics import YOLO
    detector = YOLO(str(args.checkpoint)).model.eval().cpu()
    spec = importlib.util.spec_from_file_location("sp_reference", args.superpoint_dir / "superpoint_pytorch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    superpoint = module.SuperPoint().eval()
    superpoint.load_state_dict(torch.load(args.superpoint_dir / "superpoint_v6_from_tf.pth",
                                          map_location="cpu", weights_only=True))
    paths = sorted(args.images.glob("*.png"))[::10]
    if not paths:
        raise ValueError("no reference images")
    torch.set_num_threads(4)
    with torch.inference_mode():
        for index, path in enumerate(paths):
            image = cv2.imread(str(path))
            blob, _ = letterbox(image, (640, 640))
            output = detector(torch.from_numpy(blob))[0].numpy()
            result = dict(image=image, detector_blob=blob, detector_output=output)
            if index < 2:
                gray = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (640, 480), interpolation=cv2.INTER_AREA)
                sp_blob = (gray.astype(np.float32) / 255)[None, None]
                features = superpoint.backbone(torch.from_numpy(sp_blob))
                result.update(superpoint_blob=sp_blob, logits=superpoint.detector(features).numpy(),
                              descriptors=superpoint.descriptor(features).numpy())
            np.savez_compressed(args.output / f"reference_{index:02d}.npz", **result)
            print(f"REFERENCE {path.name}", flush=True)


if __name__ == "__main__":
    main()
