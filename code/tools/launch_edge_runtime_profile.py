"""Launch the full localization chain from one staged edge-runtime profile."""

import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--platform-calibration", required=True)
    parser.add_argument("--vision-calibration", required=True)
    parser.add_argument("--reference-calibration", required=True)
    parser.add_argument("--input-mode", choices=("LIVE", "SIMULATION", "REPLAY"), default="LIVE")
    parser.add_argument("--run-id", default="UNREGISTERED")
    parser.add_argument("ros_arguments", nargs="*")
    args = parser.parse_args()
    document = json.loads(args.profile.read_text(encoding="utf-8"))
    assets = document.get("assets", {})
    required = {"full_detector", "lite_detector", "robust_detector", "full_superpoint", "lite_superpoint"}
    if document.get("schema_version") != 1 or set(assets) != required or not all(Path(value).is_file() for value in assets.values()):
        raise ValueError("edge runtime profile assets are incomplete")
    identity = document.get("identity_transformer_checkpoint")
    if not isinstance(identity, str) or not Path(identity).is_file():
        raise ValueError("edge runtime profile identity transformer is missing")
    values = {"platform_calibration": args.platform_calibration, "vision_calibration": args.vision_calibration,
              "reference_calibration": args.reference_calibration, "input_mode": args.input_mode, "run_id": args.run_id,
              "detector_manifest": assets["full_detector"], "lite_detector_manifest": assets["lite_detector"],
              "robust_detector_manifest": assets["robust_detector"], "superpoint_manifest": assets["full_superpoint"],
              "lite_superpoint_manifest": assets["lite_superpoint"], "identity_transformer_checkpoint": identity,
              "rolling_shutter_readout_s": document.get("rolling_shutter_readout_s", 0.),
              "exposure_time_s": document.get("exposure_time_s", 0.),
              "policy_profile_file": document.get("policy_profile_file", "")}
    command = ["ros2", "launch", "nexus_bringup", "dual_localization.launch.py"]
    command.extend(f"{key}:={value}" for key, value in values.items())
    command.extend(args.ros_arguments)
    subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
