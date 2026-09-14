"""Stage checksum-verified full/lite/robust assets as one deployable edge bundle.

Model binaries intentionally remain outside Git.  This tool copies explicitly
named manifests and their referenced weights into a user-selected external
directory, rewrites the manifests locally, and emits one profile consumed by
the optimized launch wrapper.  It never guesses asset paths or model roles.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil


ROLES = ("full_detector", "lite_detector", "robust_detector", "full_superpoint", "lite_superpoint")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage(role, manifest_path, root):
    source_manifest = Path(manifest_path).resolve()
    document = json.loads(source_manifest.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("model_file"), str) or not document.get("sha256"):
        raise ValueError(f"{role} manifest is incomplete")
    source_model = source_manifest.parent / document["model_file"]
    if not source_model.is_file() or digest(source_model) != document["sha256"].lower():
        raise ValueError(f"{role} model is missing or checksum-mismatched")
    destination = root / role
    destination.mkdir(parents=True)
    model = destination / source_model.name
    shutil.copy2(source_model, model)
    document["model_file"], document["sha256"] = model.name, digest(model)
    manifest = destination / "manifest.json"
    manifest.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return str(manifest)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True, help="new external runtime directory")
    for role in ROLES:
        parser.add_argument("--" + role.replace("_", "-"), type=Path, required=True)
    parser.add_argument("--identity-transformer", type=Path, required=True)
    parser.add_argument("--policy-profile", type=Path, default=None)
    parser.add_argument("--rolling-shutter-readout-s", type=float, default=0.)
    parser.add_argument("--exposure-time-s", type=float, default=0.)
    args = parser.parse_args()
    if args.output.exists() or min(args.rolling_shutter_readout_s, args.exposure_time_s) < 0:
        raise ValueError("output must be new and timing values must be nonnegative")
    identity = args.identity_transformer.resolve()
    if not identity.is_file():
        raise ValueError("identity transformer checkpoint is required")
    args.output.mkdir(parents=True)
    manifests = {role: stage(role, getattr(args, role), args.output) for role in ROLES}
    identity_destination = args.output / "identity_transformer.npz"
    shutil.copy2(identity, identity_destination)
    profile = {"schema_version": 1, "assets": manifests,
               "identity_transformer_checkpoint": str(identity_destination),
               "rolling_shutter_readout_s": args.rolling_shutter_readout_s,
               "exposure_time_s": args.exposure_time_s}
    if args.policy_profile is not None:
        policy = args.policy_profile.resolve()
        if not policy.is_file():
            raise ValueError("policy profile does not exist")
        staged = args.output / "adaptive_policy.json"
        shutil.copy2(policy, staged)
        profile["policy_profile_file"] = str(staged)
    output = args.output / "edge_runtime_profile.json"
    output.write_text(json.dumps(profile, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(str(output))


if __name__ == "__main__":
    main()
