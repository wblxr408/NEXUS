"""Attach acquired visual-reference assets to a frozen instance registry.

The canonical physical-map registry stays immutable.  A capture session is
written to a new registry revision so every reference image keeps its stable
instance ID, declared camera calibration, requested view and SHA-256 digest.
"""

import argparse
import hashlib
import json
from pathlib import Path


IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png"}


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value.strip()


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def attach_reference_assets(registry, capture_manifest, *, manifest_path):
    """Return a new registry revision with verified image assets attached.

    ``capture_manifest`` is intentionally separate from the image bytes.  This
    lets the capture process produce a small reviewable manifest while keeping
    original images in the project's raw-data area.
    """
    if not isinstance(registry, dict) or registry.get("schema_version") != 1:
        raise ValueError("registry requires schema_version=1")
    instances = registry.get("instances")
    if not isinstance(instances, list) or not instances:
        raise ValueError("registry requires nonempty instances")
    if not isinstance(capture_manifest, dict) or capture_manifest.get("schema_version") != 1:
        raise ValueError("capture manifest requires schema_version=1")
    session_id = _text(capture_manifest.get("capture_session_id"), "capture_session_id")
    calibration_id = _text(capture_manifest.get("calibration_id"), "calibration_id")
    entries = capture_manifest.get("captures")
    if not isinstance(entries, list) or not entries:
        raise ValueError("capture manifest requires nonempty captures")

    by_id = {item.get("instance_id"): item for item in instances if isinstance(item, dict)}
    if len(by_id) != len(instances) or any(not isinstance(key, str) or not key for key in by_id):
        raise ValueError("registry instance IDs must be unique nonempty strings")
    pending = {}
    root = manifest_path.parent
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("capture entry must be an object")
        identifier = _text(entry.get("instance_id"), "instance_id")
        view = _text(entry.get("view"), "view")
        raw_path = _text(entry.get("asset_path"), "asset_path")
        instance = by_id.get(identifier)
        if instance is None:
            raise ValueError(f"capture instance_id is not registered: {identifier}")
        required = instance.get("reference_capture", {}).get("required_views")
        if not isinstance(required, list) or view not in required:
            raise ValueError(f"capture view is not required for {identifier}: {view}")
        path = Path(raw_path)
        path = path if path.is_absolute() else (root / path).resolve()
        if path.suffix.lower() not in IMAGE_SUFFIXES or not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"capture asset must be a nonempty supported image: {raw_path}")
        key = identifier, view
        if key in pending:
            raise ValueError(f"duplicate capture for {identifier}/{view}")
        pending[key] = {
            "view": view,
            "asset_path": str(path),
            "sha256": _digest(path),
            "bytes": path.stat().st_size,
            "calibration_id": calibration_id,
            "capture_session_id": session_id,
        }

    output = json.loads(json.dumps(registry))
    for item in output["instances"]:
        identifier = item["instance_id"]
        capture = item["reference_capture"]
        existing = capture.get("assets", [])
        if not isinstance(existing, list):
            raise ValueError(f"registry assets must be a list for {identifier}")
        by_view = {asset.get("view"): asset for asset in existing if isinstance(asset, dict)}
        for (entry_id, view), asset in pending.items():
            if entry_id == identifier:
                by_view[view] = asset
        assets = [by_view[view] for view in capture["required_views"] if view in by_view]
        capture["assets"] = assets
        capture["asset_paths"] = [asset["asset_path"] for asset in assets]
        if not assets:
            capture["state"] = "pending"
        elif len(assets) == len(capture["required_views"]):
            capture["state"] = "captured"
        else:
            capture["state"] = "partial"
    output["reference_status"] = "reference_images_registered" if any(
        item["reference_capture"]["state"] == "captured" for item in output["instances"]
    ) else "geometry_registered_reference_images_pending"
    output["reference_capture_manifest"] = {
        "capture_session_id": session_id,
        "calibration_id": calibration_id,
        "source": str(manifest_path.resolve()),
    }
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--capture-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not args.registry.is_file() or not args.capture_manifest.is_file():
        raise ValueError("registry and capture manifest files are required")
    if args.output.exists() or args.output.resolve() == args.registry.resolve():
        raise ValueError("output must be a new registry revision, not the canonical registry")
    registry = json.loads(args.registry.read_text(encoding="utf-8"))
    manifest = json.loads(args.capture_manifest.read_text(encoding="utf-8"))
    output = attach_reference_assets(registry, manifest, manifest_path=args.capture_manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "instances": len(output["instances"]),
                      "reference_status": output["reference_status"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
