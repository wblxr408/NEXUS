import hashlib
import json

import pytest

from calibration.register_reference_assets import attach_reference_assets, main


def registry():
    return {
        "schema_version": 1,
        "reference_status": "geometry_registered_reference_images_pending",
        "instances": [{
            "instance_id": "J01-NW-HIGH",
            "reference_capture": {
                "state": "pending",
                "required_views": ["nadir", "oblique_north", "oblique_south"],
                "asset_paths": [],
            },
        }],
    }


def manifest(captures):
    return {"schema_version": 1, "capture_session_id": "E040_capture", "calibration_id": "imx219_v03",
            "captures": captures}


def test_attach_reference_assets_keeps_truth_id_view_calibration_and_digest(tmp_path):
    image = tmp_path / "nadir.png"
    image.write_bytes(b"reference pixels")
    result = attach_reference_assets(
        registry(), manifest([{"instance_id": "J01-NW-HIGH", "view": "nadir", "asset_path": image.name}]),
        manifest_path=tmp_path / "capture_manifest.json")
    capture = result["instances"][0]["reference_capture"]
    assert capture["state"] == "partial"
    assert capture["asset_paths"] == [str(image)]
    assert capture["assets"][0]["calibration_id"] == "imx219_v03"
    assert capture["assets"][0]["sha256"] == hashlib.sha256(b"reference pixels").hexdigest()
    assert result["reference_status"] == "geometry_registered_reference_images_pending"


def test_all_required_views_change_only_that_instance_to_captured(tmp_path):
    captures = []
    for view in ("nadir", "oblique_north", "oblique_south"):
        image = tmp_path / f"{view}.jpg"
        image.write_bytes(view.encode())
        captures.append({"instance_id": "J01-NW-HIGH", "view": view, "asset_path": image.name})
    result = attach_reference_assets(registry(), manifest(captures), manifest_path=tmp_path / "manifest.json")
    assert result["instances"][0]["reference_capture"]["state"] == "captured"
    assert result["reference_status"] == "reference_images_registered"


@pytest.mark.parametrize("capture, reason", [
    ({"instance_id": "UNKNOWN", "view": "nadir", "asset_path": "x.png"}, "not registered"),
    ({"instance_id": "J01-NW-HIGH", "view": "side", "asset_path": "x.png"}, "not required"),
])
def test_unknown_id_or_view_is_rejected_before_an_asset_can_be_registered(tmp_path, capture, reason):
    (tmp_path / "x.png").write_bytes(b"x")
    with pytest.raises(ValueError, match=reason):
        attach_reference_assets(registry(), manifest([capture]), manifest_path=tmp_path / "manifest.json")


def test_cli_creates_a_new_registry_revision_and_refuses_canonical_overwrite(tmp_path, capsys):
    path = tmp_path / "registry.json"
    manifest_path = tmp_path / "capture.json"
    image = tmp_path / "nadir.png"
    output = tmp_path / "revision.json"
    path.write_text(json.dumps(registry()), encoding="utf-8")
    image.write_bytes(b"reference")
    manifest_path.write_text(json.dumps(manifest([
        {"instance_id": "J01-NW-HIGH", "view": "nadir", "asset_path": image.name}
    ])), encoding="utf-8")
    main(["--registry", str(path), "--capture-manifest", str(manifest_path), "--output", str(output)])
    assert json.loads(capsys.readouterr().out)["output"] == str(output)
    assert json.loads(output.read_text(encoding="utf-8"))["instances"][0]["reference_capture"]["state"] == "partial"
    with pytest.raises(ValueError, match="new registry revision"):
        main(["--registry", str(path), "--capture-manifest", str(manifest_path), "--output", str(path)])
