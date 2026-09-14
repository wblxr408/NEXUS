"""Unit contract for external, checksum-bound edge asset staging."""

import hashlib
import importlib.util
import json
from pathlib import Path


def _load():
    path = Path(__file__).parents[1] / "prepare_edge_runtime_bundle.py"
    spec = importlib.util.spec_from_file_location("prepare_edge_runtime_bundle", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stage_copies_model_and_rewrites_local_manifest(tmp_path):
    module = _load()
    source = tmp_path / "source"
    source.mkdir()
    model = source / "model.onnx"
    model.write_bytes(b"synthetic model fixture")
    manifest = source / "manifest.json"
    manifest.write_text(json.dumps({"model_file": model.name, "sha256": hashlib.sha256(model.read_bytes()).hexdigest()}), encoding="utf-8")
    staged = Path(module.stage("lite_detector", manifest, tmp_path / "bundle"))
    document = json.loads(staged.read_text(encoding="utf-8"))
    copied = staged.parent / document["model_file"]
    assert copied.is_file() and module.digest(copied) == document["sha256"]
