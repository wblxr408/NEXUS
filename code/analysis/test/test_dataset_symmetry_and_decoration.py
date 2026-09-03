"""Dataset-side checks for the ADR-009 caliber changes (L0-1, L0-6, L0-7).

These run the real generator on a small config rather than asserting on the
committed 3280x2464 corpus, so a caliber regression fails here instead of
surfacing as an unexplained metric change three experiments later.
"""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).parents[1]
REPOSITORY = ROOT.parents[1]
sys.path.insert(0, str(ROOT))

from evaluation.pose_metrics import model_is_symmetric, symmetry_rotations


def _generator():
    path = REPOSITORY / "simulation/generate_target_detection_dataset.py"
    spec = importlib.util.spec_from_file_location("generate_target_detection_dataset", path)
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(path.parent))
    spec.loader.exec_module(module)
    return module


CATALOG = yaml.safe_load((REPOSITORY / "simulation/target_catalog_v01.yaml").read_text(encoding="utf-8"))
EXPECTED_SYMMETRY = {1: "continuous_z", 2: "continuous_x_and_z2", 3: "continuous_z", 4: "none",
                     5: "discrete_z6", 6: "discrete_z2", 7: "none", 8: "discrete_z4",
                     9: "continuous_z", 10: "discrete_z2"}


def test_catalog_symmetry_matches_the_adr_ruling():
    labels = {int(target["object_id"]): target["symmetry"]["label"] for target in CATALOG["targets"]}
    assert labels == EXPECTED_SYMMETRY


def test_catalog_base_z_is_consistent_with_position_and_height():
    for target in CATALOG["targets"]:
        expected = float(target["position_mm"][2]) - float(target["size_mm"][2]) / 2.0
        assert float(target["base_z_mm"]) == pytest.approx(expected, abs=1e-9)
    # Nine of ten sit below the 50 mm deck; the support factor must not assume
    # z_base = deck (design section 4.3).
    sunk = [t for t in CATALOG["targets"] if float(t["base_z_mm"]) < float(CATALOG["support_deck_z_mm"])]
    assert len(sunk) == 9


def test_symmetry_block_expands_to_the_expected_bop_group_sizes():
    generator = _generator()
    expected_rotations = {1: 72, 2: 144, 3: 72, 4: 1, 5: 6, 6: 2, 7: 1, 8: 4, 9: 72, 10: 2}
    for target in CATALOG["targets"]:
        fields = generator._bop_symmetries(target["symmetry"])
        object_id = int(target["object_id"])
        assert len(symmetry_rotations(fields)) == expected_rotations[object_id]
        assert model_is_symmetric(fields) is (expected_rotations[object_id] > 1)
        for value in fields.get("symmetries_discrete", []):
            assert len(value) == 16 and np.allclose(np.reshape(value, (4, 4))[3], [0.0, 0.0, 0.0, 1.0])
        for entry in fields.get("symmetries_continuous", []):
            assert set(entry) == {"axis", "offset"} and len(entry["axis"]) == 3


def test_decoration_paints_faces_without_touching_the_mesh():
    generator = _generator()
    scale = 0.001
    for target in CATALOG["targets"]:
        size = np.asarray(target["size_mm"], dtype=float) * scale
        vertices, faces = generator._target_mesh(target["shape"], size)
        colors = generator._decoration_face_colors(target["decoration"], vertices, faces, target["color_rgb"])
        assert colors is not None and len(colors) == len(faces), target["decoration"]
        assert len(np.unique(colors, axis=0)) == 2, f"{target['decoration']} selected no faces"
        # Same call again must not mutate the mesh the PLY is written from.
        assert generator._target_mesh(target["shape"], size)[0].shape == vertices.shape


def test_pose_diversity_widens_relative_rotation_coverage(tmp_path):
    """v01 stays a single orientation per target; v02 varies it per trajectory."""
    generator = _generator()
    rng = np.random.default_rng(0)
    fixed = {generator._target_rotation(10.0, rng, {})[0, 0] for _ in range(4)}
    assert len(fixed) == 1
    diverse = {generator._target_rotation(10.0, rng, {"target_orientation_mode": "per_trajectory",
                                                     "target_yaw_jitter_deg": [-180.0, 180.0],
                                                     "target_tilt_deg": 12.0})[0, 0] for _ in range(4)}
    assert len(diverse) == 4
    with pytest.raises(ValueError):
        generator._target_rotation(10.0, rng, {"target_orientation_mode": "per_frame"})
    # Roll is applied about the optical axis, so the forward row is preserved.
    position, look_at = np.array([2.0, 2.0, 3.5]), np.array([2.0, 2.35, 0.1])
    plain = generator._look_at(position, look_at)
    rolled = generator._look_at(position, look_at, 40.0)
    assert np.allclose(plain[2], rolled[2])
    assert not np.allclose(plain[0], rolled[0])


def test_generated_models_info_carries_bop_symmetries_and_base_z(tmp_path):
    config = yaml.safe_load((REPOSITORY / "simulation/target_detection_dataset_v02.yaml").read_text(encoding="utf-8"))
    config["camera"]["image_width_px"], config["camera"]["image_height_px"] = 328, 246
    config["camera"]["K"] = [[197.8892939, 0.0, 164.5], [0.0, 197.8892939, 123.5], [0.0, 0.0, 1.0]]
    config["render"]["texture_landmark_count"] = 20
    for split in config["splits"].values():
        split["frames_per_trajectory"] = 1
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    output = tmp_path / "dataset"
    subprocess.run([sys.executable, "simulation/generate_target_detection_dataset.py",
                    "--config", str(config_path), "--out", str(output)], cwd=REPOSITORY, check=True)
    models_info = json.loads((output / "models" / "models_info.json").read_text(encoding="utf-8"))
    for target in CATALOG["targets"]:
        details = models_info[str(int(target["object_id"]))]
        assert details["symmetry"] == EXPECTED_SYMMETRY[int(target["object_id"])]
        assert details["base_z_mm"] == pytest.approx(float(target["base_z_mm"]))
        assert details["support_deck_z_mm"] == pytest.approx(float(CATALOG["support_deck_z_mm"]))
    manifest = json.loads((output / "dataset_manifest.json").read_text(encoding="utf-8"))
    assert manifest["pose_diversity"]["target_orientation_mode"] == "per_trajectory"
    assert "decoration" in " ".join(manifest["limitations"]).lower()
