import yaml

from calibration.build_physical_sandbox_catalog import build_catalog, build_model_reference_registry


def test_catalog_expands_all_small_layout_primitives():
    with open("simulation/sandbox_scene.yaml", encoding="utf-8") as stream:
        scene = yaml.safe_load(stream)
    with open("experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/sandbox_physical_reference_v01.yaml", encoding="utf-8") as stream:
        reference = yaml.safe_load(stream)
    with open("experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/traffic_light_photo_survey_v01.yaml", encoding="utf-8") as stream:
        traffic_survey = yaml.safe_load(stream)
    catalog = build_catalog(scene, reference, traffic_survey)
    assert catalog["status"] == "operational_ground_truth_user_accepted"
    assert catalog["category_counts"]["tree"] == 204
    assert catalog["category_counts"]["road_marking"] == 186
    assert catalog["category_counts"]["traffic_light"] == 48
    assert catalog["category_counts"]["traffic_cone"] == 32
    assert catalog["category_counts"]["tank"] == 15
    assert next(item for item in catalog["items"] if item["id"] == "TR-F02-01")["height_m"] == 0.06
    assert next(item for item in catalog["items"] if item["id"] == "NW-T01")["height_m"] == 0.16
    assert next(item for item in catalog["items"] if item["id"] == "J01-NW-HIGH")["height_m"] == 0.22
    assert next(item for item in catalog["items"] if item["id"] == "J06-SE-LOW")["height_m"] == 0.12


def test_model_reference_registry_preserves_operational_truth_without_faking_assets():
    with open("simulation/sandbox_scene.yaml", encoding="utf-8") as stream:
        scene = yaml.safe_load(stream)
    with open("experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/sandbox_physical_reference_v01.yaml", encoding="utf-8") as stream:
        reference = yaml.safe_load(stream)
    with open("experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/traffic_light_photo_survey_v01.yaml", encoding="utf-8") as stream:
        traffic_survey = yaml.safe_load(stream)
    registry = build_model_reference_registry(build_catalog(scene, reference, traffic_survey))
    assert registry["coordinate_frame"] == "physical_sandbox_map_v01"
    assert registry["reference_status"] == "geometry_registered_reference_images_pending"
    assert len(registry["instances"]) == 517
    lamp = next(item for item in registry["instances"] if item["instance_id"] == "J01-NW-HIGH")
    assert lamp["role"] == "candidate_target"
    assert lamp["height_m"] == 0.22
    assert lamp["reference_capture"]["asset_paths"] == []
