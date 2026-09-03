from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from nexus_coord_transform.geometry import quaternion_to_matrix

def test_target_launch_exists_without_hardware_source():
    launch_file = Path(__file__).parents[1] / "launch" / "target_localization.launch.py"
    content = launch_file.read_text(encoding="utf-8")
    assert "nexus_vision_localization" in content
    assert "apriltag_localization.launch.py" in content
    assert "marker_size_m" in content
    assert "nexus_fusion_localization" in content
    assert "nexus_viz_dashboard" in content
    assert "motion_001" not in content


def test_target_launch_constructs_real_nodes_and_rviz():
    import importlib.util
    launch_file = Path(__file__).parents[1] / "launch" / "target_localization.launch.py"
    spec = importlib.util.spec_from_file_location("target_localization_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    entities = list(description.entities)
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    arguments = [entity for entity in entities if isinstance(entity, DeclareLaunchArgument)]
    assert any(node.node_package == "nexus_coord_transform" for node in nodes)
    assert any(node.node_package == "nexus_fusion_localization" for node in nodes)
    assert any(node.node_package == "rviz2" for node in nodes)
    assert {argument.name for argument in arguments} >= {
        "transform_file", "use_rviz", "max_pair_delta_ms"}


def test_pre_hardware_transform_file_is_simulation_only_and_matches_simulation_sources():
    import math
    import xml.etree.ElementTree as ET

    import numpy as np
    import yaml

    repository_root = Path(__file__).resolve().parents[5]
    transform_file = Path(__file__).parents[1] / "config" / "pre_hardware_transforms.yaml"
    document = yaml.safe_load(transform_file.read_text(encoding="utf-8"))
    assert document["calibration_version"] == "1.1"
    assert document["data_status"] == "simulation_only_pending_hardware_calibration"
    assert document["map_definition"]["survey_status"] == "simulation_only_not_surveyed"
    assert all(item["status"] == "simulation_design_value_not_measured"
               for item in document["transforms"])

    scene = yaml.safe_load((repository_root / "simulation" / "sandbox_scene.yaml").read_text(encoding="utf-8"))
    assert scene["unit"] == "mm"
    assert document["map_definition"]["unit"] == "m"
    assert document["map_definition"]["deck_height_m"] == scene["dimensions"]["deck_height"] / 1000.0
    assert document["map_definition"]["envelope_x_m"] == scene["dimensions"]["width_x"] / 1000.0
    assert document["map_definition"]["envelope_y_m"] == scene["dimensions"]["length_y"] / 1000.0

    # map -> base_link is a moving platform pose and must never be broadcast statically.
    assert all(item["child"] != "base_link" for item in document["transforms"])
    assert any(item["parent"] == "map" and item["child"] == "base_link"
               for item in document["dynamic_transforms"])

    entry = next(item for item in document["transforms"]
                 if item["child"] == "nexus_uav_camera_optical_frame")
    assert entry["parent"] == "base_link"
    assert math.isclose(float(np.linalg.norm(entry["rotation_xyzw"])), 1.0, abs_tol=1e-9)
    world = ET.parse(repository_root / "simulation" / "nexus_sandbox_imx219.world")
    pose = [float(value) for value in world.findtext(".//sensor[@name='imx219']/pose").split()]
    assert entry["translation"] == pytest.approx(pose[:3], abs=1e-9)
    # base_link (REP-103 x forward / y left / z up) -> optical (x right / y down / z forward)
    # is the axis swap below, composed with the pitch of the simulated mount.
    swap = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    pitch = pose[4]
    mount = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                      [0.0, 1.0, 0.0],
                      [-math.sin(pitch), 0.0, math.cos(pitch)]])
    assert quaternion_to_matrix(entry["rotation_xyzw"]) == pytest.approx(mount @ swap, abs=1e-9)
