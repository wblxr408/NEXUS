from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

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


def test_pre_hardware_transform_file_contains_no_guessed_geometry():
    transform_file = Path(__file__).parents[1] / "config" / "pre_hardware_transforms.yaml"
    content = transform_file.read_text(encoding="utf-8")
    assert "transforms: []" in content
    assert "TBD" in content
