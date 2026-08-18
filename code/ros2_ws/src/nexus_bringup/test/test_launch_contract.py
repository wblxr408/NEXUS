from pathlib import Path


def test_target_launch_exists_without_hardware_source():
    launch_file = Path(__file__).parents[1] / "launch" / "target_localization.launch.py"
    content = launch_file.read_text(encoding="utf-8")
    assert "nexus_vision_localization" in content
    assert "nexus_fusion_localization" in content
    assert "nexus_viz_dashboard" in content
    assert "motion_001" not in content
