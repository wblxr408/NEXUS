import importlib.util
import json
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.utilities import perform_substitutions
import numpy as np
import pytest
import yaml


def launch_module():
    path = Path(__file__).parents[1] / "launch/dual_localization.launch.py"
    spec = importlib.util.spec_from_file_location("dual_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def configured_context(tmp_path):
    module = launch_module()
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None:
            action.execute(context)
    platform = {"schema_version": 1, "calibration_id": "synthetic", "calibration_status": "synthetic_test",
                "map_frame": "map", "body_frame": "base_link", "initial_velocity_sigma_mps": .1,
                "initial_accel_bias_sigma_mps2": .1, "initial_gyro_bias_sigma_rps": .01,
                "uwb": {"tag_body_m": [0., 0., 0.], "anchors_map_m": {}},
                "imu": {"frame_id": "imu_link", "rotation_body_imu": np.eye(3).tolist(), "at_body_origin_or_compensated": True,
                        "noise": {"accel_density": .08, "gyro_density": .004, "accel_bias_walk": .001,
                                  "gyro_bias_walk": .0001, "maximum_gap_s": .05}}}
    vision = {"schema_version": 1, "calibration_id": "synthetic", "calibration_status": "synthetic_test",
              "camera_frame": "camera_optical_frame", "image_rectified": True, "transform_body_camera": np.eye(4).tolist(),
              "extrinsic_covariance": np.zeros((6, 6)).tolist(), "pixel_sigma_px": .5}
    reference = {**vision, "reference_frame": "map", "tag_family": "36h11", "reference_covariance": np.zeros((6, 6)).tolist(),
                 "tags": [{"id": 7, "size_m": .2, "transform_map_tag": np.eye(4).tolist()}]}
    for name, data in (("platform_calibration", platform), ("vision_calibration", vision), ("reference_calibration", reference),
                       ("superpoint_manifest", {"synthetic_launch_contract": True}), ("detector_manifest", {"synthetic_launch_contract": True})):
        path = tmp_path / (name + ".yaml")
        path.write_text(yaml.safe_dump(data))
        context.launch_configurations[name] = str(path)
    context.launch_configurations.update({"allow_test_calibration": "true", "input_mode": "SIMULATION", "run_id": "synthetic_launch"})
    return module, context


def test_unified_launch_resolves_estimation_display_and_no_control_nodes(tmp_path):
    module, context = configured_context(tmp_path)
    actions = module.build_actions(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    executables = {node.node_executable for node in nodes}
    assert executables == {"platform_localization_node", "apriltag_node", "fixed_reference_node", "object_detection_node",
                           "superpoint_motion_node", "target_metric_node", "target_kinematic_fusion_node",
                           "localization_dashboard_node", "rviz2", "rosbridge_websocket"}
    assert not any("uav_route" in str(node.cmd) or "cmd_vel" in str(node.cmd) for node in nodes)
    assert len([action for action in actions if isinstance(action, ExecuteProcess) and not isinstance(action, Node)]) == 1
    assert not (tmp_path / "run.json").exists()
    assert all(not node.additional_env for node in nodes)
    # Explicit record enables one recorder; no sensor or vehicle commands.
    context.launch_configurations.update({"record": "true", "record_output": str(tmp_path / "new_run"), "dds_transport": "LARGE_DATA"})
    recorded_actions = module.build_actions(context)
    for action in recorded_actions:
        if isinstance(action, ExecuteProcess) and (isinstance(action, Node) or action.additional_env):
            environment = {perform_substitutions(context, key): perform_substitutions(context, value)
                           for key, value in action.additional_env}
            assert environment == {"FASTDDS_BUILTIN_TRANSPORTS": "LARGE_DATA", "ROS_LOCALHOST_ONLY": "0"}
    assert (tmp_path / "new_run/run.json").is_file()
    runtime = json.loads((tmp_path / "new_run/run.json").read_text())["runtime_environment"]
    assert runtime["FASTDDS_BUILTIN_TRANSPORTS"] == "LARGE_DATA"
    assert runtime["ROS_LOCALHOST_ONLY"] == "0"
    with pytest.raises(FileExistsError):
        module.build_actions(context)


def test_unified_launch_rejects_conflicting_mounts_and_mislabeled_test_data(tmp_path):
    module, context = configured_context(tmp_path)
    context.launch_configurations["input_mode"] = "LIVE"
    with pytest.raises(ValueError, match="synthetic calibration"):
        module.build_actions(context)
    context.launch_configurations["input_mode"] = "SIMULATION"
    path = Path(context.launch_configurations["reference_calibration"])
    data = yaml.safe_load(path.read_text())
    data["transform_body_camera"][0][3] = .2
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="camera calibration disagree"):
        module.build_actions(context)
