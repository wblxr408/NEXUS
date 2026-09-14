"""Localization and explicit target registration; no flight control."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import numpy as np

from nexus_bringup.dual_recording import prepare_recording, recording_topics
from nexus_fusion_localization.platform_node import load_platform_calibration
from nexus_vision_localization.fixed_reference_node import load_reference_calibration
from nexus_vision_localization.target_metric_node import load_target_calibration


def build_actions(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name).lower() == "true"

    mode, run_id = value("input_mode"), value("run_id")
    if mode not in {"LIVE", "SIMULATION", "REPLAY"} or mode == "REPLAY" and run_id == "UNREGISTERED":
        raise ValueError("input_mode must be explicit; replay requires run_id")
    allow_test = enabled("allow_test_calibration")
    allow_approximate = enabled("allow_approximate_calibration")
    fixed_reference = enabled("fixed_reference")
    configs = {name: str(Path(value(name)).expanduser().resolve()) for name in (
        "platform_calibration", "vision_calibration", "superpoint_manifest", "detector_manifest")}
    if fixed_reference:
        configs["reference_calibration"] = str(Path(value("reference_calibration")).expanduser().resolve())
    for path in configs.values():
        if not Path(path).is_file():
            raise ValueError(f"required calibration/model manifest missing: {path}")
    platform = load_platform_calibration(configs["platform_calibration"], allow_test, allow_approximate)
    vision = load_target_calibration(configs["vision_calibration"], allow_test, allow_approximate)
    if fixed_reference:
        reference, tags = load_reference_calibration(configs["reference_calibration"], allow_test)
        if (vision["camera_frame"] != reference["camera_frame"] or vision["image_rectified"] != reference["image_rectified"]
                or not np.allclose(vision["transform_body_camera"], reference["transform_body_camera"])):
            raise ValueError("visual and fixed-reference camera calibration disagree")
    if not np.allclose(platform.get("vision", {}).get("camera_body_m", [0., 0., 0.]),
                       np.array(vision["transform_body_camera"])[:3, 3]):
        raise ValueError("platform visual lever arm disagrees with camera mounting")
    if not vision["image_rectified"]:
        raise ValueError("AprilTag unified input must be a rectified image with matching CameraInfo")
    if allow_test and mode == "LIVE":
        raise ValueError("synthetic calibration must use SIMULATION or REPLAY mode")
    common = {"use_sim_time": mode != "LIVE", "allow_test_calibration": allow_test,
              "allow_approximate_calibration": allow_approximate}
    image, camera = value("image_topic"), value("camera_info_topic")
    reliability = {"reliability_mode": value("reliability_mode"), "reliability_model": value("reliability_model")}
    transport = value("dds_transport")
    if transport not in {"inherit", "LARGE_DATA"}:
        raise ValueError("dds_transport must be inherit or LARGE_DATA")
    # Humble's localhost-only mode overrides Fast DDS builtin transports.
    # Explicit LARGE_DATA opt-in must also be applied to upstream ROS sources.
    child_env = {"FASTDDS_BUILTIN_TRANSPORTS": "LARGE_DATA", "ROS_LOCALHOST_ONLY": "0"} if transport == "LARGE_DATA" else {}
    if reliability["reliability_mode"] not in {"fixed", "rule", "learned"}:
        raise ValueError("invalid reliability mode")
    if reliability["reliability_mode"] == "learned":
        configs["reliability_model"] = str(Path(reliability["reliability_model"]).resolve())
        if not Path(configs["reliability_model"]).is_file():
            raise ValueError("learned reliability requires model weights")
    optional_assets = {name: value(name) for name in ("lite_superpoint_manifest", "lite_detector_manifest",
                                                       "robust_detector_manifest", "identity_transformer_checkpoint")}
    for name, raw in optional_assets.items():
        if raw:
            path = str(Path(raw).expanduser().resolve())
            if not Path(path).is_file():
                raise ValueError(f"optional edge asset missing: {path}")
            optional_assets[name] = path

    def node(package, executable, parameters=None, **kwargs):
        return Node(package=package, executable=executable, output="screen",
                    parameters=[{**common, **(parameters or {})}], additional_env=child_env, **kwargs)

    actions = [
        node("nexus_fusion_localization", "platform_localization_node",
             {"calibration_file": configs["platform_calibration"], "publish_tf": True,
              "prediction_output_hz": float(value("platform_prediction_hz")), **reliability},
             remappings=[("/nexus/uwb/ranges", value("platform_ranges_topic"))]),
        node("nexus_vision_localization", "object_detection_node", {"model_manifest": configs["detector_manifest"],
             "lite_model_manifest": optional_assets["lite_detector_manifest"],
             "robust_model_manifest": optional_assets["robust_detector_manifest"], "image_topic": image}),
        node("nexus_vision_localization", "superpoint_motion_node", {"model_manifest": configs["superpoint_manifest"],
             "lite_model_manifest": optional_assets["lite_superpoint_manifest"],
             "identity_transformer_checkpoint": optional_assets["identity_transformer_checkpoint"],
             "calibration_file": configs["vision_calibration"], "image_topic": image, "camera_info_topic": camera, "enable_target_tracking": True,
             "rolling_shutter_readout_s": float(value("rolling_shutter_readout_s")),
             "exposure_time_s": float(value("exposure_time_s"))}),
        node("nexus_vision_localization", "target_metric_node", {"calibration_file": configs["vision_calibration"], "camera_info_topic": camera}),
        node("nexus_fusion_localization", "target_kinematic_fusion_node", reliability),
        node("nexus_vision_localization", "adaptive_observation_node", {
            "forbidden_circles_json": value("forbidden_circles_json"), "forbidden_spheres_json": value("forbidden_spheres_json"),
            "maximum_speed_mps": float(value("maximum_speed_mps")), "maximum_acceleration_mps2": float(value("maximum_acceleration_mps2")),
            "planning_horizon_s": float(value("planning_horizon_s")), "policy_profile_file": value("policy_profile_file")}),
        node("nexus_viz_dashboard", "localization_dashboard_node", {"input_mode": mode, "run_id": run_id}),
    ]
    if fixed_reference:
        actions.extend([
        node("apriltag_ros", "apriltag_node", {"family": reference["tag_family"], "size": float(tags[0].size_m),
             "tag.ids": [int(tag.marker_id) for tag in tags], "tag.sizes": [float(tag.size_m) for tag in tags],
             "max_hamming": 0}, remappings=[("image_rect", image), ("camera_info", camera), ("detections", "/apriltag/detections")]),
        node("nexus_vision_localization", "fixed_reference_node",
             {"calibration_file": configs["reference_calibration"], "camera_info_topic": camera}),
        ])
    if enabled("telemetry_udp"):
        actions.append(node("nexus_bringup", "telemetry_udp_node", {"listen_port": int(value("telemetry_udp_port"))}))
    share = Path(get_package_share_directory("nexus_viz_dashboard"))
    if enabled("use_rviz"):
        actions.append(node("rviz2", "rviz2", arguments=["-d", str(share / "rviz/nexus_dual_localization.rviz")]))
    if enabled("use_web"):
        actions.append(node("rosbridge_server", "rosbridge_websocket", {"address": "127.0.0.1", "port": int(value("rosbridge_port")),
                            "topics_pub_glob": ParameterValue(
                                "['/nexus/vision/target_reference_image', '/nexus/vision/target_requests']", value_type=str),
                            "services_glob": ParameterValue("[]", value_type=str),
                            "actions_glob": ParameterValue("[]", value_type=str)}))
        actions.append(ExecuteProcess(cmd=["python3", "-m", "http.server", value("web_port"), "--bind", "127.0.0.1",
                                           "--directory", str(share / "web")], output="screen"))
    if enabled("record"):
        command = prepare_recording(value("record_output"), run_id, mode, configs,
                                    recording_topics(image, camera, value("preview_topic")), runtime_env=child_env)
        actions.append(ExecuteProcess(cmd=command, output="screen", additional_env=child_env))
    return actions


def generate_launch_description():
    required = [DeclareLaunchArgument(name) for name in (
        "platform_calibration", "vision_calibration", "superpoint_manifest", "detector_manifest")]
    defaults = {"platform_prediction_hz": "0.0", "platform_ranges_topic": "/nexus/uwb/ranges", "reference_calibration": "", "fixed_reference": "true", "allow_approximate_calibration": "false",
                "input_mode": "LIVE", "run_id": "UNREGISTERED", "allow_test_calibration": "false",
                "image_topic": "/camera/image_rect", "camera_info_topic": "/camera/camera_info",
                "preview_topic": "/nexus/camera/imx219/image_raw/compressed", "telemetry_udp": "false",
                "telemetry_udp_port": "14551", "use_rviz": "true", "use_web": "true", "record": "false",
                "web_port": "8766", "rosbridge_port": "9090",
                "dds_transport": "inherit",
                "record_output": "", "reliability_mode": "fixed", "reliability_model": "",
                "lite_superpoint_manifest": "", "lite_detector_manifest": "", "robust_detector_manifest": "",
                "identity_transformer_checkpoint": "", "rolling_shutter_readout_s": "0.0", "exposure_time_s": "0.0",
                "forbidden_circles_json": "[]", "forbidden_spheres_json": "[]", "maximum_speed_mps": "3.0",
                "maximum_acceleration_mps2": "2.0", "planning_horizon_s": "3.0", "policy_profile_file": ""}
    return LaunchDescription(required + [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
                             + [OpaqueFunction(function=build_actions)])
