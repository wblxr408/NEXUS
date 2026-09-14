from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("superpoint_manifest"),
        DeclareLaunchArgument("detector_manifest"),
        DeclareLaunchArgument("lite_superpoint_manifest", default_value=""),
        DeclareLaunchArgument("lite_detector_manifest", default_value=""),
        DeclareLaunchArgument("robust_detector_manifest", default_value=""),
        DeclareLaunchArgument("identity_transformer_checkpoint", default_value=""),
        # Camera timing is intentionally explicit: 0 disables row compensation
        # when a global-shutter camera is used; a rolling-shutter camera must
        # provide measured readout/exposure values rather than guessed values.
        DeclareLaunchArgument("rolling_shutter_readout_s", default_value="0.0"),
        DeclareLaunchArgument("exposure_time_s", default_value="0.0"),
        DeclareLaunchArgument("forbidden_circles_json", default_value="[]"),
        DeclareLaunchArgument("forbidden_spheres_json", default_value="[]"),
        DeclareLaunchArgument("maximum_speed_mps", default_value="3.0"),
        DeclareLaunchArgument("maximum_acceleration_mps2", default_value="2.0"),
        DeclareLaunchArgument("planning_horizon_s", default_value="3.0"),
        DeclareLaunchArgument("policy_profile_file", default_value=""),
        DeclareLaunchArgument("calibration_file"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_rect"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("enable_target_tracking", default_value="true"),
        DeclareLaunchArgument("enable_target_geometry", default_value="true"),
        DeclareLaunchArgument("enable_target_fusion", default_value="true"),
        DeclareLaunchArgument("target_reliability_mode", default_value="fixed"),
        DeclareLaunchArgument("target_reliability_model", default_value=""),
        Node(package="nexus_vision_localization", executable="object_detection_node", output="screen",
             parameters=[{"model_manifest": LaunchConfiguration("detector_manifest"),
                          "lite_model_manifest": LaunchConfiguration("lite_detector_manifest"),
                          "robust_model_manifest": LaunchConfiguration("robust_detector_manifest"),
                          "image_topic": LaunchConfiguration("image_topic")}]),
        Node(package="nexus_vision_localization", executable="superpoint_motion_node", output="screen",
             parameters=[{"model_manifest": LaunchConfiguration("superpoint_manifest"),
                          "lite_model_manifest": LaunchConfiguration("lite_superpoint_manifest"),
                          "identity_transformer_checkpoint": LaunchConfiguration("identity_transformer_checkpoint"),
                          "calibration_file": LaunchConfiguration("calibration_file"),
                          "image_topic": LaunchConfiguration("image_topic"),
                          "enable_target_tracking": ParameterValue(LaunchConfiguration("enable_target_tracking"), value_type=bool),
                          "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                          "rolling_shutter_readout_s": LaunchConfiguration("rolling_shutter_readout_s"),
                          "exposure_time_s": LaunchConfiguration("exposure_time_s")}]),
        Node(package="nexus_vision_localization", executable="target_metric_node", output="screen",
             condition=IfCondition(LaunchConfiguration("enable_target_geometry")),
             parameters=[{"calibration_file": LaunchConfiguration("calibration_file"),
                          "camera_info_topic": LaunchConfiguration("camera_info_topic")}]),
        Node(package="nexus_fusion_localization", executable="target_kinematic_fusion_node", output="screen",
            condition=IfCondition(LaunchConfiguration("enable_target_fusion")),
            parameters=[{"reliability_mode": LaunchConfiguration("target_reliability_mode"),
                          "reliability_model": LaunchConfiguration("target_reliability_model")}]),
        Node(package="nexus_vision_localization", executable="adaptive_observation_node", output="screen",
             parameters=[{"forbidden_circles_json": LaunchConfiguration("forbidden_circles_json"),
                          "forbidden_spheres_json": LaunchConfiguration("forbidden_spheres_json"),
                          "maximum_speed_mps": LaunchConfiguration("maximum_speed_mps"),
                          "maximum_acceleration_mps2": LaunchConfiguration("maximum_acceleration_mps2"),
                          "planning_horizon_s": LaunchConfiguration("planning_horizon_s"),
                          "policy_profile_file": LaunchConfiguration("policy_profile_file")}]),
    ])
