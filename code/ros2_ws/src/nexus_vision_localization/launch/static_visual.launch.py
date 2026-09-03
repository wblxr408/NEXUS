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
                          "image_topic": LaunchConfiguration("image_topic")}]),
        Node(package="nexus_vision_localization", executable="superpoint_motion_node", output="screen",
             parameters=[{"model_manifest": LaunchConfiguration("superpoint_manifest"),
                          "calibration_file": LaunchConfiguration("calibration_file"),
                          "image_topic": LaunchConfiguration("image_topic"),
                          "enable_target_tracking": ParameterValue(LaunchConfiguration("enable_target_tracking"), value_type=bool),
                          "camera_info_topic": LaunchConfiguration("camera_info_topic")}]),
        Node(package="nexus_vision_localization", executable="target_metric_node", output="screen",
             condition=IfCondition(LaunchConfiguration("enable_target_geometry")),
             parameters=[{"calibration_file": LaunchConfiguration("calibration_file"),
                          "camera_info_topic": LaunchConfiguration("camera_info_topic")}]),
        Node(package="nexus_fusion_localization", executable="target_kinematic_fusion_node", output="screen",
             condition=IfCondition(LaunchConfiguration("enable_target_fusion")),
             parameters=[{"reliability_mode": LaunchConfiguration("target_reliability_mode"),
                          "reliability_model": LaunchConfiguration("target_reliability_model")}]),
    ])
