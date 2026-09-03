from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_file", description="Surveyed fixed-tag map and camera extrinsic YAML"),
        DeclareLaunchArgument("detections_topic", default_value="/apriltag/detections"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        Node(package="nexus_vision_localization", executable="fixed_reference_node",
             name="nexus_fixed_reference", output="screen", parameters=[{
                 "calibration_file": LaunchConfiguration("calibration_file"),
                 "detections_topic": LaunchConfiguration("detections_topic"),
                 "camera_info_topic": LaunchConfiguration("camera_info_topic"),
             }]),
    ])
