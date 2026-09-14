from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_file", description="Measured platform calibration YAML; no default hardware values"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("allow_approximate_calibration", default_value="false"),
        DeclareLaunchArgument("reliability_mode", default_value="fixed"),
        DeclareLaunchArgument("reliability_model", default_value=""),
        Node(package="nexus_fusion_localization", executable="platform_localization_node",
             name="nexus_platform_localization", output="screen", parameters=[{
                 "calibration_file": LaunchConfiguration("calibration_file"),
                 "publish_tf": LaunchConfiguration("publish_tf"),
                 "allow_approximate_calibration": LaunchConfiguration("allow_approximate_calibration"),
                 "reliability_mode": LaunchConfiguration("reliability_mode"),
                 "reliability_model": LaunchConfiguration("reliability_model"),
             }]),
    ])
