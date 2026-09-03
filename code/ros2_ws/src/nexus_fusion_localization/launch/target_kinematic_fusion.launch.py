from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("reliability_mode", default_value="fixed"),
        DeclareLaunchArgument("reliability_model", default_value=""),
        Node(package="nexus_fusion_localization", executable="target_kinematic_fusion_node", output="screen",
             parameters=[{"reliability_mode": LaunchConfiguration("reliability_mode"),
                          "reliability_model": LaunchConfiguration("reliability_model")}]),
    ])
