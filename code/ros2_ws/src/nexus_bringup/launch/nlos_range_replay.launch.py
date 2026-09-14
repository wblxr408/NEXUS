"""Replay prepared LOS/NLOS UWB range packets into the platform range topic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("packet_file"),
        DeclareLaunchArgument("speed", default_value="1.0"),
        DeclareLaunchArgument("topic", default_value="/nexus/uwb/ranges"),
        Node(package="nexus_bringup", executable="nlos_range_replay_node", output="screen",
             parameters=[{"packet_file": LaunchConfiguration("packet_file"), "speed": LaunchConfiguration("speed"),
                          "topic": LaunchConfiguration("topic")}]),
    ])
