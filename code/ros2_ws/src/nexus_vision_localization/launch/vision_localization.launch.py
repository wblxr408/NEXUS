from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nexus_vision_localization",
            executable="vision_localization_node",
            name="nexus_vision_localization",
            output="screen",
            parameters=[{"marker_size_m": 0.0}],
        ),
    ])
