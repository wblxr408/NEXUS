from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nexus_fusion_localization",
            executable="fusion_localization_node",
            name="nexus_fusion_localization",
            output="screen",
        ),
    ])
