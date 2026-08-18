from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nexus_coord_transform",
            executable="coord_transform_node",
            name="nexus_coord_transform",
            output="screen",
            parameters=[{"transform_file": ""}],
        ),
    ])
