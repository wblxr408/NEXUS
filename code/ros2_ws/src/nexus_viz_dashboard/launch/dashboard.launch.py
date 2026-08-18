from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nexus_viz_dashboard",
            executable="dashboard_node",
            name="nexus_viz_dashboard",
            output="screen",
        ),
    ])
