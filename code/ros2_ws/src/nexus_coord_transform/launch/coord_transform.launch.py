from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("transform_file", default_value=""),
        Node(
            package="nexus_coord_transform",
            executable="coord_transform_node",
            name="nexus_coord_transform",
            output="screen",
            parameters=[{"transform_file": LaunchConfiguration("transform_file")}],
        ),
    ])
