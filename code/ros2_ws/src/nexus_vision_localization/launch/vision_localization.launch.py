from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    marker_size_m = LaunchConfiguration("marker_size_m")
    return LaunchDescription([
        DeclareLaunchArgument("marker_size_m", default_value="0.0"),
        Node(
            package="nexus_vision_localization",
            executable="vision_localization_node",
            name="nexus_vision_localization",
            output="screen",
            parameters=[{
                "marker_size_m": marker_size_m,
                "target_id": "target_0",
                "tag_id": 0,
                "tag_family": "36h11",
            }],
        ),
    ])
