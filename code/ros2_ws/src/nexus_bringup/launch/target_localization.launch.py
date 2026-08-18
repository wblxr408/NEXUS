from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_required = LaunchConfiguration("hardware_required")
    return LaunchDescription([
        DeclareLaunchArgument("hardware_required", default_value="true"),
        Node(
            package="nexus_bringup",
            executable="preflight_node",
            name="nexus_bringup_preflight",
            output="screen",
            parameters=[{"hardware_required": hardware_required}],
        ),
        Node(
            package="nexus_vision_localization",
            executable="vision_localization_node",
            name="nexus_vision_localization",
            output="screen",
        ),
        Node(
            package="nexus_coord_transform",
            executable="coord_transform_node",
            name="nexus_coord_transform",
            output="screen",
        ),
        Node(
            package="nexus_fusion_localization",
            executable="fusion_localization_node",
            name="nexus_fusion_localization",
            output="screen",
        ),
        Node(
            package="nexus_viz_dashboard",
            executable="dashboard_node",
            name="nexus_viz_dashboard",
            output="screen",
        ),
    ])
