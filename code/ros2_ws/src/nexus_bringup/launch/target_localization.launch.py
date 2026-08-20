from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_required = LaunchConfiguration("hardware_required")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    marker_size_m = LaunchConfiguration("marker_size_m")
    return LaunchDescription([
        DeclareLaunchArgument("hardware_required", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_rect"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("marker_size_m", default_value="0.0"),
        Node(
            package="nexus_bringup",
            executable="preflight_node",
            name="nexus_bringup_preflight",
            output="screen",
            parameters=[{"hardware_required": hardware_required}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("nexus_vision_localization"),
                "launch",
                "apriltag_localization.launch.py",
            ])),
            launch_arguments={
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "marker_size_m": marker_size_m,
            }.items(),
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
