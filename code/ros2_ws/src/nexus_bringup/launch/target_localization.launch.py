from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_required = LaunchConfiguration("hardware_required")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    marker_size_m = LaunchConfiguration("marker_size_m")
    position_variance_m2 = LaunchConfiguration("position_variance_m2")
    transform_file = LaunchConfiguration("transform_file")
    use_rviz = LaunchConfiguration("use_rviz")
    max_pair_delta_ms = LaunchConfiguration("max_pair_delta_ms")
    telemetry_udp_port = LaunchConfiguration("telemetry_udp_port")
    default_transform_file = PathJoinSubstitution([
        FindPackageShare("nexus_bringup"), "config", "pre_hardware_transforms.yaml"])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("nexus_viz_dashboard"), "rviz", "nexus_target.rviz"])
    return LaunchDescription([
        DeclareLaunchArgument("hardware_required", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_rect"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("marker_size_m", default_value="0.0"),
        DeclareLaunchArgument("position_variance_m2", default_value="0.0"),
        DeclareLaunchArgument("transform_file", default_value=default_transform_file),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("max_pair_delta_ms", default_value="50.0"),
        DeclareLaunchArgument("telemetry_udp_port", default_value="14551"),
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
                "position_variance_m2": position_variance_m2,
            }.items(),
        ),
        Node(
            package="nexus_coord_transform",
            executable="coord_transform_node",
            name="nexus_coord_transform",
            output="screen",
            parameters=[{"transform_file": transform_file}],
        ),
        Node(
            package="nexus_bringup",
            executable="telemetry_udp_node",
            name="nexus_telemetry_udp",
            output="screen",
            parameters=[{"listen_port": telemetry_udp_port}],
        ),
        Node(
            package="nexus_fusion_localization",
            executable="fusion_localization_node",
            name="nexus_fusion_localization",
            output="screen",
            parameters=[{
                "vision_topic": "/nexus/vision/map_target_observation",
                "max_pair_delta_ms": max_pair_delta_ms,
            }],
        ),
        Node(
            package="nexus_viz_dashboard",
            executable="dashboard_node",
            name="nexus_viz_dashboard",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="nexus_rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
