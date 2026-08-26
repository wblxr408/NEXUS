from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    marker_size_m = LaunchConfiguration("marker_size_m")
    position_variance_m2 = LaunchConfiguration("position_variance_m2")
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image_rect"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("marker_size_m", default_value="0.0"),
        DeclareLaunchArgument("position_variance_m2", default_value="0.0"),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            remappings=[("image_rect", image_topic), ("camera_info", camera_info_topic)],
            parameters=[{
                "family": "36h11",
                "size": marker_size_m,
                "max_hamming": 0,
            }],
        ),
        Node(
            package="nexus_vision_localization",
            executable="vision_localization_node",
            name="nexus_vision_localization",
            output="screen",
            remappings=[
                ("~/detections", "/apriltag/detections"),
                ("~/camera_info", camera_info_topic),
            ],
            parameters=[{
                "marker_size_m": marker_size_m,
                "target_id": "target_0",
                "tag_id": 0,
                "tag_family": "36h11",
                "position_variance_m2": position_variance_m2,
            }],
        ),
    ])
