"""Camera-only ingress with measured network clock alignment; no FC nodes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("nexus_pi_readonly_ingress"))
    return LaunchDescription([
        DeclareLaunchArgument("remote_host", default_value="192.168.1.143"),
        DeclareLaunchArgument("intrinsics_file",
                              default_value=str(share / "config/imx219_intrinsics_measured_v01.json")),
        Node(package="nexus_pi_readonly_ingress", executable="camera_file_node.py",
             output="screen", parameters=[{
                 "remote_host": LaunchConfiguration("remote_host"),
                 "transport": "tcp_client", "clock_sync": True,
                 "intrinsics_file": LaunchConfiguration("intrinsics_file"),
             }]),
    ])
