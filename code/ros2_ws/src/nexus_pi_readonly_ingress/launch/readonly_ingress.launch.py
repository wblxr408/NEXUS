from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("nexus_pi_readonly_ingress"))
    return LaunchDescription([
        DeclareLaunchArgument("hardware_config", default_value=str(share / "config/hardware_user_v01.yaml")),
        DeclareLaunchArgument("listen_port", default_value="14551"),
        DeclareLaunchArgument("transport", default_value="tcp_client"),
        DeclareLaunchArgument("remote_host", default_value="192.168.1.143"),
        DeclareLaunchArgument("camera_dir", default_value="/home/pi/nexus_readonly_runs/live_camera_v2"),
        DeclareLaunchArgument("camera_transport", default_value="tcp_client"),
        DeclareLaunchArgument("camera_remote_port", default_value="14552"),
        Node(package="nexus_pi_readonly_ingress", executable="telemetry_ingress_node.py",
             parameters=[{"hardware_config": LaunchConfiguration("hardware_config"),
                          "listen_port": LaunchConfiguration("listen_port"),
                          "transport": LaunchConfiguration("transport"),
                          "remote_host": LaunchConfiguration("remote_host")}], output="screen"),
        Node(package="nexus_pi_readonly_ingress", executable="camera_file_node.py",
             parameters=[{"camera_dir": LaunchConfiguration("camera_dir"),
                          "transport": LaunchConfiguration("camera_transport"),
                          "remote_host": LaunchConfiguration("remote_host"),
                          "remote_port": LaunchConfiguration("camera_remote_port")}], output="screen"),
    ])
