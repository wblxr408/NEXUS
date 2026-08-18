from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    output = LaunchConfiguration("output")
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="nexus_record"),
        ExecuteProcess(cmd=[
            "ros2", "bag", "record", "-o", output,
            "/nexus/fcu/odom", "/nexus/fcu/imu",
            "/nexus/vision/target_observation", "/nexus/target/pose",
        ], output="screen"),
    ])
