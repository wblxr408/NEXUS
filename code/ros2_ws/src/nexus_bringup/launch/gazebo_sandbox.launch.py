from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    follow_route = LaunchConfiguration("follow_route")
    world = PathJoinSubstitution([
        FindPackageShare("nexus_bringup"), "simulation", "nexus_sandbox_imx219.world"])
    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("follow_route", default_value="true"),
        ExecuteProcess(
            cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_init.so", world],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "nexus_bringup", "uav_route_node"],
            condition=IfCondition(follow_route),
            output="screen",
        ),
        ExecuteProcess(
            cmd=["gzclient", "--verbose"],
            condition=IfCondition(use_gui),
            output="screen",
        ),
    ])
