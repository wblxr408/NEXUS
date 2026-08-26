from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    follow_route = LaunchConfiguration("follow_route")
    config_file = LaunchConfiguration("config_file")
    sigma_m = LaunchConfiguration("sigma_m")
    random_seed = LaunchConfiguration("random_seed")

    default_config = PathJoinSubstitution([
        FindPackageShare("nexus_bringup"), "simulation", "configs", "uwb_simulation_config.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("follow_route", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("sigma_m", default_value="0.0"),
        DeclareLaunchArgument("random_seed", default_value="42"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("nexus_bringup"), "launch", "gazebo_sandbox.launch.py"])),
            launch_arguments={
                "use_gui": use_gui,
                "follow_route": follow_route,
            }.items(),
        ),
        Node(
            package="nexus_uwb_simulation",
            executable="uwb_simulation_node",
            name="nexus_uwb_simulation",
            output="screen",
            parameters=[{
                "config_file": config_file,
                "odom_topic": "/nexus/gazebo/uav/odom",
                "output_topic": "/nexus/uwb/target_observation",
                "algorithm": "uwb.matlab.trilateration",
                "sigma_m": sigma_m,
                "random_seed": random_seed,
            }],
        ),
    ])
