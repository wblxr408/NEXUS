from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription([
        Node(
            package="nexus_uwb_simulation",
            executable="uwb_simulation_node",
            name="nexus_uwb_simulation",
            output="screen",
            parameters=[{
                "config_file": config_file,
                "odom_topic": "/nexus/gazebo/uav/odom",
                "output_topic": "/nexus/uwb/target_observation",
            }],
        ),
    ])
