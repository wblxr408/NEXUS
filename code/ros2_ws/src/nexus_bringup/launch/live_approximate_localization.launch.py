"""Single ground-computer entry for the existing Pi FC bridge and camera stream."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('nexus_bringup'))
    ingress = Path(get_package_share_directory('nexus_pi_readonly_ingress'))
    platform = LaunchConfiguration('platform_calibration')
    vision = LaunchConfiguration('vision_calibration')
    return LaunchDescription([
        DeclareLaunchArgument('platform_calibration', default_value=str(share / 'config/platform_live_approximate.yaml')),
        DeclareLaunchArgument('vision_calibration', default_value=str(share / 'config/vision_live_approximate.yaml')),
        DeclareLaunchArgument('detector_manifest'), DeclareLaunchArgument('superpoint_manifest'),
        DeclareLaunchArgument('remote_host', default_value='192.168.1.143'),
        DeclareLaunchArgument('imu_input_topic', default_value='/nexus/pi/imu_aligned'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('web_port', default_value='8766'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('run_id', default_value='live_approximate'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('record_output', default_value=''),
        Node(package='nexus_pi_readonly_ingress', executable='camera_file_node.py', output='screen',
             parameters=[{'transport': 'tcp_client', 'clock_sync': True,
                          'remote_host': LaunchConfiguration('remote_host'),
                          'intrinsics_file': str(ingress / 'config/imx219_intrinsics_measured_v01.json')}]),
        Node(package='nexus_pi_readonly_ingress', executable='fcu_clock_ingress_node.py', output='screen',
             parameters=[{'remote_host': LaunchConfiguration('remote_host'), 'transport': 'tcp'}]),
        Node(package='nexus_pi_readonly_ingress', executable='range_smoothing_node.py', output='screen',
             name='nexus_uwb_startup_statistics',
             remappings=[('/nexus/uwb/ranges', '/nexus/pi/ranges_aligned')],
             parameters=[{'window_size': 200, 'output_topic': '/nexus/uwb/startup_statistics'}]),
        Node(package='nexus_bringup', executable='approximate_initial_pose_node', output='screen',
             parameters=[{'calibration_file': platform, 'allow_approximate_calibration': True,
                          'imu_input_topic': LaunchConfiguration('imu_input_topic')}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/dual_localization.launch.py')),
            launch_arguments={'platform_calibration': platform, 'vision_calibration': vision,
                'detector_manifest': LaunchConfiguration('detector_manifest'),
                'superpoint_manifest': LaunchConfiguration('superpoint_manifest'),
                'allow_approximate_calibration': 'true', 'fixed_reference': 'false',
                'platform_prediction_hz': '50.0', 'platform_ranges_topic': '/nexus/pi/ranges_aligned',
                'input_mode': 'LIVE', 'run_id': LaunchConfiguration('run_id'), 'telemetry_udp': 'false',
                'record': LaunchConfiguration('record'), 'record_output': LaunchConfiguration('record_output'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'web_port': LaunchConfiguration('web_port'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port')}.items()),
    ])
