from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # 模式参数: rviz=启动rviz面板控制, cli=启动命令行控制
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='rviz',
        description='控制模式: rviz (面板控制) 或 cli (命令行控制)'
    )

    mode = LaunchConfiguration('mode')

    # RViz 面板控制模式 (mode == rviz)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [os.path.join(
            get_package_share_directory('fcu_core'),
            'rviz',
            'default.rviz'
            )]
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'rviz'"])),
    )

    # 命令行控制模式 (mode == cli)
    fcu_command_node = Node(
        package='fcu_core',
        executable='fcu_command',
        name='fcu_command',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'cli'"])),
    )

    return LaunchDescription([
        mode_arg,
        rviz_node,
        fcu_command_node,
    ])
