import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the pure pursuit node, optionally with RViz."""
    pkg = get_package_share_directory('pure_pursuit')
    default_config = os.path.join(pkg, 'config', 'params.yaml')
    default_rviz = os.path.join(pkg, 'rviz', 'pure_pursuit.rviz')

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='Path to the pure_pursuit params YAML.')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz with the bundled config.')

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        config_arg,
        rviz_arg,
        pure_pursuit_node,
        rviz_node,
    ])
