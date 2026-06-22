import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up nav2 map_server and auto-activate it via a lifecycle manager.

    The map_server is a lifecycle node and only publishes /map once it is
    transitioned to the active state. The lifecycle manager with autostart
    handles that transition and keeps the node alive, so the map shows in RViz
    without any manual lifecycle commands. Optionally launches RViz too.
    """
    pkg = get_package_share_directory('pure_pursuit')
    default_rviz = os.path.join(pkg, 'rviz', 'pure_pursuit.rviz')

    map_arg = DeclareLaunchArgument(
        'map', description='Absolute path to the map .yaml file.')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz with the bundled config.')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
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
        map_arg,
        rviz_arg,
        map_server,
        lifecycle_manager,
        rviz_node,
    ])
