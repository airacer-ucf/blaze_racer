import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the standalone waypoint visualizer, optionally with RViz."""
    pkg = get_package_share_directory('pure_pursuit')
    default_rviz = os.path.join(pkg, 'rviz', 'pure_pursuit.rviz')

    file_arg = DeclareLaunchArgument(
        'waypoints_file', default_value='example_waypoints.csv',
        description='Waypoint CSV to visualize (bare name or absolute path).')
    frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='Frame the waypoints are expressed in.')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz with the bundled config.')

    visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualizer_node',
        name='waypoint_visualizer_node',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'map_frame': LaunchConfiguration('map_frame'),
        }],
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
        file_arg,
        frame_arg,
        rviz_arg,
        visualizer_node,
        rviz_node,
    ])
