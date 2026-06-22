"""
Combined race launch: SLAM Toolbox (localization) + Pure Pursuit + optional RViz.

Prerequisites
-------------
The subsystem_stack must already be running (it provides /scan, /odom and the
odom -> base_link TF that SLAM Toolbox needs):

    ros2 launch subsystem_stack bringup_launch.py

Then, in a second terminal (source both workspaces first):

    ros2 launch pure_pursuit race.launch.py [rviz:=true]

To point at a different saved map:

    ros2 launch pure_pursuit race.launch.py map:=/abs/path/to/my_map

The ``map`` argument should be the path WITHOUT a file extension; SLAM Toolbox
finds the matching ``.posegraph`` and ``.data`` files automatically.

Localization vs. mapping
------------------------
This launch always runs SLAM Toolbox in *localization* mode, which means it
loads the saved posegraph and corrects the map->odom TF without modifying the
map.  Use ``blaze_slam_launch.py`` if you need to record a fresh map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pp_pkg = get_package_share_directory('pure_pursuit')
    slam_pkg = get_package_share_directory('blaze_slam')

    default_map = os.path.join(
        os.path.expanduser('~'), 'blaze_racer', 'maps', 'track')
    default_slam_params = os.path.join(slam_pkg, 'config', 'blaze_slam.yaml')
    default_pp_config = os.path.join(pp_pkg, 'config', 'params.yaml')
    default_rviz = os.path.join(pp_pkg, 'rviz', 'pure_pursuit.rviz')

    # ------------------------------------------------------------------ args
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Absolute path to the saved SLAM posegraph (no extension).')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Set true to launch RViz with the bundled config.')

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_pp_config,
        description='Path to the pure_pursuit params YAML.')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Set true when running in the F1Tenth gym simulation.')

    # ----------------------------------------------------------- SLAM Toolbox
    # Run in localization mode: loads the saved posegraph and continuously
    # corrects the map->odom transform so pure pursuit gets a map-frame pose.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            # Base sensor / frame / tuning config from blaze_slam.
            default_slam_params,
            # Override just the fields that differ in localization mode.
            {
                'mode': 'localization',
                'map_file_name': ParameterValue(
                    LaunchConfiguration('map'), value_type=str),
                # Start the robot at the pose stored in the posegraph rather
                # than requiring a manual 2D-Pose-Estimate in RViz.
                'map_start_at_dock': True,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )

    # ----------------------------------------------------------- Pure Pursuit
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[LaunchConfiguration('config')],
        output='screen',
        emulate_tty=True,
    )

    # ------------------------------------------------------------------ RViz
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
        config_arg,
        use_sim_time_arg,
        slam_node,
        pure_pursuit_node,
        rviz_node,
    ])
