# MIT License
#
# blaze_slam bringup: SLAM Toolbox (online async) + nav2 map saver +
# the blaze_slam map_autosaver that writes a png/yaml map on loop closure.
#
# Run AFTER /scan and the odom -> base_link TF are already being published
# (i.e. after the car's subsystem_stack bringup, or after the sim).
#
#   Car:  ros2 launch blaze_slam blaze_slam_launch.py
#   Sim:  ros2 launch blaze_slam blaze_slam_launch.py use_sim_time:=true
#
# Useful overrides:
#   map_name:=my_track
#   save_policy:=overwrite      (default is rename)
#   autosave_on_loop_closure:=false

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('blaze_slam')
    default_params = PathJoinSubstitution(
        [pkg_share, 'config', 'blaze_slam.yaml'])
    default_maps_dir = os.path.join(
        os.path.expanduser('~'), 'blaze_racer', 'maps')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    maps_dir = LaunchConfiguration('maps_dir')
    map_name = LaunchConfiguration('map_name')
    save_policy = LaunchConfiguration('save_policy')
    autosave = LaunchConfiguration('autosave_on_loop_closure')
    serialize = LaunchConfiguration('serialize_posegraph')
    min_interval = LaunchConfiguration('min_save_interval_sec')

    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true in the F1Tenth gym sim, false on the car'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'maps_dir', default_value=default_maps_dir,
            description='Directory maps are written to'),
        DeclareLaunchArgument(
            'map_name', default_value='track',
            description='Base name for saved map files'),
        DeclareLaunchArgument(
            'save_policy', default_value='rename',
            description="'overwrite' an existing map, or 'rename' (track_1, ...)"),
        DeclareLaunchArgument(
            'autosave_on_loop_closure', default_value='true'),
        DeclareLaunchArgument(
            'serialize_posegraph', default_value='true',
            description='Also write .posegraph/.data for localization / reuse'),
        DeclareLaunchArgument(
            'min_save_interval_sec', default_value='10.0',
            description='Debounce between automatic saves'),
    ]

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            # Foxy declares save_map_timeout as an INTEGER (milliseconds).
            # Galactic+ uses a double (seconds). Passing a double here is what
            # caused the map_saver ParameterTypeException / -6 crash on Foxy.
            'save_map_timeout': 5000,
            # free/occupied thresholds and image_format/map_mode are set
            # per-call by map_autosaver in the SaveMap request, so no
            # server-side defaults are needed here.
        }],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': ['map_saver'],
        }],
    )

    autosaver = Node(
        package='blaze_slam',
        executable='map_autosaver',
        name='map_autosaver',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'maps_dir': maps_dir,
            'map_name': map_name,
            'save_policy': save_policy,
            'autosave_on_loop_closure':
                ParameterValue(autosave, value_type=bool),
            'serialize_posegraph':
                ParameterValue(serialize, value_type=bool),
            'min_save_interval_sec':
                ParameterValue(min_interval, value_type=float),
            'image_format': 'png',
            'map_mode': 'trinary',
            'free_thresh': 0.25,
            'occupied_thresh': 0.65,
            'map_topic': '/map',
            'graph_topic': '/slam_toolbox/graph_visualization',
        }],
    )

    return LaunchDescription(args + [
        slam_node, map_saver, lifecycle, autosaver])