"""Localize the car on a previously built map with slam_toolbox.

Loads a serialized pose graph (saved during mapping) and publishes the
map -> odom TF without modifying the map. The subsystem stack must be running.

Example:
  ros2 launch blaze_slam localization.launch.py \\
      map_file_name:=$HOME/maps/blaze_track
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('blaze_slam')
    default_params = os.path.join(pkg_share, 'config', 'localization.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params,
        description='Path to the slam_toolbox localization parameters file.')
    map_file_arg = DeclareLaunchArgument(
        'map_file_name', default_value='',
        description='Serialized pose graph to load (path without extension).')
    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf', default_value='true',
        description="Bridge /odom into the odom->base_link TF. Set 'false' if "
                    "vesc_to_odom already publishes that TF.")
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic to bridge into TF.')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use the /clock topic (true in simulation / bag replay).')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz2 with the SLAM view.')

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    odom_tf_node = Node(
        package='blaze_slam',
        executable='odom_tf_publisher',
        name='odom_tf_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': use_sim_time,
        }],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': use_sim_time,
                'map_file_name': LaunchConfiguration('map_file_name'),
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        slam_params_arg,
        map_file_arg,
        publish_odom_tf_arg,
        odom_topic_arg,
        use_sim_time_arg,
        rviz_arg,
        odom_tf_node,
        slam_node,
        rviz_node,
    ])
