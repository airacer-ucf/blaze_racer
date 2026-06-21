"""Online-async track mapping with slam_toolbox for blaze_racer.

Brings up:
  * odom_tf_publisher  -- /odom topic  ->  odom -> base_link TF (optional)
  * async_slam_toolbox_node  -- builds /map from /scan + the TF
  * rviz2 (optional)

The subsystem stack (bringup_launch.py) must already be running so that /scan,
/odom and the static base_link -> laser TF are present.
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
    default_params = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params,
        description='Path to the slam_toolbox parameters file.')
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
        description='Open RViz2 with the SLAM mapping view.')

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
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': use_sim_time},
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
        publish_odom_tf_arg,
        odom_topic_arg,
        use_sim_time_arg,
        rviz_arg,
        odom_tf_node,
        slam_node,
        rviz_node,
    ])
