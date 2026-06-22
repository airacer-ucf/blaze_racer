from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the waypoint logger for recording a manual driving line."""
    odom_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic to record poses from.')
    output_arg = DeclareLaunchArgument(
        'output_file', default_value='waypoints.csv',
        description='CSV file to write recorded waypoints to.')
    distance_arg = DeclareLaunchArgument(
        'min_distance', default_value='0.2',
        description='Minimum spacing between recorded waypoints (metres).')
    min_vel_arg = DeclareLaunchArgument(
        'min_velocity', default_value='0.0',
        description='Ignore samples slower than this (m/s).')

    logger_node = Node(
        package='pure_pursuit',
        executable='waypoint_logger_node',
        name='waypoint_logger_node',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'output_file': LaunchConfiguration('output_file'),
            'min_distance': LaunchConfiguration('min_distance'),
            'min_velocity': LaunchConfiguration('min_velocity'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        odom_arg,
        output_arg,
        distance_arg,
        min_vel_arg,
        logger_node,
    ])
