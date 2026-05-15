from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('realsense_rgb')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'realsense_rgb.yaml'])

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the parameter YAML file'
    )

    node = Node(
        package='realsense_rgb',
        executable='realsense_rgb_node',
        name='realsense_rgb_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )

    return LaunchDescription([config_arg, node])