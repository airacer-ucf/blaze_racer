import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_mapping')
    cmd_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    scanmatch_slam_params = os.path.join(
        pkg_share, 'config', 'slam_toolbox_scanmatch.yaml')
    default_saver_params = os.path.join(
        pkg_share, 'config', 'loop_closure_saver.yaml')
    default_odom_params = os.path.join(
        pkg_share, 'config', 'cmd_odometry.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'slam_mapping.rviz')

    # odom_source selects how the odom -> base_link transform is produced:
    #   'scan_matching' (recommended on this sensorless car) - a static identity
    #       odom->base_link is published and slam_toolbox derives all motion from
    #       LiDAR scan matching. No drifting wheel/command odometry.
    #   'cmd' - dead-reckon odom from the commanded VESC speed/steering
    #       (cmd_odometry). Useful if you have reliable command tracking.
    odom_source_arg = DeclareLaunchArgument(
        'odom_source',
        default_value='scan_matching',
        description="Odometry source: 'scan_matching' or 'cmd'")
    # Optional explicit override; if empty the file is chosen from odom_source.
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value='',
        description='Override path to the slam_toolbox parameters file')
    saver_params_arg = DeclareLaunchArgument(
        'saver_params_file',
        default_value=default_saver_params,
        description='Path to the loop_closure_map_saver parameters file')
    odom_params_arg = DeclareLaunchArgument(
        'odom_params_file',
        default_value=default_odom_params,
        description='Path to the cmd_odometry parameters file')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use the /clock topic (true in simulation)')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Open RViz2 with the SLAM mapping view')

    is_scan_matching = PythonExpression(
        ["'", LaunchConfiguration('odom_source'), "' == 'scan_matching'"])
    is_cmd = PythonExpression(
        ["'", LaunchConfiguration('odom_source'), "' == 'cmd'"])

    # Pick the slam config from odom_source unless explicitly overridden.
    slam_params = PythonExpression(
        ["'", LaunchConfiguration('slam_params_file'),
         "' or ('", scanmatch_slam_params, "' if '",
         LaunchConfiguration('odom_source'), "' == 'scan_matching' else '",
         cmd_slam_params, "')"])

    # Coerce to a real bool so the parameter is typed correctly (a bare string
    # 'false' would not set the bool use_sim_time parameter as intended).
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    # scan_matching mode: publish a static identity odom -> base_link so the TF
    # chain is complete and never lags; slam_toolbox supplies the motion.
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_identity_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=IfCondition(is_scan_matching),
    )

    # cmd mode: dead-reckon odom -> base_link from the commanded VESC values.
    cmd_odometry_node = Node(
        package='slam_mapping',
        executable='cmd_odometry',
        name='cmd_odometry',
        output='screen',
        condition=IfCondition(is_cmd),
        parameters=[
            LaunchConfiguration('odom_params_file'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # slam_toolbox builds the map from /scan + the odom TF as the car is driven.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Saves the map automatically the moment the lap (loop) is closed.
    saver_node = Node(
        package='slam_mapping',
        executable='loop_closure_map_saver',
        name='loop_closure_map_saver',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('saver_params_file'),
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
        odom_source_arg,
        slam_params_arg,
        saver_params_arg,
        odom_params_arg,
        use_sim_time_arg,
        rviz_arg,
        static_odom_tf,
        #cmd_odometry_node,
        slam_node,
        saver_node,
        rviz_node,
    ])
