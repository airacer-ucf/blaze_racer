import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("camera_capture"),
        "config",
        "camera_capture.yaml",
    )

    return LaunchDescription([
        Node(
            package="camera_capture",
            executable="camera_capture_node",
            name="camera_capture_node",
            output="screen",
            parameters=[config],
        ),
    ])