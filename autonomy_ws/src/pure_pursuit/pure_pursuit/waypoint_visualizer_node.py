#!/usr/bin/env python3
"""
Waypoint visualizer: publish a waypoint CSV as RViz markers, no driving.

Use this to sanity-check a path before handing it to pure pursuit: load any
CSV and see the line, the per-point speed colouring, and the point density in
RViz. It republishes on a timer so RViz can be opened at any time.

It shares the loader and marker builders with the controller, so what you see
here is exactly what pure pursuit will track.
"""

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

from . import markers
from .waypoint_loader import load_waypoints


class WaypointVisualizerNode(Node):
    """Load a waypoint CSV and publish it as RViz markers periodically."""

    def __init__(self):
        """Declare parameters, load the file, and start the publish timer."""
        super().__init__('waypoint_visualizer_node')

        self.declare_parameter('waypoints_file', 'example_waypoints.csv')
        self.declare_parameter('waypoints_delimiter', ',')
        self.declare_parameter('x_col', 0)
        self.declare_parameter('y_col', 1)
        self.declare_parameter('velocity_col', 2)
        self.declare_parameter('default_velocity', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_topic', '/pure_pursuit/markers')
        self.declare_parameter('period', 1.0)

        name = self.get_parameter('waypoints_file').value
        path = self._resolve(name)
        self.frame = self.get_parameter('map_frame').value
        self.waypoints = load_waypoints(
            path,
            x_col=self.get_parameter('x_col').value,
            y_col=self.get_parameter('y_col').value,
            v_col=self.get_parameter('velocity_col').value,
            delimiter=self.get_parameter('waypoints_delimiter').value,
            default_velocity=self.get_parameter('default_velocity').value,
        )
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {path}')

        topic = self.get_parameter('marker_topic').value
        self.pub = self.create_publisher(Marker, topic, 10)
        period = float(self.get_parameter('period').value)
        self.create_timer(period, self._publish)

    @staticmethod
    def _resolve(name):
        """Resolve a bare filename against the package waypoints directory."""
        if os.path.isabs(name) or os.path.exists(name):
            return name
        share = get_package_share_directory('pure_pursuit')
        return os.path.join(share, 'waypoints', name)

    def _publish(self):
        """Publish the path and per-point markers."""
        stamp = self.get_clock().now().to_msg()
        self.pub.publish(
            markers.build_path_marker(self.waypoints, self.frame, stamp))
        self.pub.publish(
            markers.build_waypoint_points_marker(
                self.waypoints, self.frame, stamp))


def main(args=None):
    """Spin the visualizer until interrupted."""
    rclpy.init(args=args)
    node = WaypointVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
