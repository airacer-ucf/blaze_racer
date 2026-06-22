#!/usr/bin/env python3
"""
ROS 2 node wrapping the Pure Pursuit controller.

Most ROS plumbing (odom subscription, drive publishing, safety stop, status
logging, parameter loading) lives in :class:`BaseDriveNode`. Pure pursuit is a
pose-driven algorithm, so this node is constructed with ``uses_odom=True`` and
reads the cached odometry inside :meth:`compute_drive`.

The base class drives ``compute_drive`` from the ``/scan`` callback, which has
a useful side effect on a real car: if the LiDAR feed dies the base class stops
the car for us. The scan itself is ignored here -- the control law only needs
the pose. On top of the base plumbing this node also publishes RViz markers for
the path, the waypoints (coloured by speed), and the live lookahead point.
"""

import os

from ament_index_python.packages import get_package_share_directory
from autonomy_core.base_drive_node import BaseDriveNode
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from visualization_msgs.msg import Marker

from . import markers
from .pure_pursuit import PurePursuitPlanner
from .waypoint_loader import load_waypoints

# Algorithm tunables. Merged on top of BaseDriveNode's defaults (topics,
# frame_id, status_interval). See config/params.yaml for the deployed values.
DEFAULT_PARAMS = {
    # Path source. A bare filename is resolved against this package's installed
    # 'waypoints' directory; an absolute path is used as-is.
    'waypoints_file': 'example_waypoints.csv',
    'waypoints_delimiter': ',',
    'x_col': 0,
    'y_col': 1,
    'velocity_col': 2,
    'default_velocity': 1.0,        # used when a row has no velocity column

    # Geometry. wheelbase is the front-to-rear axle distance of the car.
    'wheelbase': 0.33,              # metres (RoboRacer / F1Tenth chassis)
    'lookahead_distance': 1.0,      # base lookahead (metres)
    'lookahead_gain': 0.0,          # extra lookahead per m/s of speed
    'min_lookahead': 0.5,           # lower clamp on lookahead (metres)
    'max_lookahead': 3.0,           # upper clamp on lookahead (metres)
    'max_steer': 0.349066,          # 20 deg steering limit (radians)

    # Speed shaping applied to the profile read from the waypoints.
    'velocity_gain': 1.0,           # global multiplier on profile speed
    'min_speed': 0.0,               # lower clamp (m/s)
    'max_speed': 7.0,               # upper clamp (m/s)

    # Visualization.
    'map_frame': 'map',             # frame the waypoints live in
    'marker_topic': '/pure_pursuit/markers',
    'publish_markers': True,
    'marker_period': 0.5,           # seconds between full path re-publishes
}


def _resolve_waypoints_path(name):
    """Resolve a waypoints filename to an absolute path.

    Absolute paths and paths that exist relative to the working directory are
    returned unchanged; a bare name is looked up in the installed package
    ``waypoints`` directory.
    """
    if os.path.isabs(name) or os.path.exists(name):
        return name
    share = get_package_share_directory('pure_pursuit')
    return os.path.join(share, 'waypoints', name)


class PurePursuitNode(BaseDriveNode):
    """Pure pursuit node: subscribes to odom, publishes /drive and markers."""

    def __init__(self):
        """Load waypoints, build the controller, and set up marker output."""
        super().__init__('pure_pursuit_node', DEFAULT_PARAMS, uses_odom=False)

        # TF2: look up the robot's pose in the map frame each control cycle.
        # SLAM Toolbox publishes map→odom; the VESC publishes odom→base_link.
        # Combining them via TF is the only correct way to get a map-frame pose.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        path = _resolve_waypoints_path(self.params.waypoints_file)
        self.get_logger().info(f'Loading waypoints from {path}')
        waypoints = load_waypoints(
            path,
            x_col=self.params.x_col,
            y_col=self.params.y_col,
            v_col=self.params.velocity_col,
            delimiter=self.params.waypoints_delimiter,
            default_velocity=self.params.default_velocity,
        )
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
        self.planner = PurePursuitPlanner(self.params, waypoints)

        self.marker_pub = None
        if self.params.publish_markers:
            self.marker_pub = self.create_publisher(
                Marker, self.params.marker_topic, 10)
            # Re-publish the (static) path/points periodically so a late RViz
            # subscriber still sees them.
            self.create_timer(
                self.params.marker_period, self._publish_path_markers)

    def compute_drive(self, scan_msg):
        """Run pure pursuit from the TF map→base_link transform; ignore scan data."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.params.map_frame, 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self._warn_rate_limited(
                f'TF lookup map→base_link failed: {e}  '
                'Is SLAM / localization running?')
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = markers.quaternion_to_yaw(
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w)

        result = self.planner.plan(x, y, yaw)
        if result is None:
            return None

        self._publish_goal_marker()
        return result

    def _publish_path_markers(self):
        """Publish the full path and per-waypoint markers (timer callback)."""
        if self.marker_pub is None:
            return
        stamp = self.get_clock().now().to_msg()
        frame = self.params.map_frame
        wps = self.planner.waypoints
        self.marker_pub.publish(
            markers.build_path_marker(wps, frame, stamp))
        self.marker_pub.publish(
            markers.build_waypoint_points_marker(wps, frame, stamp))

    def _publish_goal_marker(self):
        """Publish a marker at the current lookahead goal point."""
        if self.marker_pub is None or self.planner.last_goal_point is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(
            markers.build_goal_marker(
                self.planner.last_goal_point, self.params.map_frame, stamp))


def main(args=None):
    """Spin the Pure Pursuit node until interrupted."""
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
