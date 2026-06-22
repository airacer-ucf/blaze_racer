"""
RViz marker construction and pose helpers for pure pursuit.

These helpers build ``visualization_msgs`` markers from plain NumPy data so the
node code stays short. ``quaternion_to_yaw`` is plain math (no ROS) and is unit
tested directly.
"""

import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def quaternion_to_yaw(qx, qy, qz, qw):
    """Return the yaw (rotation about z, radians) of a quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def _color(r, g, b, a=1.0):
    """Build a ``std_msgs/ColorRGBA``."""
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def build_path_marker(waypoints, frame_id, stamp, scale=0.08):
    """
    Build a green LINE_STRIP marker tracing the waypoint path.

    :param waypoints: ``(N, 3)`` array of ``[x, y, velocity]`` rows.
    :param frame_id: frame the points are expressed in (e.g. ``map``).
    :param stamp: ROS time stamp for the marker header.
    :param scale: line width in metres.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit_path'
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = float(scale)
    marker.color = _color(0.0, 0.9, 0.2, 0.8)
    marker.pose.orientation.w = 1.0
    for wx, wy in waypoints[:, :2]:
        p = Point()
        p.x, p.y, p.z = float(wx), float(wy), 0.0
        marker.points.append(p)
    return marker


def build_waypoint_points_marker(waypoints, frame_id, stamp, scale=0.12):
    """
    Build a SPHERE_LIST marker for the individual waypoints.

    Each point is tinted from blue (slow) to red (fast) by its velocity so the
    velocity profile is visible at a glance.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit_points'
    marker.id = 1
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = float(scale)
    marker.scale.y = float(scale)
    marker.scale.z = float(scale)
    marker.pose.orientation.w = 1.0

    vels = waypoints[:, 2]
    v_min = float(vels.min())
    v_max = float(vels.max())
    v_range = max(v_max - v_min, 1e-6)
    for wx, wy, wv in waypoints:
        p = Point()
        p.x, p.y, p.z = float(wx), float(wy), 0.0
        marker.points.append(p)
        frac = (float(wv) - v_min) / v_range
        marker.colors.append(_color(frac, 0.2, 1.0 - frac, 1.0))
    return marker


def build_goal_marker(goal_xy, frame_id, stamp, scale=0.3):
    """Build a yellow SPHERE marker at the current lookahead goal point."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'pure_pursuit_goal'
    marker.id = 2
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = float(scale)
    marker.scale.y = float(scale)
    marker.scale.z = float(scale)
    marker.color = _color(1.0, 0.9, 0.0, 1.0)
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(goal_xy[0])
    marker.pose.position.y = float(goal_xy[1])
    marker.pose.position.z = 0.0
    # Expire quickly so the sphere disappears when the car stops rather than
    # lingering at the last commanded goal point.
    marker.lifetime.sec = 0
    marker.lifetime.nanosec = 500_000_000  # 0.5 s
    return marker
