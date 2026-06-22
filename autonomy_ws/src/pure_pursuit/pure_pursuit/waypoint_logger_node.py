#!/usr/bin/env python3
"""
Waypoint logger: record a driveable path by driving the car manually.

Drive the car around the track however you like -- keyboard teleop, a joystick,
or even Follow-the-Gap -- with this node running. It subscribes to odometry and
appends a waypoint every time the car has moved at least ``min_distance`` from
the last recorded point. Each waypoint stores the car's current forward speed
(from the odometry twist), so the resulting CSV already carries a rough
velocity profile you can hand-tune later.

Rows are written and flushed as they are recorded, so an unexpected shutdown
keeps everything captured so far. The file is also closed cleanly on Ctrl-C.

Output format (consumed directly by ``pure_pursuit_node``)::

    x_m,y_m,velocity_mps

This node does not subclass BaseDriveNode because it never commands the car; it
only listens.
"""

import math
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class WaypointLoggerNode(Node):
    """Record odometry into a waypoint CSV while the car is driven manually."""

    def __init__(self):
        """Declare parameters, open the output file, and subscribe to odom."""
        super().__init__('waypoint_logger_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_file', 'waypoints.csv')
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('min_velocity', 0.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.output_file = os.path.abspath(
            self.get_parameter('output_file').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.min_velocity = float(self.get_parameter('min_velocity').value)

        self.last_x = None
        self.last_y = None
        self.count = 0

        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        self.csv_file = open(self.output_file, 'w')
        self.csv_file.write('x_m,y_m,velocity_mps\n')
        self.csv_file.flush()

        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            f'Recording waypoints from {self.odom_topic} to '
            f'{self.output_file} (min_distance={self.min_distance} m)')

    def odom_callback(self, msg):
        """Append a waypoint when the car has moved far enough."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        if speed < self.min_velocity:
            return

        if self.last_x is not None:
            moved = math.hypot(x - self.last_x, y - self.last_y)
            if moved < self.min_distance:
                return

        self.csv_file.write(f'{x:.6f},{y:.6f},{speed:.6f}\n')
        self.csv_file.flush()
        self.last_x = x
        self.last_y = y
        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(f'Recorded {self.count} waypoints')

    def close(self):
        """Flush and close the output file."""
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(
                f'Saved {self.count} waypoints to {self.output_file}')


def main(args=None):
    """Spin the logger until interrupted, then close the file cleanly."""
    rclpy.init(args=args)
    node = WaypointLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
