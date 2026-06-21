#!/usr/bin/env python3
"""
Automatically save the SLAM map once the track loop is closed.

slam_toolbox builds an occupancy-grid map of the track while the car is driven
(manually or autonomously). This node tracks the robot's pose and, the moment
the car completes a lap — i.e. it has driven away from its starting point and
then returned to it after covering at least a full lap's distance — it saves
the map to disk as a ``.png`` + ``.yaml`` pair and, optionally, serializes 
the slam_toolbox pose graph for later localization.

The pose is taken from the SLAM-corrected ``map -> base_link`` transform by
default (``pose_source: tf``), which works regardless of the odometry source
and reflects loop-closure corrections. Set ``pose_source: odom`` to instead use
the ``/odom`` topic.

Why return-to-start rather than a SLAM "loop closure" callback? slam_toolbox
does not expose a loop-closure event on a topic. On a closed-circuit track,
returning to the start pose after a full lap is exactly the condition under
which slam_toolbox runs its loop-closure optimisation, so this is a robust,
deterministic trigger that needs no extra instrumentation.

The actual map writing is delegated to ``nav2_map_server``'s ``map_saver_cli``,
run as a subprocess, so the on-disk result is identical to saving the map by
hand from the command line.
"""

import os
import subprocess

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (ConnectivityException, ExtrapolationException,
                     LookupException)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LoopClosureMapSaver(Node):
    """Detect lap completion from the robot pose and save the map once."""

    def __init__(self):
        """Declare parameters and set up the pose source."""
        super().__init__('loop_closure_map_saver')

        # --- Pose source: 'tf' (map -> base_link) or 'odom' (/odom topic) ---
        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pose_rate', 10.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')

        # --- Where and how to save ---
        self.declare_parameter('map_save_dir', '~/maps')
        self.declare_parameter('map_name', 'blaze_track')
        # Image/threshold settings
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('map_mode', 'trinary')
        self.declare_parameter('occupied_thresh', 0.45)
        self.declare_parameter('free_thresh', 0.196)
        self.declare_parameter('save_timeout', 30.0)

        # --- Loop-closure (lap) detection geometry, in metres ---
        # Must drive at least this far before a lap can count (rules out a save
        # while still near the start line).
        self.declare_parameter('min_lap_distance', 8.0)
        # How far from start counts as "back home".
        self.declare_parameter('loop_closure_radius', 1.0)
        # How far the car must first travel from start before we start checking
        # for a return (prevents an instant trigger from jitter at the start).
        self.declare_parameter('departure_radius', 2.5)
        # Reject implausible single-sample jumps (m). At 50 Hz, 0.5 m is 25 m/s.
        # This guards against odometry glitches and stray second publishers on
        # the odom topic.
        self.declare_parameter('max_odom_step', 0.5)
        # A lap must take at least this long (s); blocks an instant false save.
        self.declare_parameter('min_lap_time', 5.0)

        # --- Behaviour after saving ---
        self.declare_parameter('serialize_pose_graph', True)
        self.declare_parameter('shutdown_after_save', False)

        self.pose_source = self.get_parameter('pose_source').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        pose_rate = self.get_parameter('pose_rate').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_save_dir = os.path.expanduser(
            self.get_parameter('map_save_dir').value)
        self.map_name = self.get_parameter('map_name').value
        self.image_format = self.get_parameter('image_format').value
        self.map_mode = self.get_parameter('map_mode').value
        self.occupied_thresh = self.get_parameter('occupied_thresh').value
        self.free_thresh = self.get_parameter('free_thresh').value
        self.save_timeout = self.get_parameter('save_timeout').value
        self.min_lap_distance = self.get_parameter('min_lap_distance').value
        self.loop_closure_radius = \
            self.get_parameter('loop_closure_radius').value
        self.departure_radius = self.get_parameter('departure_radius').value
        self.max_odom_step = self.get_parameter('max_odom_step').value
        self.min_lap_time = self.get_parameter('min_lap_time').value
        self.serialize_pose_graph = \
            self.get_parameter('serialize_pose_graph').value
        self.shutdown_after_save = \
            self.get_parameter('shutdown_after_save').value

        # Lap-tracking state.
        self.start_xy = None      # pose where mapping began
        self.last_xy = None       # previous odometry sample
        self.traveled = 0.0       # cumulative path length
        self.armed = False        # True once we have left the start region
        self.saved = False        # True once the map has been saved
        self.arm_time = None      # time the lap tracking was armed
        self.last_warn_ns = 0     # rate-limit for jump warnings

        # Pose input. 'tf' polls the SLAM-corrected map -> base_link transform;
        # 'odom' subscribes to the odometry topic.
        if self.pose_source == 'odom':
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self.odom_callback, 10)
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(1.0 / pose_rate, self._tf_tick)

        self.get_logger().info(
            'loop_closure_map_saver ready (pose_source=%s): will auto-save '
            "'%s' to %s on loop closure (>= %.1f m lap, return within %.1f m "
            'of start).' % (self.pose_source, self.map_name, self.map_save_dir,
                            self.min_lap_distance, self.loop_closure_radius))

    def _tf_tick(self):
        """Poll the SLAM map -> base_link transform for the current pose."""
        if self.saved:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            return  # map -> base_link not available yet (slam still starting)
        self._process_position(tf.transform.translation.x,
                               tf.transform.translation.y)

    def odom_callback(self, msg):
        """Feed an /odom sample into the lap detector (pose_source: odom)."""
        self._process_position(msg.pose.pose.position.x,
                               msg.pose.pose.position.y)

    def _process_position(self, x, y):
        """Accumulate distance and trigger a save when the lap closes."""
        if self.saved:
            return

        if self.start_xy is None:
            self.start_xy = (x, y)
            self.last_xy = (x, y)
            return

        # Reject implausible jumps (odometry glitches, or a second publisher on
        # the odom topic). Resync the reference without accumulating distance.
        step = self._dist((x, y), self.last_xy)
        if step > self.max_odom_step:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_warn_ns > 1e9:
                self.get_logger().warning(
                    f'Ignoring {step:.2f} m odometry jump (> max_odom_step '
                    f'{self.max_odom_step:.2f} m). Check for duplicate odom '
                    'publishers.')
                self.last_warn_ns = now_ns
            self.last_xy = (x, y)
            return

        # Accumulate path length since the last sample.
        self.traveled += step
        self.last_xy = (x, y)

        dist_from_start = self._dist((x, y), self.start_xy)

        # Arm the return check only after the car has clearly left the start.
        if not self.armed and dist_from_start > self.departure_radius:
            self.armed = True
            self.arm_time = self.get_clock().now()
            self.get_logger().info(
                f'Left start region ({dist_from_start:.2f} m away); '
                'lap tracking armed.')
            return

        # Require a minimum elapsed time since arming so a glitch cannot trigger
        # an instant "lap".
        armed_long_enough = (
            self.arm_time is not None
            and (self.get_clock().now() - self.arm_time).nanoseconds * 1e-9
            >= self.min_lap_time)

        # Loop closed: a full lap covered and we are back near the start.
        if (self.armed
                and armed_long_enough
                and self.traveled >= self.min_lap_distance
                and dist_from_start <= self.loop_closure_radius):
            self.saved = True
            self.get_logger().info(
                f'Loop closure detected (lap of {self.traveled:.1f} m, '
                f'{dist_from_start:.2f} m from start). Saving map...')
            self.save_map()
            if self.serialize_pose_graph:
                self.serialize_graph()
            if self.shutdown_after_save:
                self.get_logger().info('Map saved; shutting down.')
                rclpy.shutdown()

    @staticmethod
    def _dist(a, b):
        """Return the Euclidean distance between two (x, y) points."""
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def _map_path(self):
        """Return the absolute output path (no extension) for the map files."""
        os.makedirs(self.map_save_dir, exist_ok=True)
        return os.path.join(self.map_save_dir, self.map_name)

    def save_map(self):
        """Save the occupancy grid as <name>.<fmt> + <name>.yaml via nav2."""
        out_path = self._map_path()
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-t', self.map_topic,
            '-f', out_path,
            '--occ', str(self.occupied_thresh),
            '--free', str(self.free_thresh),
            '--fmt', self.image_format,
            '--mode', self.map_mode,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True,
                timeout=self.save_timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f'map_saver_cli timed out after {self.save_timeout:.0f} s. '
                'Is /map being published?')
            return

        if result.returncode == 0:
            self.get_logger().info(
                f'Map saved: {out_path}.{self.image_format} and '
                f'{out_path}.yaml')
        else:
            self.get_logger().error(
                'map_saver_cli failed (exit '
                f'{result.returncode}): {result.stderr.strip()}')

    def serialize_graph(self):
        """Serialize the slam_toolbox pose graph for later localization."""
        out_path = self._map_path()
        cmd = [
            'ros2', 'service', 'call', '/slam_toolbox/serialize_map',
            'slam_toolbox/srv/SerializePoseGraph',
            '{filename: "%s"}' % out_path,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True,
                timeout=self.save_timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                'Pose-graph serialization timed out; skipping.')
            return

        if result.returncode == 0:
            self.get_logger().info(
                f'Pose graph serialized: {out_path}.posegraph / .data')
        else:
            self.get_logger().warning(
                'Pose-graph serialization failed (is slam_toolbox running?): '
                f'{result.stderr.strip()}')


def main(args=None):
    """Spin the loop-closure map saver until interrupted."""
    rclpy.init(args=args)
    node = LoopClosureMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
