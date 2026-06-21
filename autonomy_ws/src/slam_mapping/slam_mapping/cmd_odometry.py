#!/usr/bin/env python3
"""
Dead-reckoning odometry from the commanded VESC actuator values.

This car's motor is sensorless (no hall sensor / encoder), so the VESC cannot
provide reliable wheel odometry — and slam_toolbox needs an ``odom -> base_link``
transform to map. This node fills that gap by integrating the commands actually
sent to the VESC:

* ``/commands/motor/speed``    (eRPM, ``std_msgs/Float64``)
* ``/commands/servo/position`` (servo 0..1, ``std_msgs/Float64``)

Those are converted back to SI speed / steering with the same VESC gains the
subsystem uses, then run through a kinematic bicycle model to produce a pose.
The result is published as ``nav_msgs/Odometry`` and broadcast as the
``odom -> base_link`` TF.

This is only a motion *prior*: slam_toolbox refines every scan with scan
matching and corrects drift on loop closure, so an open-loop estimate from the
commands is enough to build a clean track map. It also works identically
whether the car is driven manually or autonomously, because both routes end up
as the same motor/servo commands.

NOTE: if the VESC's own telemetry-based odometry (``vesc_to_odom`` with
``publish_tf: true``) is ever restored, disable one of the two so there are not
two publishers of ``odom -> base_link``.
"""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class CmdOdometry(Node):
    """Integrate commanded speed/steering into an odom -> base_link estimate."""

    def __init__(self):
        """Declare parameters, set up I/O, and start the integration timer."""
        super().__init__('cmd_odometry')

        # Command topics (outputs of ackermann_to_vesc, inputs to vesc_driver).
        self.declare_parameter('speed_topic', '/commands/motor/speed')
        self.declare_parameter('servo_topic', '/commands/servo/position')

        # VESC conversion gains — must match subsystem_stack/config/vesc.yaml.
        self.declare_parameter('speed_to_erpm_gain', 4214.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('steering_to_servo_gain', -0.8375)
        self.declare_parameter('steering_to_servo_offset', 0.409375)
        self.declare_parameter('wheelbase', 0.33)

        # Frames / output.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_tf', True)

        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        self.speed_to_erpm_offset = \
            self.get_parameter('speed_to_erpm_offset').value
        self.steering_to_servo_gain = \
            self.get_parameter('steering_to_servo_gain').value
        self.steering_to_servo_offset = \
            self.get_parameter('steering_to_servo_offset').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        publish_rate = self.get_parameter('publish_rate').value

        # Integrated pose and latest commands.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0       # m/s, from the motor command
        self.steering = 0.0    # rad, from the servo command
        self.last_time = self.get_clock().now()

        self.create_subscription(
            Float64, self.get_parameter('speed_topic').value,
            self.speed_callback, 10)
        self.create_subscription(
            Float64, self.get_parameter('servo_topic').value,
            self.servo_callback, 10)

        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf \
            else None

        self.create_timer(1.0 / publish_rate, self.update)

        self.get_logger().info(
            'cmd_odometry publishing %s and %s -> %s TF from commanded '
            'speed/steering (sensorless dead reckoning).' % (
                self.get_parameter('odom_topic').value,
                self.odom_frame, self.base_frame))

    def speed_callback(self, msg):
        """Convert commanded eRPM to forward speed in m/s."""
        self.speed = (msg.data - self.speed_to_erpm_offset) \
            / self.speed_to_erpm_gain

    def servo_callback(self, msg):
        """Convert commanded servo position to a steering angle in radians."""
        self.steering = (msg.data - self.steering_to_servo_offset) \
            / self.steering_to_servo_gain

    def update(self):
        """Integrate the bicycle model and publish odometry + TF."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        # Kinematic bicycle model: advance pose by the latest command.
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        if self.wheelbase > 0.0:
            self.yaw += self.speed * math.tan(self.steering) \
                / self.wheelbase * dt

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.angular.z = (
            self.speed * math.tan(self.steering) / self.wheelbase
            if self.wheelbase > 0.0 else 0.0)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    """Spin the command-based odometry node until interrupted."""
    rclpy.init(args=args)
    node = CmdOdometry()
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
