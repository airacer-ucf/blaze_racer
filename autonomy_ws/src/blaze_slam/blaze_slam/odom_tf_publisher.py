"""Bridge the VESC /odom topic into the odom -> base_link TF.

slam_toolbox reads the robot's motion from the ``odom -> base_link``
**transform** on /tf, NOT from the /odom topic. On this car ``vesc_to_odom``
publishes the
/odom topic but is configured with ``publish_tf: false`` (see
subsystem_stack/config/vesc.yaml), so that transform is missing and every scan
gets dropped with ``Message Filter dropping message ... reason 'Unknown'`` --
which is exactly why the map came out unusable.

This node subscribes to /odom and re-broadcasts the same pose as the
``odom -> base_link`` TF, using the odometry message's own header stamp so the
transform stays time-consistent with the scans. It is the single, lightweight
piece that lets the standard slam_toolbox workflow work on this car.

If you instead enable ``publish_tf: true`` on ``vesc_to_odom`` (now that VESC
odometry is reliable), do NOT run this node -- two publishers of the same
transform break slam_toolbox. Launch with ``publish_odom_tf:=false`` in that
case.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')

        # Frames default to what vesc_to_odom publishes on this car.
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # If true, override the message frames with the parameters above.
        # If false (default), trust the frame ids already in the /odom message.
        self.declare_parameter('override_frames', False)

        self.odom_frame = self.get_parameter(
            'odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.override_frames = self.get_parameter(
            'override_frames').get_parameter_value().bool_value
        odom_topic = self.get_parameter(
            'odom_topic').get_parameter_value().string_value

        self.br = TransformBroadcaster(self)
        # Match the publisher's QoS loosely; sensor-data is a safe default for
        # high-rate odometry and avoids a reliability mismatch dropping data.
        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)

        self._warned_empty_child = False
        self.get_logger().info(
            "Re-broadcasting '%s' as the %s -> %s TF for slam_toolbox."
            % (odom_topic, self.odom_frame, self.base_frame))

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # Keep the odometry timestamp so /tf and /scan line up in time.
        t.header.stamp = msg.header.stamp

        if self.override_frames:
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
        else:
            t.header.frame_id = msg.header.frame_id or self.odom_frame
            child = msg.child_frame_id or self.base_frame
            if not msg.child_frame_id and not self._warned_empty_child:
                self.get_logger().warn(
                    "Odometry child_frame_id is empty; falling back to '%s'."
                    % self.base_frame)
                self._warned_empty_child = True
            t.child_frame_id = child

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
