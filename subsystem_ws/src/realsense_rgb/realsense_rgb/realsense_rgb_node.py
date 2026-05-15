import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RealSenseRGBNode(Node):
    def __init__(self):
        super().__init__('realsense_rgb_node')

        # Declare parameters with defaults
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('topic', '/camera/color/image_raw')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('serial_number', '')

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        serial = self.get_parameter('serial_number').value

        self.get_logger().info(
            f'Starting RealSense RGB: {width}x{height} @ {fps}fps -> {topic}'
        )

        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        self.pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)

        try:
            self.pipeline.start(config)
        except RuntimeError as e:
            self.get_logger().error(f'Failed to start RealSense pipeline: {e}')
            raise

        self.timer = self.create_timer(1.0 / float(fps), self.publish_frame)

    def publish_frame(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as e:
            self.get_logger().warn(f'Frame wait timed out: {e}')
            return

        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        img = np.asanyarray(color_frame.get_data())
        msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealSenseRGBNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()