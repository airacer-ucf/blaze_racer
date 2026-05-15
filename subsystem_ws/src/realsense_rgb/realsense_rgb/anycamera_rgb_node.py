#incomplete. works with udev rules

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RealSenseRGBNode(Node):
    def __init__(self):
        super().__init__('realsense_rgb_node')

        self.declare_parameter('device', '/dev/realsense_rgb')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('topic', '/camera/color/image_raw')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('fourcc', 'YUYV')

        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        fourcc_str = self.get_parameter('fourcc').value

        self.get_logger().info(
            f'Opening {device} at {width}x{height} @ {fps}fps -> {topic}'
        )

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open device: {device}')
            raise RuntimeError(f'Could not open {device}')

        # Set pixel format before resolution - order matters on V4L2
        fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Verify what we actually got - V4L2 may negotiate different values
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Actual stream: {actual_w}x{actual_h} @ {actual_fps}fps'
        )

        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / float(fps), self.publish_frame)

    def publish_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Frame grab failed')
            return

        # OpenCV gives BGR by default - publish as bgr8 to match
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
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