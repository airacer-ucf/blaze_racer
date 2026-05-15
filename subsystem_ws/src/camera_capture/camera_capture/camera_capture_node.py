#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy



class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__("camera_capture_node")

        # Declare parameters with defaults
        self.declare_parameter("photo_button", 0)
        self.declare_parameter("record_start_button", 2)
        self.declare_parameter("record_stop_button", 3)

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("image_topic", "/camera/color/image_raw")

        self.declare_parameter(
            "save_directory", os.path.expanduser("~/captures")
        )
        self.declare_parameter("photo_prefix", "photo")
        self.declare_parameter("video_prefix", "video")
        self.declare_parameter("image_format", "jpg")
        self.declare_parameter("video_format", "mp4")
        self.declare_parameter("video_fps", 30)
        self.declare_parameter("video_codec", "mp4v")

        self.declare_parameter("button_debounce_sec", 0.3)
        self.declare_parameter("log_captures", True)

        # Read parameters
        self.photo_btn = self.get_parameter("photo_button").value
        self.rec_start_btn = self.get_parameter("record_start_button").value
        self.rec_stop_btn = self.get_parameter("record_stop_button").value

        self.joy_topic = self.get_parameter("joy_topic").value
        self.image_topic = self.get_parameter("image_topic").value

        self.save_dir = self.get_parameter("save_directory").value
        self.photo_prefix = self.get_parameter("photo_prefix").value
        self.video_prefix = self.get_parameter("video_prefix").value
        self.image_format = self.get_parameter("image_format").value
        self.video_format = self.get_parameter("video_format").value
        self.video_fps = int(self.get_parameter("video_fps").value)
        self.video_codec = self.get_parameter("video_codec").value

        self.debounce = float(self.get_parameter("button_debounce_sec").value)
        self.log_captures = self.get_parameter("log_captures").value

        # Make sure save directory exists
        if not os.path.isdir(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)

        # State
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_size = None
        self.is_recording = False
        self.video_writer = None
        self.last_press_time = {
            self.photo_btn: self.get_clock().now(),
            self.rec_start_btn: self.get_clock().now(),
            self.rec_stop_btn: self.get_clock().now(),
        }
        self.prev_buttons = []

        # Image topics from camera drivers are usually BEST_EFFORT.
        # Using a matching QoS so we actually receive frames.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.create_subscription(
            Image, self.image_topic, self.image_callback, image_qos
        )
        self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10
        )

        self.get_logger().info("camera_capture_node started.")
        self.get_logger().info(f"Saving to: {self.save_dir}")
        self.get_logger().info(
            f"Buttons -> photo: {self.photo_btn}, "
            f"rec_start: {self.rec_start_btn}, "
            f"rec_stop: {self.rec_stop_btn}"
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        self.latest_frame = frame
        current_size = (frame.shape[1], frame.shape[0])

        # If the writer was opened with a different size, frames get dropped silently
        if self.is_recording and self.video_writer is not None:
            if current_size != self.frame_size:
                self.get_logger().warn(
                    f"Frame size changed mid-recording "
                    f"({self.frame_size} -> {current_size}), skipping frame."
                )
            else:
                self.video_writer.write(frame)

        self.frame_size = current_size

    def joy_callback(self, msg):
        # On the first message, just store the state
        if not self.prev_buttons or len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = list(msg.buttons)
            return

        now = self.get_clock().now()

        def rising_edge(idx):
            if idx >= len(msg.buttons):
                return False
            pressed_now = msg.buttons[idx] == 1
            pressed_before = self.prev_buttons[idx] == 1
            if pressed_now and not pressed_before:
                elapsed = (now - self.last_press_time[idx]).nanoseconds / 1e9
                if elapsed >= self.debounce:
                    self.last_press_time[idx] = now
                    return True
            return False

        if rising_edge(self.photo_btn):
            self.take_photo()
        if rising_edge(self.rec_start_btn):
            self.start_recording()
        if rising_edge(self.rec_stop_btn):
            self.stop_recording()

        self.prev_buttons = list(msg.buttons)

    def take_photo(self):
        if self.latest_frame is None:
            self.get_logger().warn("No frame received yet, cannot take photo.")
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{self.photo_prefix}_{ts}.{self.image_format}"
        path = os.path.join(self.save_dir, filename)
        success = cv2.imwrite(path, self.latest_frame)
        if success and self.log_captures:
            self.get_logger().info(f"Photo saved: {path}")
        elif not success:
            self.get_logger().error(f"Failed to save photo to {path}")

    def start_recording(self):
        if self.is_recording:
            self.get_logger().info("Already recording, ignoring.")
            return
        if self.frame_size is None:
            self.get_logger().warn("No frame received yet, cannot start recording.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.video_prefix}_{ts}.{self.video_format}"
        path = os.path.join(self.save_dir, filename)

        fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
        self.video_writer = cv2.VideoWriter(
            path, fourcc, self.video_fps, self.frame_size
        )

        if not self.video_writer.isOpened():
            self.get_logger().error(f"Failed to open video writer for {path}")
            self.video_writer = None
            return

        self.is_recording = True
        if self.log_captures:
            self.get_logger().info(f"Recording started: {path}")

    def stop_recording(self):
        if not self.is_recording:
            self.get_logger().info("Not currently recording, ignoring.")
            return
        self.is_recording = False
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        if self.log_captures:
            self.get_logger().info("Recording stopped.")

    def destroy_node(self):
        if self.is_recording:
            self.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()