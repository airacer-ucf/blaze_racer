# camera_capture

A ROS2 package that lets you take photos and record videos from a camera topic using a controller.

## Overview

This package subscribes to a `joy` topic and an `image` topic, and saves photos or videos based on controller button presses. 

Intended button mapping:
- X: take a photo
- Triangle: start recording
- Square: stop recording

## Dependencies

- ROS2 Foxy or Humble
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `joy` (for the joystick driver)
- OpenCV (`cv2`)

The `joy` node is not started by this package and must be running separately.

## Installation

Clone the package into your workspace's `src` directory and build:

```
cd ~/your_ws/src
git clone <your-repo-url> camera_capture
cd ~/your_ws
colcon build --packages-select camera_capture
source install/setup.bash
```

## Usage

First, make sure a joy node is running:

```
ros2 run joy joy_node
```

Then launch the capture node:

```
ros2 launch camera_capture camera_capture.launch.py
```

By default, captures are saved to `~/captures`. Photos are timestamped JPGs and videos are timestamped MP4s.

## Configuration

All parameters live in `config/camera_capture.yaml`. The most common things to change:

- `joy_topic` and `image_topic`: change these if your topics are named differently
- `save_directory`: where photos and videos go (change path based on your machine)
- `photo_button`, `record_start_button`, `record_stop_button`: button indices on your controller
- `video_fps`: should match the actual publish rate of your camera, otherwise video playback duration will be wrong
- `image_format`: jpg or png
- `video_format` and `video_codec`: mp4 with mp4v codec, or avi with MJPG codec
- `button_debounce_sec`: how long to ignore repeat presses of the same button

After editing the yaml, rebuild for the changes to take effect:

```
colcon build --packages-select camera_capture
```

Or use `colcon build --symlink-install` once, after which yaml edits take effect immediately on relaunch.

## Verifying button indices

SB and Bluetooth connections, or alternative drivers like `ds4_driver`, can produce different mappings.

To check which index corresponds to which button, run:

```
ros2 topic echo /joy
```

Then press each button you care about and note which index in the `buttons` array changes from 0 to 1. Update the yaml accordingly.

## Notes on QoS

The image subscriber uses `BEST_EFFORT` reliability with a depth of 1, which matches what most camera drivers (RealSense, Zed, etc.) publish with. If your camera driver publishes with `RELIABLE` QoS, you may need to change the QoS profile in the node code.

## Troubleshooting

**No photo saved or "No frame received yet" warning**
The node hasn't received any image messages. Check that the camera is publishing and the topic name matches:

```
ros2 topic hz /camera/color/image_raw
```

If that prints no messages, either the camera isn't running, the topic name is wrong, or there's a QoS mismatch.

**Video plays back as a single frame for the whole duration**
Either the camera publishes with QoS the subscriber doesn't accept, or `video_fps` in the yaml doesn't match the camera's actual rate. Check `ros2 topic hz` to see the real rate and update the yaml.

**Wrong button triggers wrong action**
Run `ros2 topic echo /joy` and verify which button index maps to each physical button on your controller. Update the yaml.

**Recording produces no file or a 0-byte file**
The codec and container don't match. Use `mp4v` codec with `mp4` extension, or `MJPG` codec with `avi` extension.

## File layout

```
camera_capture/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── camera_capture.yaml
├── launch/
│   └── camera_capture.launch.py
└── camera_capture/
    ├── __init__.py
    └── camera_capture_node.py
```