# Subsystem Workspace Packages

ROS 2 packages that make up the vehicle's subsystem stack. A single bringup
launch file enables drive-by-wire, teleoperation, sensor drivers, and camera
streaming.

## Quick Start

### Install Dependencies if Missing

**If you had never initialized `rosdep` before, run:*
```bash
sudo rosdep init
```

Go to the `susbsytem_ws` workspace where the folder `src` resides

Run the following:

```bash
rosdep update
rosdep install --from-paths src -i -y
```

*Might need to run `rosdep update --include-eol-distros`*

### Build Workspace

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch subsystem_stack bringup.launch.py
```

## Deadman's Switch

A deadman's switch must be held for control inputs to take effect. Defaults
by controller:

| Controller       | Teleop      | Navigation  |
|------------------|-------------|-------------|
| Logitech F-710   | LB          | RB          |
| PS4 (DualShock)  | L1          | R1          |

Button bindings are configurable in the joy_teleop parameter file.

## Steering Axis Configuration

Steering axis mappings differ by controller and must be set correctly in the configuration file under:

```
human_control -> drive-steering_angle
```

Use the following defaults:

| Controller      | Axis |
| --------------- | ---- |
| PS4 (DualShock) | 2    |
| Logitech F-710  | 3    |

Ensure the correct axis is set to achieve proper steering response during teleoperation.

---

## Mapping Joystick Controls

If during teleoperation the joystick isn't responding as expected, you may need to remap the joystick axes in the `joy_teleop.yaml` file. Here's how to do it:

**Locate the Configuration File**
The file you need to edit is located at:

```
.../subsystem_stack/config/joy_teleop.yaml
```

**Launch the Bringup and Check the Joy Topic**
To identify the joystick mapping, launch the bringup system and monitor the `/joy` topic to inspect joystick input.

Run the following command:

```
ros2 topic echo /joy
```

**Move the Joystick**
As you move the joystick in different directions, observe the values in the echoed message. The indices in the `axes` array that change correspond to the axis IDs for each joystick movement.

**Modify the YAML**
Once you've identified the correct indices (axis IDs), update the `joy_teleop.yaml` file under the `human_control` section to reflect the correct mappings.

**Rebuild the Package (if needed)**
After making changes, rebuild your workspace to ensure updates take effect:

```
colcon build
```

Then source your workspace again:

```
source install/setup.bash
```

---

## Topics

### Subscribed by the Driver Stack

| Topic    | Type                    | Purpose                          |
|----------|-------------------------|----------------------------------|
| `/drive` | `AckermannDriveStamped` | Autonomous navigation commands   |

### Published by the Driver Stack

| Topic               | Type         | Purpose                            |
|---------------------|--------------|------------------------------------|
| `/scan`             | `LaserScan`  | Hokuyo LiDAR scans                 |
| `/odom`             | `Odometry`   | Wheel odometry from the VESC       |
| `/sensors/imu/raw`  | `Imu`        | Raw IMU data from the VESC         |
| `/sensors/core`     | `VescStateStamped` | VESC telemetry              |

### Published by the Camera Stack

| Topic                       | Type    | Source                |
|-----------------------------|---------|-----------------------|
| `/camera/color/image_raw`   | `Image` | RealSense D435i RGB   |

## External Dependencies

| Package         | Purpose                          | Link                                                            |
|-----------------|----------------------------------|-----------------------------------------------------------------|
| `ackermann_msgs`| Ackermann command messages       | https://index.ros.org/r/ackermann_msgs                          |
| `urg_node`      | Hokuyo LiDAR driver              | https://index.ros.org/p/urg_node                                |
| `joy`           | Joystick driver                  | https://index.ros.org/p/joy                                     |
| `teleop_tools`  | Joystick teleop                  | https://index.ros.org/p/teleop_tools                            |
| `vesc`          | VESC motor controller driver     | https://github.com/f1tenth/vesc/tree/ros2                       |
| `ackermann_mux` | Multiplexer for Ackermann msgs   | https://github.com/f1tenth/ackermann_mux                        |
| `cv_bridge`     | OpenCV / ROS image bridge        | https://index.ros.org/p/cv_bridge                               |
| `python3-opencv`| OpenCV Python bindings           | apt: `sudo apt install python3-opencv`                          |

<!-- rosbridge_suite (https://index.ros.org/p/rosbridge_suite) is optional
     for WebSocket connectivity. -->

Install ROS dependencies in one shot from the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Packages in This Repository

| Package          | Description                                                |
|------------------|------------------------------------------------------------|
| `subsystem_stack`| Bringup launch file and parameter files for the full stack |
| `realsense_rgb`  | RGB-only RealSense D435i node |

### realsense_rgb

A lightweight node that streams the D435i's RGB sensor

Standalone launch:

```bash
ros2 launch realsense_rgb realsense_rgb.launch.py
```

When `subsystem_stack` is launched, `realsense_rgb_node` is included
automatically.

## Nodes Launched by Bringup

1. `joy` - joystick driver
2. `joy_teleop` - joystick to Ackermann command mapping
3. `ackermann_to_vesc_node` - converts Ackermann commands to VESC commands
4. `vesc_to_odom_node` - integrates VESC telemetry into odometry
5. `vesc_driver_node` - low-level VESC driver
6. `urg_node` - Hokuyo LiDAR driver
7. `ackermann_mux` - multiplexer for Ackermann command sources
8. `realsense_rgb_node` - RealSense D435i RGB stream

## Node Parameters and Topics

### vesc_driver

**Parameters**

- `duty_cycle_min`, `duty_cycle_max`
- `current_min`, `current_max`
- `brake_min`, `brake_max`
- `speed_min`, `speed_max`
- `position_min`, `position_max`
- `servo_min`, `servo_max`

**Publishes**

- `sensors/core`
- `sensors/servo_position_command`
- `sensors/imu`
- `sensors/imu/raw`

**Subscribes**

- `commands/motor/duty_cycle`
- `commands/motor/current`
- `commands/motor/brake`
- `commands/motor/speed`
- `commands/motor/position`
- `commands/servo/position`

### ackermann_to_vesc

**Parameters**

- `speed_to_erpm_gain`
- `speed_to_erpm_offset`
- `steering_angle_to_servo_gain`
- `steering_angle_to_servo_offset`

**Publishes**

- `commands/motor/speed`
- `commands/servo/position`

**Subscribes**

- `ackermann_cmd`

### vesc_to_odom

**Parameters**

- `odom_frame`
- `base_frame`
- `use_servo_cmd_to_calc_angular_velocity`
- `speed_to_erpm_gain`
- `speed_to_erpm_offset`
- `steering_angle_to_servo_gain`
- `steering_angle_to_servo_offset`
- `wheelbase`
- `publish_tf`

**Publishes**

- `odom`

**Subscribes**

- `sensors/core`
- `sensors/servo_position_command`

### throttle_interpolator

**Parameters**

- `rpm_input_topic`, `rpm_output_topic`
- `servo_input_topic`, `servo_output_topic`
- `max_acceleration`
- `speed_max`, `speed_min`
- `throttle_smoother_rate`
- `speed_to_erpm_gain`
- `max_servo_speed`
- `steering_angle_to_servo_gain`
- `steering_angle_to_servo_offset`
- `servo_smoother_rate`
- `servo_max`, `servo_min`

**Publishes**

- The topic named by `rpm_output_topic`
- The topic named by `servo_output_topic`

**Subscribes**

- The topic named by `rpm_input_topic`
- The topic named by `servo_input_topic`

### realsense_rgb_node

**Parameters**

- `device` (default: `/dev/realsense_rgb`) - V4L2 device path or symlink
- `width` (default: 640)
- `height` (default: 480)
- `fps` (default: 30)
- `topic` (default: `/camera/color/image_raw`)
- `frame_id` (default: `camera_color_optical_frame`)
- `fourcc` (default: `YUYV`) - V4L2 pixel format; common alternatives are
  `MJPG` and `YUY2`

**Publishes**

- The topic named by `topic` (default `/camera/color/image_raw`), as
  `sensor_msgs/Image` with `bgr8` encoding

**Subscribes**

- None
