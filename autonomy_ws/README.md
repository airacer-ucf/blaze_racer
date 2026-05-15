# Autonomy Workspace

ROS 2 workspace containing autonomous navigation packages for the blaze_racer platform. Each package in `src/` is a self-contained algorithm that subscribes to sensor data and publishes drive commands to `/drive`.

The subsystem workspace must be running before launching any autonomy package, since it provides `/scan`, `/odom`, and the hardware command interface.

---

## Prerequisites

- ROS 2
- The `subsystem_ws` bringup must be active (provides `/scan`, `/odom`, `/drive` interface)

---

## Building the Workspace

```bash
cd ~/blaze_racer/autonomy_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Interface Contract

All autonomy packages in this workspace follow a common interface so they can be swapped without changes to the subsystem stack.

**Subscribed (provided by subsystem_ws)**

| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR range data from the Hokuyo |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry from the VESC |

**Published (consumed by subsystem_ws)**

| Topic | Type | Description |
|---|---|---|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Steering angle and speed commands |

The `ackermann_mux` in the subsystem stack gives the human teleop operator priority over `/drive`. Hold the navigation deadman button (RB on Logitech F-710, R1 on PS4 DualShock) to allow autonomous commands through.

---

## Packages

| Package | Algorithm | Sensors Used |
|---|---|---|
| `follow_the_gap` | Reactive gap-following planner | LiDAR (`/scan`) |

---

## Adding a New Package

1. Create the package inside `src/`:

```bash
cd ~/blaze_racer/autonomy_ws/src
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy sensor_msgs ackermann_msgs
```

2. Implement a node that subscribes to `/scan` (and optionally `/odom`) and publishes to `/drive`.

3. Add a launch file and a `config/params.yaml`.

4. Build and source:

```bash
cd ~/blaze_racer/autonomy_ws
colcon build --packages-select <package_name>
source install/setup.bash
```