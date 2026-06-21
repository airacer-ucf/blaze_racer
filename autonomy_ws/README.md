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

| Package | Role | Sensors Used |
|---|---|---|
| `autonomy_core` | Shared library: `BaseDriveNode` + `lidar_utils` (no algorithm) | — |
| `follow_the_gap` | Reactive gap-following planner | LiDAR (`/scan`) |
| `slam_mapping` | SLAM track mapping; auto-saves the map on loop closure | LiDAR (`/scan`), odom (`/odom`) |

> `slam_mapping` is a perception/mapping package, not a driving algorithm — it
> consumes `/scan` + `/odom` and produces a saved map rather than `/drive`. Run
> it alongside teleop or any planner to map a track. See
> [`slam_mapping/README.md`](src/slam_mapping/README.md).

---

## Architecture: how algorithms stay modular

Every algorithm is built on `autonomy_core`, which owns the repetitive ROS
plumbing so each planner is small and testable:

- **`BaseDriveNode`** (`autonomy_core/base_drive_node.py`) — handles parameter
  loading, the `/scan` (and optional `/odom`) subscriptions, `/drive`
  publishing, the safety stop on bad/missing LiDAR, and status logging. A
  subclass only implements `compute_drive(scan_msg) -> (steering, speed) | None`.
- **`lidar_utils`** (`autonomy_core/lidar_utils.py`) — ROS-free NumPy helpers
  (FOV cropping, smoothing, clipping, beam-index-to-steering) shared across
  algorithms and unit-testable on their own.

The algorithm itself lives in a plain Python class with no ROS imports (see
`follow_the_gap/planner.py`), so it can be unit-tested with synthetic scans.

---

## Adding a New Package

1. Create the package inside `src/`, depending on `autonomy_core`:

```bash
cd ~/blaze_racer/autonomy_ws/src
ros2 pkg create --build-type ament_python <package_name> \
    --dependencies rclpy sensor_msgs ackermann_msgs autonomy_core
```

2. Put the algorithm in a plain Python class (no ROS) — easy to unit-test.

3. Add a node that subclasses `BaseDriveNode` and implements `compute_drive`:

```python
from autonomy_core.base_drive_node import BaseDriveNode
from .planner import MyPlanner

DEFAULT_PARAMS = {'max_steer': 0.349, 'speed': 2.0}  # algorithm tunables


class MyNode(BaseDriveNode):
    def __init__(self):
        super().__init__('my_node', DEFAULT_PARAMS, uses_odom=False)
        self.planner = MyPlanner(self.params)

    def compute_drive(self, scan_msg):
        # return (steering_angle_rad, speed_mps), or None to stop
        return self.planner.plan(
            scan_msg.ranges, scan_msg.angle_min, scan_msg.angle_increment)
```

4. Add a launch file and a `config/params.yaml` (declare the same keys as
   `DEFAULT_PARAMS`). The subsystem stack needs no changes — the `/scan` →
   `/drive` contract is unchanged.

5. Build and source:

```bash
cd ~/blaze_racer/autonomy_ws
colcon build --packages-select <package_name>
source install/setup.bash
```