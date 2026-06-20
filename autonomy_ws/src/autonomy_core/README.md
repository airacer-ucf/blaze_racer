# autonomy_core

Shared building blocks for the autonomy workspace. This package contains **no
algorithm of its own** — it provides the common scaffolding so that every
reactive planner (follow-the-gap today, pure pursuit / wall-follow / MPC / RL
tomorrow) is a small, focused, testable piece of code.

## What's inside

### `BaseDriveNode` (`base_drive_node.py`)

An abstract ROS 2 node that owns all the plumbing a drive algorithm shouldn't
have to repeat:

- declares parameters from a single dict and exposes them as `self.params`,
- subscribes to `/scan` (and optionally `/odom`),
- publishes `ackermann_msgs/AckermannDriveStamped` to `/drive`,
- runs a **safety stop** whenever the LiDAR feed is missing/invalid or the
  algorithm declines to produce a command,
- does rate-limited status logging.

### `lidar_utils` (`lidar_utils.py`)

Pure NumPy helpers with no ROS dependency, so they can be unit-tested directly:

- `crop_to_fov(...)` — keep only the forward field of view, using the scan's
  real `angle_min` / `angle_increment` (works for any LiDAR FOV, not just 360°).
- `smooth(...)` — moving-average noise suppression.
- `clip_ranges(...)` — clamp ranges (also removes `inf`).
- `index_to_steering(...)` — convert a chosen beam index to a clamped steering
  angle from the beam's true bearing.

## Writing a new algorithm

1. Create an `ament_python` package that depends on `autonomy_core`.
2. Put the algorithm in a plain Python class (no ROS) — easy to unit-test.
3. Add a node that subclasses `BaseDriveNode`:

   ```python
   from autonomy_core.base_drive_node import BaseDriveNode

   DEFAULT_PARAMS = {'my_gain': 1.0, 'max_steer': 0.349}

   class MyNode(BaseDriveNode):
       def __init__(self):
           super().__init__('my_node', DEFAULT_PARAMS, uses_odom=False)
           self.planner = MyPlanner(self.params)

       def compute_drive(self, scan_msg):
           # return (steering_angle_rad, speed_mps), or None to stop
           return self.planner.plan(scan_msg)
   ```

The base class handles the rest. See `follow_the_gap` for a complete reference
implementation.
