# Pure Pursuit

A ROS 2 package implementing a Pure Pursuit waypoint-following planner for the
blaze_racer platform. Given a recorded racing line (waypoints plus a velocity
profile) and a source of localization, the node steers the car along the path
and publishes Ackermann drive commands. The package also ships helpers for
recording waypoints by driving the track manually and for visualizing a path
in RViz.

This package follows the same structure as `follow_the_gap`: all ROS plumbing
lives in `autonomy_core.base_drive_node.BaseDriveNode`, and the tracking math
lives in a ROS-free module (`pure_pursuit.py`) that is unit tested on its own.

---

## Algorithm Overview

Pure pursuit is a geometric path tracker. On every control cycle it:

1. **Finds the nearest waypoint** to the car's current position.
2. **Picks a goal point** by walking forward along the path (wrapping around,
   since the track is a closed loop) to the first waypoint that is at least one
   `lookahead_distance` away.
3. **Transforms the goal into the vehicle frame** (x forward, y left) using the
   car's pose.
4. **Solves the steering angle** with the bicycle model:
   `delta = atan2(2 * wheelbase * sin(alpha), lookahead)`, where `alpha` is the
   bearing to the goal point. The result is clamped to `max_steer`.
5. **Sets the speed** from the goal waypoint's velocity profile, scaled by
   `velocity_gain` and clamped to `[min_speed, max_speed]`.

The lookahead distance can grow with speed (`lookahead_gain`) so the car looks
further ahead when moving fast. Set `lookahead_gain` to 0 for a fixed lookahead.

The control loop is clocked by the `/scan` callback inherited from
`BaseDriveNode`. The scan data itself is not used by the controller, but
running off the scan gives one free safety behavior: if the LiDAR feed dies,
the base class publishes a stop command. The LiDAR must therefore be streaming
for the car to move.

---

## Topics

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/odom` (param `odom_topic`) | Subscribed | `nav_msgs/Odometry` | Car pose in the map frame |
| `/scan` (param `scan_topic`) | Subscribed | `sensor_msgs/LaserScan` | Loop clock and dead-LiDAR guard only |
| `/drive` (param `drive_topic`) | Published | `ackermann_msgs/AckermannDriveStamped` | Steering and speed commands |
| `/pure_pursuit/markers` (param `marker_topic`) | Published | `visualization_msgs/Marker` | Path, waypoints (colored by speed), and live goal point |

---

## Parameters

See `config/params.yaml` for the deployed values.

| Parameter | Default | Description |
|---|---|---|
| `waypoints_file` | `example_waypoints.csv` | Path source. Bare name resolves to the package `waypoints/` dir; absolute path used as-is. |
| `waypoints_delimiter` | `,` | Field separator (`,` for the logger, `;` for TUM raceline exports). |
| `x_col`, `y_col`, `velocity_col` | `0`, `1`, `2` | Column indices in the CSV. |
| `default_velocity` | `1.0` | Speed used when a row has no velocity column (m/s). |
| `odom_topic` | `/odom` | Map-frame pose source. See "Localization" below. |
| `wheelbase` | `0.33` | Front-to-rear axle distance (m). |
| `lookahead_distance` | `1.0` | Base lookahead distance (m). |
| `lookahead_gain` | `0.0` | Extra lookahead per m/s of speed. |
| `min_lookahead`, `max_lookahead` | `0.5`, `3.0` | Clamps on lookahead (m). |
| `max_steer` | `0.349066` | Steering limit, 20 deg (rad). |
| `velocity_gain` | `1.0` | Global multiplier on profile speed. |
| `min_speed`, `max_speed` | `0.0`, `7.0` | Speed clamps (m/s). |
| `map_frame` | `map` | Frame the waypoints live in. |
| `publish_markers` | `true` | Publish RViz markers. |
| `marker_period` | `0.5` | Seconds between path re-publishes. |

---

## Waypoint File Format

A CSV with one waypoint per row. The canonical format produced by the logger
is:

```
x_m,y_m,velocity_mps
-3.000000,-2.500000,5.500000
-2.850000,-2.500000,5.500000
...
```

The loader auto-skips a non-numeric header row and tolerates `#` comments. To
use a differently shaped file (for example a TUM `;`-separated raceline), set
`waypoints_delimiter` and the `*_col` parameters. If the velocity column is
missing, `default_velocity` is used for every point.

`waypoints/example_waypoints.csv` is a generated closed-loop stadium track with
a curvature-based velocity profile (slow in the corners, fast on the straights)
so the package runs out of the box. Replace it with your own track file.

---

## File Layout

```
pure_pursuit/
  pure_pursuit/
    pure_pursuit.py              ROS-free pursuit geometry (unit tested)
    waypoint_loader.py           ROS-free CSV load/save (unit tested)
    markers.py                   RViz marker builders + quaternion helper
    pure_pursuit_node.py         the controller node (subclasses BaseDriveNode)
    waypoint_logger_node.py      record waypoints by driving manually
    waypoint_visualizer_node.py  publish a CSV as markers, no driving
  config/params.yaml
  launch/
    pure_pursuit.launch.py
    waypoint_logger.launch.py
    waypoint_visualizer.launch.py
  rviz/pure_pursuit.rviz
  waypoints/example_waypoints.csv
  test/                          copyright / flake8 / pep257 + unit tests
```

---

## Build

From the workspace root (Ubuntu 20.04, ROS 2 Foxy):

```
cd ~/autonomy_ws
colcon build --packages-select autonomy_core pure_pursuit
source install/setup.bash
```

`autonomy_core` must be built first (or in the same command) because this
package depends on it.

---

## Usage

### 1. Localization (providing the map and pose)

Pure pursuit needs the car's pose in the same frame as the waypoints (the
`map` frame). Pure pursuit does not localize on its own; it consumes the
odometry your existing stack publishes. Bring up your map and localization the
usual way, for example a `nav2` `map_server` plus a particle filter / AMCL, or
the simulator. Then point `odom_topic` at whatever publishes the map-frame
pose as `nav_msgs/Odometry`:

- f1tenth gym sim: `odom_topic: /ego_racecar/odom`
- real car with a particle filter: the Odometry your PF publishes
- raw wheel odometry (drifts, fine for quick tests): `/odom`

To show the map behind the path in RViz, run a map server, e.g.:

```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml
ros2 run nav2_util lifecycle_bringup map_server
```

### 2. Record a racing line (optional)

With localization running, drive the car manually (keyboard teleop, joystick,
or even Follow-the-Gap) while logging:

```
ros2 launch pure_pursuit waypoint_logger.launch.py \
  odom_topic:=/odom output_file:=my_track.csv min_distance:=0.2
```

A waypoint is saved each time the car moves `min_distance` metres, tagged with
its current speed. Drive one clean lap and stop the node (Ctrl-C); the absolute
path of the saved CSV is printed on shutdown. Hand-tune the velocity column
afterward if you want a sharper profile.

### 3. Visualize a path (optional)

Check a CSV in RViz before driving it:

```
ros2 launch pure_pursuit waypoint_visualizer.launch.py \
  waypoints_file:=my_track.csv rviz:=true
```

The line is green; individual waypoints are colored blue (slow) to red (fast).

### 4. Drive the track

Point `waypoints_file` and `odom_topic` in `config/params.yaml` at your track
and pose source, then:

```
ros2 launch pure_pursuit pure_pursuit.launch.py rviz:=true
```

Or override the config file:

```
ros2 launch pure_pursuit pure_pursuit.launch.py config:=/path/to/params.yaml
```

The yellow sphere in RViz is the live lookahead goal point.

---

## Tuning

- **Cutting corners / understeer:** lower `lookahead_distance`. Shorter
  lookahead tracks the line more tightly but can oscillate if too short.
- **Oscillation / weaving:** raise `lookahead_distance`, or add
  `lookahead_gain` so the lookahead grows with speed.
- **Too slow / too fast overall:** scale the whole profile with
  `velocity_gain`, or edit the velocity column. Use `max_speed` as a safety cap
  while testing.
- **Wrong wheelbase** makes the steering systematically off; measure your car's
  axle-to-axle distance and set `wheelbase`.

Start slow: set `max_speed` low (e.g. 1.5) and `velocity_gain` to 1.0, confirm
the line is tracked, then raise the cap.

---

## Testing

```
colcon test --packages-select pure_pursuit
colcon test-result --verbose
```

The unit tests in `test/test_pure_pursuit.py` cover the steering geometry
(straight line, left/right offset, clamping), the speed shaping, and the
waypoint loader (round trip, header skipping, velocity fill). The
`copyright`, `flake8`, and `pep257` tests enforce the workspace style.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Car does not move | No odometry on `odom_topic`, or LiDAR not publishing on `/scan` (the base class stops the car without a valid scan). |
| Markers do not appear in RViz | Fixed frame is not `map`, or `map_frame` does not match your localization frame. |
| Car drives off the line immediately | `odom_topic` pose is in a different frame than the waypoints, or the waypoints are in the wrong units/columns. |
| Steering feels inverted | Pose yaw convention mismatch, or x/y columns swapped in the CSV. |
| "No valid waypoint rows parsed" | Wrong `waypoints_delimiter` or `*_col` for your file format. |
