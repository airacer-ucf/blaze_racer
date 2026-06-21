# blaze_slam

2-D occupancy-grid track mapping for **blaze_racer** with
[`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox), following the
standard Lab 4 online-async workflow: feed `/scan` + the `odom → base_link` TF,
build a `/map` live, and save it with `nav2_map_server`.

This package replaces `slam_mapping`. It drops the unreliable command
dead-reckoning and the auto-saver and instead uses the **real VESC `/odom`**
that now works on the car.

## The one thing that makes SLAM work here

slam_toolbox does **not** read the `/odom` *topic*. It reads the robot's motion
from the `odom → base_link` **TF transform**. On this car, `vesc_to_odom`
publishes the `/odom` topic but is configured with `publish_tf: false`
(`subsystem_stack/config/vesc.yaml`), so that transform is missing — every scan
is dropped (`Message Filter dropping message … reason 'Unknown'`) and the map
comes out unusable.

`blaze_slam` fixes this with a tiny bridge node, **`odom_tf_publisher`**, that
subscribes to `/odom` and re-broadcasts the same pose as the `odom → base_link`
TF (using the message's own timestamp). That completes the TF chain
slam_toolbox needs:

```
map ──(slam_toolbox)──► odom ──(odom_tf_publisher)──► base_link ──(static, bringup)──► laser
```

| Frame edge | Published by |
|---|---|
| `map → odom` | `slam_toolbox` (this package) |
| `odom → base_link` | `odom_tf_publisher` (this package) — or `vesc_to_odom` if you set `publish_tf: true` |
| `base_link → laser` | static TF in `subsystem_stack/launch/bringup_launch.py` (`0.27 0 0.11`) |

> If you instead enable `publish_tf: true` on `vesc_to_odom`, **don't** run the
> bridge — two publishers of the same transform break slam_toolbox. Launch with
> `publish_odom_tf:=false`.

## Build

```bash
cd ~/blaze_racer/autonomy_ws
rosdep install --from-paths src --ignore-src -r -y   # slam_toolbox, nav2_map_server
colcon build --packages-select blaze_slam
source install/setup.bash
```

## Run (mapping)

**1. Subsystem stack** (provides `/scan`, `/odom`, the `base_link → laser` TF) —
in its own terminal:

```bash
cd ~/blaze_racer/subsystem_ws && source install/setup.bash
ros2 launch subsystem_stack bringup_launch.py
```

**2. SLAM mapping** (this package):

```bash
cd ~/blaze_racer/autonomy_ws && source install/setup.bash
ros2 launch blaze_slam online_async.launch.py rviz:=true
```

**3. Drive the track** — teleop with the joystick, or run an autonomy package
(e.g. `follow_the_gap`). Complete at least one full lap; the loop closure
straightens the map.

Watch the map build in RViz (`Fixed Frame: map`, `Map` on `/map`, `LaserScan`
on `/scan`).

### TF sanity check (do this before mapping)

```bash
ros2 run tf2_ros tf2_echo odom base_link     # must print continuously
ros2 run tf2_ros tf2_echo map base_link      # appears once slam_toolbox is up
```

If `odom → base_link` does **not** print, slam_toolbox will drop every scan —
make sure exactly one of (`odom_tf_publisher`, `vesc_to_odom publish_tf`) is
publishing it.

## Save the map

When the map looks good, save it (the standard slam_toolbox / nav2 way):

```bash
mkdir -p ~/maps

# Occupancy grid (.pgm/.png + .yaml) for nav2_map_server:
ros2 run nav2_map_server map_saver_cli -t /map -f ~/maps/blaze_track

# Serialized pose graph (.posegraph + .data) for re-localization:
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/maps/blaze_track'}"
```

View it: `eog ~/maps/blaze_track.pgm`

## Localization on a saved map

```bash
ros2 launch blaze_slam localization.launch.py \
    map_file_name:=$HOME/maps/blaze_track rviz:=true
```

Loads the serialized graph and localizes the car on it without changing the map.

## Launch arguments

| Argument | Default | Purpose |
|---|---|---|
| `rviz` | `false` | Open RViz2 with the SLAM view |
| `publish_odom_tf` | `true` | Run the `/odom` → `odom→base_link` TF bridge; set `false` if `vesc_to_odom` already publishes that TF |
| `odom_topic` | `/odom` | Odometry topic to bridge |
| `slam_params_file` | `config/mapper_params_online_async.yaml` | Override slam_toolbox parameters |
| `use_sim_time` | `false` | Set `true` for simulation / rosbag replay (with `--clock`) |
| `map_file_name` | `''` | (localization only) serialized graph to load, path without extension |

## Tuning (`config/mapper_params_online_async.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `resolution` | `0.05` | Metres per pixel (5 cm). Lower = finer, bigger, slower |
| `max_laser_range` | `15.0` | Max Hokuyo range used for mapping |
| `minimum_travel_distance` / `_heading` | `0.2` / `0.2` | Motion before a new keyframe |
| `do_loop_closing` | `true` | Straightens the map when the lap closes |
| `base_frame` | `base_link` | **Must be `base_link`, not `laser`** |

## Mapping from a rosbag (offline)

```bash
ros2 launch blaze_slam online_async.launch.py use_sim_time:=true rviz:=true
ros2 bag play <bag> --clock
```
