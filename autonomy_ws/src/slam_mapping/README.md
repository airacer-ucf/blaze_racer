# slam_mapping

Builds a 2-D occupancy-grid map of the track with
[`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) while the car is
driven **manually or autonomously**, and **automatically saves the map the
moment a lap is completed** (loop closure).

The output is a `.png` + `.yaml` pair ready to load with `nav2_map_server` / for localization.

## How it works

| Node | Package | Role |
|---|---|---|
| `cmd_odometry` | `slam_mapping` | Publishes the `odom → base_link` TF + `/odom` from the commanded VESC values (see below). |
| `slam_toolbox` (`async_slam_toolbox_node`) | `slam_toolbox` | Consumes `/scan` and the `odom → base_link` TF; publishes `/map` and the `map → odom` TF. |
| `loop_closure_map_saver` | `slam_mapping` | Watches `/odom`; when the car finishes a lap, saves `/map` to disk. |

## Odometry on a sensorless car

slam_toolbox does **not** read the `/odom` topic — it gets the robot's motion
from the `odom → base_link` **TF**. On this car the motor has no hall sensor or
encoder, and the VESC telemetry (`/sensors/core`) is unreliable, so
`vesc_to_odom` cannot supply that TF — every scan gets dropped
(`Message Filter dropping message ... reason 'Unknown'`).

`cmd_odometry` solves this by dead-reckoning from the values actually commanded
to the VESC (`/commands/motor/speed` + `/commands/servo/position`), converted
back to m/s + steering with the VESC gains and integrated through a bicycle
model. This is only a *prior*: slam_toolbox refines each scan with scan matching
and corrects drift on loop closure, so an open-loop estimate is enough to build
a clean map — and it works the same whether you drive manually or autonomously.

It is enabled by default (`use_cmd_odometry:=true`). If a real
wheel-odometry source that publishes `odom → base_link` is used, launch with
`use_cmd_odometry:=false` (and set `publish_tf: false` on whichever you don't
want) so there is only one publisher of that transform.

**Loop-closure trigger.** slam_toolbox does not publish a loop-closure event, so
the saver uses a deterministic geometric condition that coincides with it on a
closed circuit: the car must drive at least `departure_radius` away from where
mapping began, then return within `loop_closure_radius` of that start point
after covering at least `min_lap_distance` of path. The map is saved once.

Map writing is delegated to `nav2_map_server map_saver_cli`, so the result is
identical to saving by hand. Thresholds default to `occupied_thresh: 0.45` /
`free_thresh: 0.196`.

## Build

```bash
cd ~/blaze_racer/autonomy_ws
rosdep install --from-paths src --ignore-src -r -y   # pulls slam_toolbox, nav2_map_server
colcon build --packages-select slam_mapping
source install/setup.bash
```

## Run

The subsystem stack (which provides `/scan` and `/odom`) must be running first.
Then drive the car around the track — either by teleop or by launching an
autonomy package such as `follow_the_gap` — while this runs:

```bash
ros2 launch slam_mapping slam_mapping.launch.py
# watch it live in RViz:
ros2 launch slam_mapping slam_mapping.launch.py rviz:=true
```

This launches `cmd_odometry` + `slam_toolbox` + the saver. Drive the track; the
map appears in RViz and is saved automatically on loop closure.

Useful launch arguments:

| Argument | Default | Purpose |
|---|---|---|
| `rviz` | `false` | Open RViz2 with the SLAM view (`config` in `rviz/slam_mapping.rviz`) |
| `use_cmd_odometry` | `true` | Run `cmd_odometry`; set `false` if another node already publishes `odom → base_link` |
| `use_sim_time` | `false` | Set `true` in simulation (also override the `ego_racecar/*` frames in a copy of `config/slam_toolbox.yaml`) |

*Quick TF sanity check before mapping:* `ros2 run tf2_ros tf2_echo odom base_link`
should print continuously once this launch is up — that's the transform
slam_toolbox needs.

Complete one lap. On loop closure you will see:

```
[loop_closure_map_saver]: Loop closure detected (lap of 42.3 m, 0.71 m from start). Saving map...
[loop_closure_map_saver]: Map saved: ~/maps/blaze_track.png and ~/maps/blaze_track.yaml
```

Visualize SLAM live in RViz2 (`Fixed Frame: map`, add a `Map` display on
`/map` and a `LaserScan` on `/scan`).

## Key parameters

`config/loop_closure_saver.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `map_save_dir` / `map_name` | `~/maps` / `blaze_track` | Output location and base filename |
| `image_format` / `map_mode` | `png` / `trinary` | Saved image format / occupancy encoding |
| `occupied_thresh` / `free_thresh` | `0.45` / `0.196` | Occupancy thresholds written to the `.yaml` |
| `min_lap_distance` | `8.0` | Path length (m) required before a lap can close — set near your track length |
| `loop_closure_radius` | `1.0` | Distance (m) from start that counts as "back home" |
| `departure_radius` | `2.5` | Distance (m) the car must first leave the start by |
| `serialize_pose_graph` | `true` | Also save `slam_toolbox` pose graph (`.posegraph`/`.data`) for localization |
| `shutdown_after_save` | `false` | Stop the saver process once the map is written |

`config/slam_toolbox.yaml` holds the SLAM tuning (frames, resolution, loop
closure). It defaults to the real car's `odom` / `base_link` frames.

## Saving manually

If you want to save before a lap closes, the standard tools still work:

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f ~/maps/blaze_track \
    --fmt png --occ 0.45 --free 0.196
```
