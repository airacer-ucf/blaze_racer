# blaze_slam

SLAM Toolbox track mapping for the blaze_racer platform. Drives SLAM Toolbox
in online async mapping mode and automatically saves the map (png + yaml, the
same format the F1Tenth gym sim and Pure Pursuit expect) whenever a loop
closure is detected.

## What it launches

1. `slam_toolbox` (async_slam_toolbox_node) in mapping mode.
2. `map_saver` (nav2_map_server map_saver_server) so maps can be written as
   png/trinary on demand.
3. `lifecycle_manager_slam` to autostart the map saver.
4. `map_autosaver` (this package) which detects loop closures and saves.

## Build

```bash
cd ~/blaze_racer        # or wherever your workspace root is
colcon build --packages-select blaze_slam --symlink-install
source install/setup.bash
```

If slam_toolbox / nav2 are not installed:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Run

`/scan` and the `odom -> base_link` TF must already be publishing, so start
the car (or the sim) first, then launch this.

On the car:

```bash
ros2 launch subsystem_stack bringup_launch.py
ros2 launch blaze_slam blaze_slam_launch.py
```

In the F1Tenth gym sim:

```bash
ros2 launch blaze_slam blaze_slam_launch.py use_sim_time:=true
```

Drive the track (teleop) until you complete a lap. When SLAM Toolbox closes
the loop, the map is written automatically to `~/blaze_racer/maps`.

## Saving behavior

Maps are saved to `~/blaze_racer/maps` as `<map_name>.png` + `<map_name>.yaml`
(default `map_name` is `track`). With `serialize_posegraph` on, it also writes
`<map_name>.posegraph` + `<map_name>.data` for later localization or continued
mapping.

Override vs rename when a map of the same name already exists:

```bash
# keep the old map, write track_1, track_2, ... (default)
ros2 launch blaze_slam blaze_slam_launch.py save_policy:=rename

# replace the existing map in place
ros2 launch blaze_slam blaze_slam_launch.py save_policy:=overwrite
```

Manual save at any time:

```bash
ros2 service call /map_autosaver/save_map std_srvs/srv/Trigger
```

## Launch arguments

| Argument                   | Default               | Purpose                                   |
|----------------------------|-----------------------|-------------------------------------------|
| `use_sim_time`             | `false`               | `true` for the gym sim                    |
| `maps_dir`                 | `~/blaze_racer/maps`  | Output directory                          |
| `map_name`                 | `track`               | Base name for the map files               |
| `save_policy`              | `rename`              | `overwrite` or `rename`                   |
| `autosave_on_loop_closure` | `true`                | Save automatically on each loop closure   |
| `serialize_posegraph`      | `true`                | Also write `.posegraph`/`.data`           |
| `min_save_interval_sec`    | `10.0`                | Debounce between automatic saves          |
| `params_file`              | `config/blaze_slam.yaml` | SLAM Toolbox parameters                |

## Using the map later

For sim or Pure Pursuit, point nav2 / map_server at the saved yaml:

```bash
~/blaze_racer/maps/track.yaml
```

To run SLAM Toolbox in localization mode against a saved track, load the
serialized graph (`track.posegraph`) with `localization_slam_toolbox_node`.

## How loop closure detection works

SLAM Toolbox does not publish a discrete loop-closure event. It publishes its
pose graph on `/slam_toolbox/graph_visualization`. In a single trajectory the
graph is a chain (`edges == nodes - 1`); each loop closure adds one extra
constraint edge. `map_autosaver` counts `edges - (nodes - 1)` and saves
whenever that count increases. `do_loop_closing` must stay `true` in the
config for this to work.
