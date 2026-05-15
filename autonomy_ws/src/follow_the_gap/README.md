# Follow the Gap / Disparity Extender

A ROS 2 package implementing a reactive gap-following planner for the blaze_racer platform. The node reads raw LiDAR scans and publishes Ackermann drive commands in real time with no map, localization, or prior environment knowledge required.

---

## Algorithm Overview

The planner runs entirely inside a single LiDAR callback. Each scan goes through the following steps:

**1. Preprocessing**
Range values above `max_lidar_dist` are clipped and `inf` readings are replaced with `max_lidar_dist`. A moving-average filter of width `preprocess_conv_size` is applied across the full scan array to reduce noise spikes.

**2. Safety bubble**
The index of the closest point in the processed scan is found. All indices within `bubble_radius` of that point are zeroed out, carving a bubble of forbidden space around the nearest obstacle.

**3. Gap finding**
The remaining non-zero scan is split into contiguous segments. The longest segment whose width exceeds `safe_threshold` indices is selected as the target gap. If no segment meets the threshold, the widest available segment is used as a fallback.

**4. Best point selection**
A second sliding-window average of width `best_point_conv_size` is applied over the selected gap. The index with the highest averaged range value is chosen as the heading target. If the gap is narrower than the window, the midpoint of the gap is used instead.

**5. Steering and speed**
The target index is converted to a LiDAR angle relative to the scan center, then halved to produce a steering command (halving reduces oversteer). Speed is selected based on the magnitude of the steering angle:

| Steering angle | Speed |
|---|---|
| Greater than `straights_steering_angle` (8 deg) | `corners_speed` |
| Greater than `fast_steering_angle` (4 deg) | `straights_speed` |
| At or below `fast_steering_angle` | `fast_speed` |

---

## Topics

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/scan` | Subscribed | `sensor_msgs/LaserScan` | Raw LiDAR input |
| `/drive` | Published | `ackermann_msgs/AckermannDriveStamped` | Steering and speed commands |

---

## Parameters

All parameters are declared in `config/params.yaml` and loaded at launch.

| Parameter | Default | Description |
|---|---|---|
| `bubble_radius` | 30 | Indices zeroed on each side of the closest point |
| `preprocess_conv_size` | 3 | Moving-average window width for scan preprocessing |
| `max_lidar_dist` | 10.0 m | Ranges above this value are clipped |
| `safe_threshold` | 15 | Minimum gap width in indices to be considered drivable |
| `best_point_conv_size` | 200 | Sliding-window width for best-point selection inside the gap |
| `max_steer` | 1.181 rad (67.7 deg) | Hard clamp on output steering angle |
| `straights_steering_angle` | 0.1396 rad (8 deg) | Boundary between corner speed and straight speed |
| `fast_steering_angle` | 0.0698 rad (4 deg) | Boundary between straight speed and fast speed |
| `corners_speed` | 1.5 m/s | Speed used in tight corners |
| `straights_speed` | 2.0 m/s | Speed used on mild curves |
| `fast_speed` | 3.5 m/s | Speed used on open straights |

---

## Build and Launch

```bash
cd ~/blaze_racer/autonomy_ws
colcon build --packages-select follow_the_gap
source install/setup.bash
ros2 launch follow_the_gap follow_the_gap.launch.py
```

The subsystem workspace bringup must already be running before launching this node.

---

## Tuning Guide

**The car clips obstacles on corners**
Increase `bubble_radius` to widen the safety bubble around the nearest obstacle, or lower `corners_speed`.

**The car hesitates or stops unnecessarily in open space**
Lower `safe_threshold`. A high threshold causes the planner to reject valid gaps that are technically wide enough to drive through.

**Steering is jerky or oscillates**
Increase `preprocess_conv_size` to smooth the scan more aggressively. Also check that `best_point_conv_size` is large relative to the gap width so the best-point selection averages over a broad region.

**The car is too slow on straights**
Raise `fast_speed`, or lower `fast_steering_angle` so the fast speed tier activates at slightly wider steering angles.

**The car cuts corners too aggressively**
Raise `straights_steering_angle` to keep the car in the corner speed tier for a longer portion of each turn.

---

## Package Layout

```
follow_the_gap/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── params.yaml
├── launch/
│   └── follow_the_gap.launch.py
└── follow_the_gap/
    ├── __init__.py
    └── follow_the_gap_node.py
```