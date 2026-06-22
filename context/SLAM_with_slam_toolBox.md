# Lab 4: Intro to SLAM with slam_toolbox

## Lab Overview
In this lab, you will learn about Simultaneous Localization and Mapping (SLAM) and gain proficiency with slam_toolbox, one of the most popular SLAM packages in ROS2. You'll work with a pre-recorded rosbag from a race car completing one lap around a track, using the LiDAR scan data and odometry to build a map of the environment.

---

## Learning Objectives
By the end of this lab, you will be able to:
1. Understand the fundamentals of SLAM and its importance in autonomous systems
2. Install and configure slam_toolbox for ROS2 Humble
3. Run slam_toolbox with different configurations and modes
4. Use RViz2 to visualize SLAM in real-time
5. Generate, save, and load maps from SLAM sessions
6. Tune SLAM parameters for different scenarios
7. Analyze SLAM performance and map quality

---

## Prerequisites

Ensure ROS2 Humble is installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

---

## Part 1: Understanding SLAM

### What is SLAM?

**SLAM (Simultaneous Localization and Mapping)** is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

### Why is SLAM Important?

For autonomous racing robots:
- **Map Building:** Create accurate track maps without prior knowledge
- **Localization:** Know precise position on track in real-time
- **Path Planning:** Plan optimal racing lines based on map
- **Obstacle Detection:** Identify and map dynamic obstacles

### SLAM Components

1. **Sensor Input:**
   - LiDAR scans (`/scan`) - Distance measurements in 360°
   - Odometry (`/odom`) - Estimated position from wheel encoders/IMU
   - Transforms (`/tf`) - Coordinate frame relationships

2. **SLAM Algorithm:**
   - Processes sensor data
   - Estimates robot pose (localization)
   - Builds map (mapping)
   - Handles loop closures (recognizing previously visited locations)

3. **Output:**
   - Occupancy grid map (2D representation)
   - Robot pose estimates
   - Updated transforms (map → base_link)

### Types of SLAM

- **Online SLAM:** Real-time mapping while navigating (what we'll do)
- **Offline SLAM:** Process recorded data to create map
- **2D SLAM:** Creates flat, top-down maps (our focus)
- **3D SLAM:** Creates 3D volumetric maps

---

## Part 2: Installing slam_toolbox

### What is slam_toolbox?

slam_toolbox is a comprehensive SLAM solution for ROS2 that provides:
- Multiple SLAM modes (online, offline, localization)
- Loop closure detection
- Map serialization (save/load)
- Interactive map manipulation
- Integration with Nav2 (will see this in later labs)

### Step 2.1: Install slam_toolbox

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

### Step 2.2: Verify Installation

```bash
ros2 pkg list | grep slam_toolbox
```

**Expected Output:**
```
slam_toolbox
```

### Step 2.3: Check Available Executables

```bash
ros2 pkg executables slam_toolbox
```

**Expected Similar Output:**
```
slam_toolbox async_slam_toolbox_node
slam_toolbox sync_slam_toolbox_node
...
```

### Step 2.4: Explore slam_toolbox Parameters

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml
```

Press `Ctrl+C` to stop. This shows that slam_toolbox requires a configuration file.

---

## Part 3: Preparing the Workspace (Suggestive)

### Step 3.1: Create Working Directory

```bash
mkdir -p ~/slam_lab
cd ~/slam_lab
mkdir -p maps config bags
```

**Potential Directory structure:**
```
~/slam_lab/
├── maps/      # For saving generated maps
├── config/    # For SLAM configuration files
└── bags/      # For rosbag files
```

### Step 3.2: Download/Copy the Rosbag

For this example, we'll assume the rosbag is named `racecar_lap.db3`.

> Note that rosbags can be in different formats. Some can be a singular file and some can be folders. To copy folder, use option `-r` as in `cp -r`

```bash
# Copy rosbag to folder:
cp /path/to/racecar_lap* ~/slam_lab/bags/

# Verify the bag
cd ~/slam_lab/bags
ros2 bag info racecar_lap
```

**Expected Output:**
```
Files:             racecar_lap_0.db3
Bag size:          XXX.X MiB
Storage id:        sqlite3
Duration:          XX.XXs
Start:             ...
End:               ...
Messages:          XXXX
Topic information:
  Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: XXX
  Topic: /ego_racecar/odom | Type: nav_msgs/msg/Odometry | Count: XXX
  Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: XXX
  Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: XXX
  Topic: /clock | Type: rosgraph_msgs/msg/Clock | Count: XXX
  Topic: /joint_states | Type: sensor_msgs/msg/JointState | Count: XXX
  Topic: /ego_robot_description | Type: std_msgs/msg/String | Count: XXX
```

### Step 3.3: Inspect the Topics

Let's understand what data we have:

**Play the bag briefly to inspect:**
```bash
ros2 bag play racecar_lap
```

**In another terminal, echo the scan topic:**
```bash
ros2 topic echo /scan --once
```

**Expected Output:**
```
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: ego_racecar/laser
angle_min: -3.14159...
angle_max: 3.14159...
angle_increment: 0.00436...
time_increment: 0.0
scan_time: 0.0
range_min: 0.0
range_max: 30.0
ranges:
- 5.234
- 5.189
- ...
intensities: []
```

**Check odometry:**
```bash
ros2 topic echo /ego_racecar/odom --once
```

---

## Part 4: Creating SLAM Configuration Files

### Understanding slam_toolbox Configuration

slam_toolbox uses YAML configuration files to set parameters. Let's explore the default and create our custom configuration.

### Step 4.1: Examine Default Configuration

```bash
cat /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml
```

> Note that this is the configurations that gets launch by default when you launch the default `slam_toolbox` launchfiles

**Key parameters you'll see:**
- `odom_frame`: Odometry frame ID
- `map_frame`: Map frame ID
- `base_frame`: Robot base frame ID
- `scan_topic`: LiDAR scan topic
- `resolution`: Map resolution (meters per pixel)
- `max_laser_range`: Maximum range to use from LiDAR
- Various tuning parameters for scan matching, loop closure, etc.

### Step 4.2: Create Custom Configuration for Async SLAM

In Async SLAM, data is processed in parallel as it arrives, rather than waiting for synchronized frames, allowing for faster real-time performance

**File:** `~/slam_lab/config/online_async_racecar.yaml`

```yaml
# Online Async SLAM Configuration for Race Car
# This mode continuously builds a map while the robot moves

slam_toolbox:
  ros__parameters:

    # ROS Parameters
    odom_frame: ego_racecar/odom
    map_frame: map
    base_frame: ego_racecar/base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping # mapping, localization, or lifelong
    
    # Debugging
    debug_logging: true
    throttle_scans: 1  # Process every scan (1), or skip scans (>1)
    
    # Transform publication
    transform_publish_period: 0.02  # 50 Hz
    map_update_interval: 5.0        # Update map every 5 seconds
    
    # Resolution of the map (meters per pixel)
    resolution: 0.05  # 5cm resolution - good for racing
    
    # Maximum usable range of the LiDAR
    max_laser_range: 25.0  # meters
    minimum_travel_distance: 0.2   # minimum distance to travel before processing (meters)
    minimum_travel_heading: 0.2    # minimum rotation before processing (radians)
    
    # Scan matching parameters
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    
    # Loop closure parameters
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    
    # Correlation parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Optimization parameters
    optimize_every_n_nodes: 20
    
    # Scan matcher parameters
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

**Save this file.**

### Step 4.3: Create Configuration for Sync SLAM

Sync SLAM processes scans synchronously (waits for each scan to be processed before accepting the next). This is more accurate but slower.

**File:** `~/slam_lab/config/online_sync_racecar.yaml`

```yaml
# Online Sync SLAM Configuration for Race Car
# More accurate but slower processing

slam_toolbox:
  ros__parameters:

    # ROS Parameters
    odom_frame: ego_racecar/odom
    map_frame: map
    base_frame: ego_racecar/base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping
    
    # Debugging
    debug_logging: true
    throttle_scans: 1
    
    # Transform publication
    transform_publish_period: 0.02
    map_update_interval: 2.0  # Update more frequently for sync mode
    
    # Resolution
    resolution: 0.05
    
    # Range
    max_laser_range: 25.0
    minimum_travel_distance: 0.15  # Process more frequently
    minimum_travel_heading: 0.15
    
    # Scan matching parameters (more strict for accuracy)
    scan_buffer_size: 20
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.2  # Higher threshold
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    
    # Loop closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.4
    loop_match_minimum_response_fine: 0.5
    
    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Optimization
    optimize_every_n_nodes: 10  # Optimize more frequently
    
    # Scan matcher
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

### Step 4.4: Create Configuration for Localization Mode

Localization mode uses an existing map to localize the robot (doesn't update the map).

**File:** `~/slam_lab/config/localization_racecar.yaml`

```yaml
# Localization Mode - Use existing map to localize robot

slam_toolbox:
  ros__parameters:

    # ROS Parameters
    odom_frame: ego_racecar/odom
    map_frame: map
    base_frame: ego_racecar/base_link
    scan_topic: /scan
    use_map_saver: false  # Don't save map in localization mode
    mode: localization
    
    # Map to load (will be set when loading)
    map_file_name: ""
    map_start_at_dock: true
    
    # Debugging
    debug_logging: true
    throttle_scans: 1
    
    # Transform publication
    transform_publish_period: 0.02
    
    # Resolution (must match saved map)
    resolution: 0.05
    
    # Range
    max_laser_range: 25.0
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    
    # Scan matching (tuned for localization)
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.3
    link_scan_maximum_distance: 1.5
    
    # No loop closure in localization mode
    do_loop_closing: false
    
    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Scan matcher
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

---

## Part 5: Running SLAM with Async Mode

### Step 5.1: Setup RViz Configuration

First, let's create an RViz config for SLAM visualization.

**File:** `~/slam_lab/config/slam_visualization.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Map1
        - /LaserScan1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        ego_racecar/base_link:
          Value: true
        ego_racecar/laser:
          Value: true
        map:
          Value: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          ego_racecar/base_link:
            ego_racecar/laser:
              {}
      Update Interval: 0
      Value: true
    
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    
    - Alpha: 1
      Axes Length: 0.3
      Axes Radius: 0.03
      Class: rviz_default_plugins/PoseWithCovariance
      Color: 255; 25; 0
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Head Length: 0.15
      Head Radius: 0.1
      Name: Robot Pose
      Shaft Length: 0.3
      Shaft Radius: 0.05
      Shape: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /pose
      Value: true
    
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 25
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.5697963237762451
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 3.1415927410125732
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State:
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 0
  Y: 0
```

### Step 5.2: Launch Async SLAM

Open **Terminal 1** - Launch RViz:
```bash
cd ~/slam_lab
source /opt/ros/humble/setup.bash
rviz2 -d config/slam_visualization.rviz
```

Open **Terminal 2** - Launch slam_toolbox:
```bash
cd ~/slam_lab
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=config/online_async_racecar.yaml
```

**Expected Output:**
```
[INFO] [async_slam_toolbox_node]: Node created
[INFO] [async_slam_toolbox_node]: Using solver plugin: solver_plugins::CeresSolver
[INFO] [async_slam_toolbox_node]: CeresSolver: Using SPARSE_NORMAL_CHOLESKY linear algebra.
```

Open **Terminal 3** - Play the rosbag:
```bash
cd ~/slam_lab/bags
source /opt/ros/humble/setup.bash
ros2 bag play racecar_lap
```

**What to Observe in RViz:**
1. **Map gradually appears** as the car drives
2. **Red laser scan points** showing LiDAR data
3. **TF frames** moving as the car navigates
4. **Map updates** showing walls and obstacles

### Step 5.3: Monitor SLAM Performance

Open **Terminal 4** - Check topics:
```bash
ros2 topic list
```

**Expected Output (new SLAM topics):**
```
/map
/map_metadata
/pose
/slam_toolbox/feedback
/slam_toolbox/graph_visualization
/slam_toolbox/scan_visualization
/slam_toolbox/update_map
```

### Step 5.4: Understanding What's Happening

As the bag plays:
1. **SLAM receives** `/scan` (LiDAR data) and `/ego_racecar/odom` (odometry)
2. **Processes scans** to match them with previous scans
3. **Estimates robot pose** by combining odometry and scan matching
4. **Builds occupancy grid map** showing free space (white), obstacles (black), and unknown (gray)
5. **Publishes transform** from `map` to `ego_racecar/base_link`
6. **Detects loop closures** when returning to previously visited areas (optimizes map)

### Step 5.5: Save the Map

Once the bag finishes playing:

**In Terminal 2 (where SLAM is running), press Ctrl+C to stop**

**Or use the map server to save manually:**

Open **Terminal 5**:
```bash
cd ~/slam_lab/maps
ros2 run nav2_map_server map_saver_cli -f racetrack_async_map
```

**Expected Output:**
```
[INFO] [map_saver]: Saving map to 'racetrack_async_map.pgm' and 'racetrack_async_map.yaml'
[INFO] [map_saver]: Map saved
```

**Two files created:**
- `racetrack_async_map.pgm` - The actual map image
- `racetrack_async_map.yaml` - Map metadata (resolution, origin, etc.)

**View the map:**
```bash
eog racetrack_async_map.pgm
```

or

```bash
gimp racetrack_async_map.pgm
```

---

## Part 6: Running SLAM with Sync Mode

### Step 6.1: Clear Previous Session

Stop all running nodes (Ctrl+C in all terminals).

### Step 6.2: Launch Sync SLAM

**Terminal 1** - RViz (same as before):
```bash
cd ~/slam_lab
rviz2 -d config/slam_visualization.rviz
```

**Terminal 2** - Launch sync slam_toolbox:
```bash
cd ~/slam_lab
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=config/online_sync_racecar.yaml
```

**Terminal 3** - Play rosbag (maybe slower to match sync processing):
```bash
cd ~/slam_lab/bags
ros2 bag play racecar_lap --rate 0.5
```

**Note the `--rate 0.5`:** This plays the bag at half speed, giving sync SLAM more time to process each scan.

### Step 6.3: Compare with Async Mode

**Observations:**
- **Sync mode** may produce more accurate maps but runs slower
- **Map updates** happen more deliberately
- **Processing** is more thorough for each scan

### Step 6.4: Save Sync Map

```bash
cd ~/slam_lab/maps
ros2 run nav2_map_server map_saver_cli -f racetrack_sync_map
```

### Step 6.5: Compare the Two Maps

```bash
cd ~/slam_lab/maps
eog racetrack_async_map.pgm racetrack_sync_map.pgm
```

**Discussion Points:**
- Which map looks more accurate?
- Which has better loop closure?
- Which has cleaner walls?
- Trade-offs between speed and accuracy

---

## Part 7: Using slam_toolbox Services and Features

slam_toolbox provides several services for runtime interaction.

### Step 7.1: List Available Services

With SLAM running:
```bash
ros2 service list | grep slam_toolbox
```

**Expected Output:**
```
/slam_toolbox/clear_changes
/slam_toolbox/clear_queue
/slam_toolbox/deserialize_map
/slam_toolbox/dynamic_map
/slam_toolbox/manual_loop_closure
/slam_toolbox/pause_new_measurements
/slam_toolbox/save_map
/slam_toolbox/serialize_map
/slam_toolbox/toggle_interactive_mode
```

### Step 7.2: Save Map via Service

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '<absolute path>/slam_lab/maps/service_saved_map'}}"
```

---

## Part 8: Localization Mode with Saved Map

Now let's use the map we created to localize the robot (instead of building a new map).

### Step 8.1: Prepare Localization

Stop all SLAM nodes.

###Step 8.2: Update Localization Config

Edit `~/slam_lab/config/localization_racecar.yaml` and update the map file path:

```yaml
slam_toolbox:
  ros__parameters:
    # ... other parameters ...
    map_file_name: <absolute path>/slam_lab/maps/slam_graph
    # ... rest of config ...
```

> Use the serialized map we saved earlier

### Step 8.3: Launch Localization Mode

**Terminal 1** - RViz:
```bash
cd ~/slam_lab
rviz2 -d config/slam_visualization.rviz
```

**Terminal 2** - Localization slam_toolbox:
```bash
cd ~/slam_lab
ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=config/localization_racecar.yaml
```

**Terminal 3** - Play rosbag:
```bash
cd ~/slam_lab/bags
ros2 bag play racecar_lap
```

**What to Observe:**
- Map loads immediately (not built from scratch)
- Robot localizes itself on the existing map
- No map updates (map is fixed)
- Laser scans should align with map walls

---

## Part 9: Parameter Tuning and Experimentation

### Experiment 1: Resolution

Create a high-resolution config:

**File:** `~/slam_lab/config/high_res.yaml`

```yaml
# Copy from online_async_racecar.yaml but change:
resolution: 0.02  # 2cm instead of 5cm
```

**Run with high resolution:**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=config/high_res.yaml
```

**Observations:**
- More detailed map
- Larger file size
- More processing required

### Experiment 2: Loop Closure Settings

**Disable loop closure:**

Edit config file:
```yaml
do_loop_closing: false
```

**Run and observe:**
- Map may have drift when completing the lap
- Start and end positions may not align perfectly

**Enable aggressive loop closure:**
```yaml
do_loop_closing: true
loop_match_minimum_response_fine: 0.3  # Lower threshold
```

**Observe:**
- Better loop closure
- Start/end positions align better

### Experiment 3: Scan Throttling

For faster processing with less accuracy:

```yaml
throttle_scans: 3  # Process only every 3rd scan
```

**Observations:**
- Faster processing
- May miss details
- Good for initial rapid mapping

### Experiment 4: Maximum Laser Range

Try different ranges:

```yaml
max_laser_range: 10.0  # Short range
```

vs

```yaml
max_laser_range: 30.0  # Long range
```

**Observations:**
- Short range: More local features, less far-field noise
- Long range: Better overall structure, more noise

---

## Part 10: Advanced Features and RViz Plugins

### Step 10.1: slam_toolbox RViz Plugin

slam_toolbox includes an interactive RViz plugin for runtime control.

**In RViz:**
1. Click **Panels → Add New Panel**
2. Select **slam_toolbox → SlamToolboxPlugin**
3. A new panel appears

**Plugin features:**
- **Save Map** button
- **Clear Changes** button
- **Serialize Map** button
- **Interactive Mode** toggle
- **Continue/Pause** controls

**Try these features while SLAM is running!**

### Step 10.2: View Pose Graph

Add graph visualization in RViz:
1. Click **Add**
2. Select **MarkerArray**
3. Set topic to `/slam_toolbox/graph_visualization`

**You'll see:**
- Nodes representing robot poses
- Edges showing scan matches
- Loop closure constraints

---

## Part 11: Analyzing SLAM Performance

### Metrics to Evaluate

1. **Map Quality:**
   - Clear wall boundaries
   - Minimal noise
   - Proper loop closure (start/end align)

2. **Computational Performance:**
   - Processing speed
   - CPU usage
   - Memory consumption

3. **Localization Accuracy:**
   - Laser scans align with map
   - Consistent pose estimates

### Step 11.1: Check CPU Usage

While SLAM is running:
```bash
top | grep slam
```

or

```bash
htop
```

### Step 11.2: Check Topic Rates

```bash
ros2 topic hz /map  # Might not always work due to QoS mismatch
ros2 topic hz /pose
ros2 topic hz /scan
```

### Step 11.3: Inspect Map File

```bash
cat ~/slam_lab/maps/racetrack_async_map.yaml
```

**Expected content:**
```yaml
image: racetrack_async_map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**Key fields:**
- `resolution`: Meters per pixel
- `origin`: Map origin in world coordinates
- `occupied_thresh`: Threshold for considering cell occupied

---

## Lab Summary

### What We Learned:

1. **SLAM Fundamentals:**
   - Simultaneous Localization and Mapping concepts
   - Importance for autonomous systems
   - Sensor fusion (LiDAR + Odometry)

2. **slam_toolbox Modes:**
   - **Async:** Fast, continuous mapping
   - **Sync:** Accurate, deliberate processing
   - **Localization:** Use existing maps

3. **Configuration:**
   - Critical parameters (resolution, range, loop closure)
   - Trade-offs between accuracy and speed
   - Tuning for different scenarios

4. **Practical Skills:**
   - Creating and saving maps
   - Serializing pose graphs
   - Using slam_toolbox services
   - RViz visualization and plugins

5. **Performance Analysis:**
   - Evaluating map quality
   - Monitoring computational load
   - Understanding parameter effects

### Key Takeaways for Autonomous Racing:

- **Pre-mapping:** Create track maps offline before racing
- **Localization:** Use pre-built maps for precise localization during races
- **Parameter tuning:** Balance accuracy vs. real-time performance
- **Loop closure:** Critical for closed-circuit tracks
- **Resolution:** Higher resolution for precise racing lines

---

# Deliverable

## SLAM Robustness Analysis and Experimentation

### Objective
Investigate how sensor noise and errors affect SLAM performance by systematically introducing different types of errors to the LiDAR scan data. You will analyze how slam_toolbox responds to various error conditions and document your findings.

---

## Assignment Overview

You will:
1. Create a ROS2 node that introduces errors to LiDAR scan data
2. Run slam_toolbox with the modified data
3. Generate maps under different error conditions
4. Compare and analyze the results
5. Document your observations

---

## Detailed Requirements

Using the provided rosbag, you will:

1. Run a **baseline** SLAM mapping pass (no modifications to the bag replay) and save that map
2. Run at least **one “corrupted input”** SLAM pass by injecting error into a main SLAM-related topic, for example: `/scan` (easiest).
3. Save that map as well. 
4. Compare results and write a short observation of how the error impacted mapping and localization.

You must run **at least two runs**:

### 1) Baseline Run (No Errors)

* Replay rosbag normally
* Run slam_toolbox
* Save and display map

### 2) Corrupted Run (With Errors)

* Replay rosbag
* Apply one error type to a SLAM-relevant topic
* Run slam_toolbox again
* Save and display new map

You may do more corrupted runs for extra insight, but **one corrupted run is required**.

---

## Some Examples of Approache

### Remap `/scan` from bag → corrupt → republish to `/scan`

1. Play bag while remapping:
ex:
   ```bash
   ros2 bag play racecar_lap --clock --remap /scan:=/scan_original
   ```
2. Run a node that:

   * subscribes to `/scan_original`
   * modifies the LaserScan data (inject error)
   * publishes the modified scan to `/scan` (the topic slam_toolbox expects)

### Change slam_toolbox to listen to your modified scan topic

* publish corrupted scans to something like `/scan_corrupted`
* configure slam_toolbox parameter(s) to subscribe to that topic

---

## Error Injection Ideas

### A. Add Noise to Ranges (Gaussian or uniform)

**Effect to watch for:** fuzzier walls, map “thickening,” loop closure instability.

### B. Random Dropout (set some ranges to `inf` / `0.0`)

Example: drop 20–40% of beams.
**Effect:** missing wall segments, map holes, lost tracking.

### C. Range Scaling Bias

Multiply ranges by 1.05–1.20 (systematic error).
**Effect:** map distortion (track becomes too big/small), misalignment over time.

### D. Angle/Index Scramble

Shuffle a portion of beams or reverse the array.
**Effect:** SLAM may quickly fail; map becomes nonsense.

### E. Lower Effective Update Rate

Publish only every Nth message.
**Effect:** pose tracking worse, map drift.

**Recommendation:** Start with **dropout** or **noise**, because they show clear degradation but still “kinda works.”

---

## Examples of What to Observe and Report

Compare baseline vs corrupted:

* Does the map still look like a track?
* Are walls straight or smeared?
* Does the map drift or rotate over time?
* Does slam_toolbox lose tracking / diverge?
* Any loop-closure behavior differences (if visible)?
* Any notable console warnings/errors?

---

## **Submission Instructions**

1. **Submit only the `src` folder** from your ROS 2 workspace.

   * All of your work **must be contained inside this `src` folder**.
   * Do **not** include `build/`, `install/`, or `log/` directories.

2. **Make sure all required packages, nodes, launch files, and config files** needed to run your assignment are inside the `src` folder.

3. **Include a `README.md` file inside the `src` folder** that clearly explains:

   * What you implemented for the assignment
   * How to build the workspace
   * How to run your code (exact commands)
   * Any report or observation if applicable

4. **Zip the `src` folder only** (not the full workspace).

5. **Name the ZIP file exactly as follows:**

   ```
   first-name_last-name_studentID_lab#.zip
   ```

   Example:

   ```
   jane_doe_1234567_lab2.zip
   ```

---

## **Grading Rubric (100 Points)**

| Category                           | Points |
| ---------------------------------- | ------ |
| Fully functioning launch file | 15     |
| Proper `rosbag` usage      | 15     |
| Node implemented properly    | 20     |
| Error Introduced Properly      | 20     |
| Insightful Analysis     | 20     |
| Well organized submission, well formatted codes          | 10     |
