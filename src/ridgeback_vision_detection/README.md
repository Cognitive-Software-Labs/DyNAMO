# Ridgeback Vision Detection System

## Overview
This package provides a complete autonomous vision-based detection and navigation system for the Ridgeback robot in a simulated warehouse/hospital environment.

## Features
- **Ridgeback Robot Simulation**: Full 4-wheel differential drive robot with realistic physics
- **Vision Detection**: OpenCV-based red object detection using camera feed
- **LiDAR Integration**: 2D laser scanner for obstacle detection and SLAM
- **Autonomous Navigation**: State machine-based navigation with Nav2 integration
- **SLAM Mapping**: Real-time mapping using SLAM Toolbox
- **RViz Visualization**: Complete visualization of robot state, sensors, and navigation

## System Components

### Hardware (Simulated)
- **Ridgeback Mobile Base**: 4-wheel differential drive platform
- **Camera**: 640x480 RGB camera mounted at front (0.6m forward, 1.0m high)
- **LiDAR**: 2D laser scanner with 180° FOV, 12m range, 720 samples

### Software Stack
- **Gazebo Harmonic**: Physics simulation
- **ROS2 Humble**: Robot middleware
- **Nav2**: Navigation framework
- **SLAM Toolbox**: Simultaneous localization and mapping
- **OpenCV**: Computer vision for target detection

## Mission Profile

The robot executes the following autonomous mission:

1. **INIT** (6 seconds): System initialization and sensor warm-up
2. **SCANNING**: Rotates in place searching for red target
3. **APPROACH**: Moves toward detected red object while maintaining visual lock
4. **OPERATING** (8 seconds): Stations at target location
5. **RETURNING**: Navigates back to starting position
6. **SUCCESS**: Mission complete

## Installation

### Prerequisites
```bash
# Ensure you have ROS2 Humble installed
sudo apt update
sudo apt install ros-humble-desktop-full

# Install required packages
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install python3-opencv
sudo apt install ros-humble-cv-bridge
```

### Build the Package
```bash
cd ~/vision_ws
colcon build --packages-select ridgeback_vision_detection --symlink-install
source install/setup.bash
```

## Usage

### Quick Start
```bash
# Use the convenient launch script
cd ~/vision_ws
./launch_ridgeback.sh
```

### Manual Launch
```bash
cd ~/vision_ws
source install/setup.bash
ros2 launch ridgeback_vision_detection ridgeback_detection_launch.py
```

## Topics

### Published Topics
- `/cmd_vel` - Robot velocity commands
- `/vision_detection/image_result` - Annotated camera feed with detection overlay
- `/joint_states` - Robot joint states

### Subscribed Topics
- `/camera/image_raw` - Raw camera images from Gazebo
- `/odom` - Odometry data
- `/scan` - LiDAR scan data

## Configuration

### Robot Spawn Position
Default: `x=-8.0, y=0.0, z=0.1`

Edit in `launch/ridgeback_detection_launch.py`:
```python
spawn_robot = Node(
    ...
    arguments=[
        '-name', 'ridgeback',
        '-topic', 'robot_description',
        '-x', '-8.0',  # Change X position
        '-y', '0.0',   # Change Y position
        '-z', '0.1'    # Change Z position
    ],
    ...
)
```

### Target Detection Parameters
Edit in `ridgeback_vision_detection/detector.py`:

```python
# Color detection thresholds (HSV)
mask = cv2.inRange(hsv, (0, 100, 40), (10, 255, 255))  # Red lower range
mask += cv2.inRange(hsv, (160, 100, 40), (180, 255, 255))  # Red upper range

# Detection persistence (frames)
self.target_persistence = 5

# Approach speed
cmd.linear.x = 0.4

# Target reached threshold (pixels)
if self.tw < 250:  # Target width in pixels
```

### Camera Configuration
Edit in `urdf/camera_extras.urdf.xacro`:

```xml
<!-- Camera position relative to chassis -->
<origin xyz="0.6 0 1.0" rpy="0 0 0"/>

<!-- Camera parameters -->
<horizontal_fov>1.047</horizontal_fov>  <!-- ~60 degrees -->
<width>640</width>
<height>480</height>
<update_rate>10.0</update_rate>
```

### LiDAR Configuration
Edit in `urdf/lidar_extras.urdf.xacro`:

```xml
<!-- LiDAR position relative to chassis -->
<origin xyz="0.35 0.0 0.32" rpy="0 0 0"/>

<!-- LiDAR parameters -->
<samples>720</samples>
<min_angle>-1.5708</min_angle>  <!-- -90 degrees -->
<max_angle>1.5708</max_angle>   <!-- +90 degrees -->
<min>0.12</min>  <!-- Min range in meters -->
<max>12.0</max>  <!-- Max range in meters -->
```

## Troubleshooting

### Camera Not Working
1. Check if camera bridge is active:
   ```bash
   ros2 topic list | grep camera
   ros2 topic echo /camera/image_raw --no-arr
   ```

2. Verify Gazebo sensor plugin is loaded:
   ```bash
   gz topic -l | grep camera
   ```

### Robot Not Moving
1. Check velocity commands:
   ```bash
   ros2 topic echo /cmd_vel
   ```

2. Verify odometry is publishing:
   ```bash
   ros2 topic echo /odom
   ```

3. Check Gazebo physics:
   - Ensure simulation is not paused
   - Verify wheel friction parameters in `urdf/ridgeback.gazebo`

### SLAM/Nav2 Issues
1. Ensure `use_sim_time` is set correctly:
   ```bash
   ros2 param get /slam_toolbox use_sim_time
   ```

2. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Build Errors
```bash
# Clean build
cd ~/vision_ws
rm -rf build/ install/ log/
colcon build --packages-select ridgeback_vision_detection --symlink-install
```

## Monitoring

### View Camera Feed with Detection Overlay
```bash
ros2 run rqt_image_view rqt_image_view /vision_detection/image_result
```

### View Raw Camera Feed
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Monitor Robot State
```bash
ros2 topic echo /vision_detection/state
```

### Check Node Status
```bash
ros2 node list
ros2 node info /vision_detector
```

## File Structure
```
ridgeback_vision_detection/
├── launch/
│   ├── ridgeback_detection_launch.py  # Main launch file
│   └── detection_launch.py            # Alternative TurtleBot3 launch
├── ridgeback_vision_detection/
│   ├── detector.py                    # Vision detection node
│   └── test_pub.py                    # Test publisher
├── urdf/
│   ├── ridgeback.urdf.xacro          # Main robot description
│   ├── ridgeback.gazebo              # Gazebo plugins
│   ├── camera_extras.urdf.xacro      # Camera sensor
│   ├── lidar_extras.urdf.xacro       # LiDAR sensor
│   └── accessories.urdf.xacro        # Optional accessories
├── worlds/
│   └── warehouse.sdf                  # Simulation world
├── meshes/                            # Robot 3D models
├── materials/                         # Textures
└── package.xml                        # Package dependencies
```

## Performance Tips

1. **Reduce visualization load**: Close RViz if you only need Gazebo
2. **Adjust physics rate**: Lower `max_step_size` in world file for faster simulation
3. **Disable shadows**: In Gazebo, View → Shadows (uncheck)
4. **Lower camera rate**: Reduce `update_rate` in camera config

## Known Limitations

- SLAM requires robot movement to build map
- Vision detection works best with high-contrast red objects
- Nav2 may require initial pose estimate in some scenarios
- Simulation performance depends on system resources

## Future Enhancements

- [ ] Multi-target detection and prioritization
- [ ] Dynamic obstacle avoidance using LiDAR
- [ ] Path planning integration with Nav2
- [ ] Machine learning-based object recognition
- [ ] Real robot deployment configuration

## License
TODO: License declaration

## Maintainer
Pratham (i.m.pratham01@gmail.com)

## Version History
- v9.0: Persistent targeting with hysteresis
- Current: Complete Ridgeback integration with Nav2 and SLAM
