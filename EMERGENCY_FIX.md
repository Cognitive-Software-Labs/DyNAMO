# EMERGENCY FIX GUIDE - Nothing Works

## Current Situation
Based on your screenshot, the system has **critical TF frame errors**:
- "frame 'odom' does not exist"
- "frame does not exist" (repeated)
- RViz is empty
- Image view shows nothing
- Gazebo may not be showing robot

## Root Cause
The robot is **NOT spawning in Gazebo**, which means:
- No TF frames are published
- No camera data
- No odometry
- Nothing for RViz to display

---

## SOLUTION 1: Use Simplified Launch (RECOMMENDED)

### Step 1: Stop Everything
```bash
# Press Ctrl+C in launch terminal
# Then kill all:
killall -9 ros2 gz python3 ruby
rm -rf /dev/shm/fastrtps_*
```

### Step 2: Rebuild
```bash
cd ~/vision_ws
colcon build --packages-select ridgeback_vision_detection --symlink-install
source install/setup.bash
```

### Step 3: Launch Simplified Version
```bash
ros2 launch ridgeback_vision_detection simple_launch.py
```

This version:
- ✅ No Nav2 (removes TF complexity)
- ✅ No SLAM (removes map errors)
- ✅ Just Gazebo + Robot + Camera + Detector
- ✅ Has delays to ensure proper startup

### Step 4: Wait and Watch
- **0-5 seconds:** Gazebo starts
- **5-10 seconds:** Robot spawns
- **10+ seconds:** Detector starts

### Step 5: View Camera (New Terminal)
```bash
source ~/vision_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /vision_detection/image_result
```

---

## SOLUTION 2: Test Gazebo First

If simplified launch doesn't work, test Gazebo:

```bash
cd ~/vision_ws
./test_minimal.sh
```

This will tell you if Gazebo itself is broken.

---

## SOLUTION 3: Manual Step-by-Step

### Terminal 1: Start Gazebo Only
```bash
cd ~/vision_ws
source install/setup.bash

# Start Gazebo with your world
gz sim install/ridgeback_vision_detection/share/ridgeback_vision_detection/worlds/warehouse.sdf
```

**Check:** Does Gazebo window open? Do you see the warehouse?

### Terminal 2: Start Robot State Publisher
```bash
source ~/vision_ws/install/setup.bash

# Process URDF
xacro src/ridgeback_vision_detection/urdf/ridgeback.urdf.xacro > /tmp/robot.urdf

# Start publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat /tmp/robot.urdf)" -p use_sim_time:=true
```

### Terminal 3: Spawn Robot
```bash
source ~/vision_ws/install/setup.bash

# Wait 5 seconds after Gazebo starts, then:
ros2 run ros_gz_sim create \
  -name ridgeback \
  -file /tmp/robot.urdf \
  -x -8.0 -y 0.0 -z 0.2
```

**Check:** Do you see the robot in Gazebo?

### Terminal 4: Start Bridge
```bash
source ~/vision_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -p use_sim_time:=true
```

### Terminal 5: Check Topics
```bash
source ~/vision_ws/install/setup.bash

# List topics
ros2 topic list

# Check camera
ros2 topic hz /camera/image_raw

# Check odom
ros2 topic echo /odom --once
```

### Terminal 6: Start Detector
```bash
source ~/vision_ws/install/setup.bash

ros2 run ridgeback_vision_detection detector \
  --ros-args -p use_sim_time:=true
```

### Terminal 7: View Camera
```bash
source ~/vision_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view /vision_detection/image_result
```

---

## Diagnostic Commands

### Check if Gazebo is Running
```bash
ps aux | grep "gz sim"
```

### Check Gazebo Models
```bash
gz model --list
```

### Check ROS2 Nodes
```bash
ros2 node list
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing TF tree
```

### Check Topics
```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /odom
```

---

## Common Problems & Fixes

### Problem: "gz: command not found"
**Fix:**
```bash
# Check Gazebo installation
which gz
gz sim --version

# If not found, install:
sudo apt install gz-harmonic
```

### Problem: "Package 'ros_gz_sim' not found"
**Fix:**
```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

### Problem: Gazebo opens but robot doesn't spawn
**Fix:**
```bash
# Check URDF is valid
xacro src/ridgeback_vision_detection/urdf/ridgeback.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf
```

### Problem: "frame 'odom' does not exist"
**Cause:** Robot not spawned or diff_drive plugin not working
**Fix:** Ensure robot spawns successfully in Gazebo

### Problem: Camera shows nothing
**Fix:**
```bash
# Check Gazebo camera topic
gz topic -l | grep camera
gz topic -e -t /camera/image_raw -n 1

# Check ROS2 camera topic
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

---

## What Should Work

### Minimal Working System:
1. Gazebo opens with warehouse
2. Robot appears in Gazebo
3. `ros2 topic list` shows:
   - /camera/image_raw
   - /odom
   - /cmd_vel
   - /scan
4. `ros2 node list` shows:
   - /robot_state_publisher
   - /ros_gz_bridge
   - /vision_detector
5. Camera view shows warehouse from robot perspective
6. Robot rotates after 6 seconds

---

## Emergency Contact

If NOTHING works:

1. **Check ROS2 version:**
   ```bash
   echo $ROS_DISTRO
   # Should be: jazzy
   ```

2. **Check Gazebo version:**
   ```bash
   gz sim --version
   # Should be: Harmonic (8.x)
   ```

3. **Reinstall dependencies:**
   ```bash
   sudo apt update
   sudo apt install --reinstall \
     ros-jazzy-ros-gz-sim \
     ros-jazzy-ros-gz-bridge \
     ros-jazzy-robot-state-publisher \
     gz-harmonic
   ```

4. **Check system resources:**
   ```bash
   free -h  # Need at least 2GB RAM free
   df -h    # Need disk space
   ```

---

## Quick Decision Tree

```
Can you run: gz sim empty.sdf
├─ YES → Gazebo works, problem is with robot/ROS2
│   └─ Use SOLUTION 1 (Simplified Launch)
│
└─ NO → Gazebo is broken
    └─ Reinstall Gazebo: sudo apt install --reinstall gz-harmonic
```

---

**START HERE:**
```bash
cd ~/vision_ws
killall -9 ros2 gz python3 ruby
colcon build --packages-select ridgeback_vision_detection
source install/setup.bash
ros2 launch ridgeback_vision_detection simple_launch.py
```

Wait 15 seconds, then in new terminal:
```bash
source ~/vision_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /vision_detection/image_result
```

---

**Last Updated:** 2026-02-12 09:48  
**Status:** CRITICAL - SIMPLIFIED LAUNCH CREATED
