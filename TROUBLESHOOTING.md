# TROUBLESHOOTING GUIDE - Nothing Works

## Current Issues Identified

Based on your screenshots, here are the problems:

### ❌ Problem 1: TF_OLD_DATA Errors
**Symptom:** Terminal shows TF transform errors  
**Cause:** Multiple robot_state_publisher nodes running (duplicates)  
**Impact:** Navigation can't localize properly

### ❌ Problem 2: RViz Shows "Unknown" 
**Symptom:** Navigation shows "unknown" state, empty map  
**Cause:** SLAM not receiving proper data or TF issues  
**Impact:** Can't navigate or visualize robot position

### ❌ Problem 3: Camera Not Publishing to ROS2
**Symptom:** Camera works in Gazebo but not in ROS2  
**Cause:** Bridge configuration mismatch  
**Impact:** Vision detection can't receive images

---

## SOLUTION: Complete System Restart

### Step 1: Stop Everything
```bash
# Press Ctrl+C in the terminal running the launch file

# Then run this to kill everything:
cd ~/vision_ws
./restart_system.sh
```

### Step 2: Verify the Fix
```bash
cd ~/vision_ws
source install/setup.bash

# Check the camera URDF was updated
cat src/ridgeback_vision_detection/urdf/camera_extras.urdf.xacro | grep topic
# Should show: <topic>camera</topic>
```

### Step 3: Launch Fresh
```bash
cd ~/vision_ws
source install/setup.bash
ros2 launch ridgeback_vision_detection ridgeback_detection_launch.py
```

### Step 4: Diagnose (In Another Terminal)
```bash
cd ~/vision_ws
./diagnose.sh
```

---

## What Was Fixed

### 1. Camera Topic Configuration
**Before:**
```xml
<topic>/camera/image_raw</topic>
```

**After:**
```xml
<topic>camera</topic>
<ignition_frame_id>camera_extra_link</ignition_frame_id>
```

**Why:** Gazebo Harmonic/Jazzy uses relative topic names, not absolute paths.

### 2. Added Frame ID
The `ignition_frame_id` ensures proper TF frame association.

---

## Expected Behavior After Fix

### ✅ In Gazebo:
- Robot spawns at (-8, 0, 0.1)
- Red sphere visible at (8, 0, 0.3)
- Robot should start rotating after 6 seconds

### ✅ In RViz:
- Robot model visible
- Map building in real-time
- Localization shows position (not "unknown")
- No TF errors

### ✅ In Terminal:
- No TF_OLD_DATA errors
- Camera publishing at ~10Hz
- Detector node processing images

---

## Verification Commands

### Check Camera (After Launch)
```bash
# In new terminal:
source ~/vision_ws/install/setup.bash

# Check if camera topic exists
ros2 topic list | grep camera

# Check if camera is publishing
ros2 topic hz /camera/image_raw

# View one message
ros2 topic echo /camera/image_raw --once | head -20
```

### Check Detector
```bash
# Check if detector is running
ros2 node list | grep vision_detector

# Check detector output
ros2 topic hz /vision_detection/image_result
```

### View Camera Feed
```bash
# Annotated feed
ros2 run rqt_image_view rqt_image_view /vision_detection/image_result

# Raw feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---

## If Still Not Working

### Option 1: Clean Everything
```bash
cd ~/vision_ws

# Kill all processes
killall -9 ros2 gz python3 ruby

# Clean shared memory
sudo rm -rf /dev/shm/fastrtps_*
sudo rm -rf /dev/shm/sem.*

# Clean build
rm -rf build/ install/ log/

# Rebuild
colcon build --packages-select ridgeback_vision_detection --symlink-install

# Source
source install/setup.bash

# Launch
ros2 launch ridgeback_vision_detection ridgeback_detection_launch.py
```

### Option 2: Check Gazebo Version
```bash
gz sim --version
```

Should show Gazebo Harmonic (version 8.x)

### Option 3: Manual Bridge Test
```bash
# Terminal 1: Launch just Gazebo
gz sim ~/vision_ws/install/ridgeback_vision_detection/share/ridgeback_vision_detection/worlds/warehouse.sdf

# Terminal 2: Spawn robot manually
source ~/vision_ws/install/setup.bash
ros2 run ros_gz_sim create -name ridgeback -topic robot_description -x -8 -y 0 -z 0.1

# Terminal 3: Test bridge
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image

# Terminal 4: Check
ros2 topic echo /camera
```

---

## Common Errors Explained

### "TF_OLD_DATA"
- **Meaning:** Transform data is too old
- **Cause:** Time synchronization issues or duplicate publishers
- **Fix:** Ensure only one robot_state_publisher running

### "Message Filter Dropping Message"
- **Meaning:** Messages arriving out of order or too late
- **Cause:** use_sim_time mismatch
- **Fix:** Ensure all nodes have use_sim_time: True

### "Topic does not appear to be published"
- **Meaning:** No data on topic
- **Cause:** Sensor not initialized or bridge not running
- **Fix:** Check Gazebo sensor and bridge configuration

---

## Debug Mode Launch

For more verbose output:
```bash
ros2 launch ridgeback_vision_detection ridgeback_detection_launch.py --ros-args --log-level debug
```

---

## Contact/Support

If none of this works:
1. Run `./diagnose.sh` and save output
2. Check `~/.ros/log/` for error logs
3. Verify ROS2 Jazzy is properly installed
4. Check Gazebo Harmonic compatibility

---

**Last Updated:** 2026-02-12  
**Status:** FIXES APPLIED - RESTART REQUIRED
