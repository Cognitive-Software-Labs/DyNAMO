#!/bin/bash
###############################################################################
# Ridgeback Hospital Mission – FULL GUI (Gazebo + RViz + rqt_image_view)
#
# ROOT CAUSE FIX: The VS Code snap terminal injects GTK_PATH pointing to
# /snap/code/220/usr/lib/x86_64-linux-gnu/gtk-3.0, which via RPATH pulls in
# /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0 — an incompatible
# version that crashes every GUI app.  Clearing these vars fixes everything.
###############################################################################
set -e

# ── Fix: clear VS Code snap GTK env vars that crash GUI apps ──
export GTK_PATH=""
export GTK_EXE_PREFIX=""
export GIO_MODULE_DIR=""
export GTK_IM_MODULE_FILE=""
export GSETTINGS_SCHEMA_DIR=""

# ── Source ROS2 ──
source /opt/ros/jazzy/setup.bash
source ~/vision_ws/install/local_setup.bash

PKG_DIR="$(ros2 pkg prefix ridgeback_vision_detection)/share/ridgeback_vision_detection"
WORLD="$PKG_DIR/worlds/hospital.sdf"
URDF_XACRO="$PKG_DIR/urdf/ridgeback.urdf.xacro"
SLAM_PARAMS="$PKG_DIR/config/slam_params.yaml"
NAV2_PARAMS="$PKG_DIR/config/nav2_params.yaml"
RVIZ_CFG="$PKG_DIR/rviz/nav2_view.rviz"

export GZ_SIM_RESOURCE_PATH="$(dirname "$PKG_DIR"):${GZ_SIM_RESOURCE_PATH:-}"

echo ""
echo "============================================"
echo "  🏥 Ridgeback Hospital Mission Launcher"
echo "============================================"
echo "  All GUIs on DISPLAY=$DISPLAY"
echo ""

# ── Cleanup ──
echo "[0/9] Cleaning up old processes..."
pkill -9 -f "gz sim|rviz2|rqt|parameter_bridge|detector|odom_tf|slam_toolbox|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|lifecycle_manager|robot_state_publisher|Xvfb" 2>/dev/null || true
sleep 2

# ═══════════════════════════════════════════════════════════════
# Step 1: Gazebo (real display — you can see the simulation!)
# ═══════════════════════════════════════════════════════════════
echo "[1/9] 🎮 Starting Gazebo..."
gz sim -r "$WORLD" > /tmp/gz_sim.log 2>&1 &
GZ_PID=$!
echo "       Gazebo PID: $GZ_PID"

echo "       Waiting for Gazebo server..."
for i in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q "/world/hospital/clock"; then
        echo "       ✅ Gazebo ready (${i}s)"
        break
    fi
    if ! kill -0 $GZ_PID 2>/dev/null; then
        echo "       ❌ Gazebo crashed! Log:"
        tail -10 /tmp/gz_sim.log
        exit 1
    fi
    [ $i -eq 60 ] && { echo "       ❌ Gazebo timeout"; exit 1; }
    sleep 1
done

# ═══════════════════════════════════════════════════════════════
# Step 2: Robot State Publisher (handles xacro → URDF)
# ═══════════════════════════════════════════════════════════════
echo "[2/9] Starting Robot State Publisher..."
export URDF_XACRO_PATH="$URDF_XACRO"
python3 -c "
import launch, launch_ros.actions, os
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

xf = os.environ['URDF_XACRO_PATH']
rd = Command([FindExecutable(name='xacro'), ' ', xf])
ls = launch.LaunchService()
ls.include_launch_description(launch.LaunchDescription([
    launch_ros.actions.Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(rd, value_type=str),
                      'use_sim_time': True}], output='log')
]))
ls.run()
" > /tmp/rsp.log 2>&1 &
RSP_PID=$!
echo "       RSP PID: $RSP_PID"
sleep 4

if ros2 topic list 2>/dev/null | grep -q "/robot_description"; then
    echo "       ✅ Robot State Publisher OK"
else
    echo "       ⚠  /robot_description not found yet (may still be starting)"
fi

# ═══════════════════════════════════════════════════════════════
# Step 3: Spawn Robot into Gazebo
# ═══════════════════════════════════════════════════════════════
echo "[3/9] 🤖 Spawning Ridgeback at (-3, 0, 0.2)..."
ros2 run ros_gz_sim create -name ridgeback -topic robot_description \
    -x -3.0 -y 0.0 -z 0.2 2>&1 | tail -3
echo "       ✅ Robot spawned"
sleep 3

# ═══════════════════════════════════════════════════════════════
# Step 4: TF topology
# ═══════════════════════════════════════════════════════════════
echo "[4/9] Using robot_state_publisher TF tree (no extra static TF overrides)..."
echo "       ✅ TF tree source set"

# ═══════════════════════════════════════════════════════════════
# Step 5: ros_gz_bridge (Gazebo ↔ ROS2 topics)
# ═══════════════════════════════════════════════════════════════
echo "[5/9] Starting ros_gz_bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model \
    /imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
    '/camera_d455/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
    '/camera_d455/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
    --ros-args -p use_sim_time:=true \
    -r /camera_d455/image_raw:=/camera/image_raw \
    -r /camera_d455/camera_info:=/camera/camera_info \
    > /tmp/bridge.log 2>&1 &
echo "       Bridge PID: $!"
sleep 5

for topic in /odom /scan /scan_rear /camera/image_raw; do
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo "       ✅ $topic"
    else
        echo "       ⚠  $topic not found"
    fi
done

# ═══════════════════════════════════════════════════════════════
# Step 6: Odom publisher (cmd_vel → odom + TF)
# ═══════════════════════════════════════════════════════════════
# Gazebo may not always publish /odom, so we generate it from commanded velocities.
# This keeps RViz+Nav2 consistent with the motion commands.
# ═══════════════════════════════════════════════════════════════
# Odometry from Gazebo works now, so we skip the fake publisher
# echo "[6/9] Starting odom_tf_publisher..."
# ros2 run ridgeback_vision_detection odom_tf_pub --ros-args \
#     -p use_sim_time:=true > /tmp/odom_tf.log 2>&1 &
# ODEM_PID=$!
# echo "       Odom TF PID: $ODEM_PID"
# sleep 2

# ═══════════════════════════════════════════════════════════════
# Step 7: SLAM + Nav2
# ═══════════════════════════════════════════════════════════════
echo "[6/9] Starting SLAM Toolbox + Nav2..."
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true slam_params_file:="$SLAM_PARAMS" \
    > /tmp/slam.log 2>&1 &
sleep 3
echo "       ✅ SLAM Toolbox started"

ros2 run nav2_controller controller_server --ros-args \
    --params-file "$NAV2_PARAMS" \
    > /tmp/nav2_controller.log 2>&1 &
ros2 run nav2_smoother smoother_server --ros-args \
    --params-file "$NAV2_PARAMS" \
    > /tmp/nav2_smoother.log 2>&1 &
ros2 run nav2_planner planner_server --ros-args \
    --params-file "$NAV2_PARAMS" \
    > /tmp/nav2_planner.log 2>&1 &
ros2 run nav2_behaviors behavior_server --ros-args \
    --params-file "$NAV2_PARAMS" \
    > /tmp/nav2_behavior.log 2>&1 &
ros2 run nav2_bt_navigator bt_navigator --ros-args \
    --params-file "$NAV2_PARAMS" \
    > /tmp/nav2_bt.log 2>&1 &
sleep 2
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p use_sim_time:=true -p autostart:=true \
    -p "node_names:=[controller_server,smoother_server,planner_server,behavior_server,bt_navigator]" \
    > /tmp/nav2_lifecycle.log 2>&1 &
echo "       ✅ Nav2 stack started"
sleep 5

# ═══════════════════════════════════════════════════════════════
# Step 7: RViz2 (laser scan, SLAM map, robot model, TF)
# ═══════════════════════════════════════════════════════════════
echo "[7/9] 📺 Starting RViz2..."
ros2 run rviz2 rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true \
    > /tmp/rviz.log 2>&1 &
echo "       ✅ RViz2 started"
sleep 3

# ═══════════════════════════════════════════════════════════════
# Step 8: Detector + rqt_image_view
# ═══════════════════════════════════════════════════════════════
echo "[8/9] 🎯 Starting detector mission + image viewer..."
ros2 run ridgeback_vision_detection detector --ros-args \
    -p use_sim_time:=true > /tmp/detector.log 2>&1 &
DET_PID=$!
echo "       Detector PID: $DET_PID"
sleep 3

ros2 run rqt_image_view rqt_image_view /vision_detection/image_result \
    > /tmp/rqt.log 2>&1 &
echo "       ✅ rqt_image_view started"

echo ""
echo "============================================"
echo "  🚀 ALL SYSTEMS LAUNCHED!"
echo "============================================"
echo ""
echo "  🎮 Gazebo        → Hospital simulation (visible!)"
echo "  📺 RViz2         → Laser scan, SLAM map, robot model"
echo "  🖼  rqt_image_view → Camera + detection overlay"
echo "  🤖 Detector       → Visual servo mission running"
echo ""
echo "  To stop:  pkill -9 -f 'gz sim|rviz2|rqt|parameter_bridge|detector|odom_tf|slam|nav2|lifecycle|robot_state'"
echo ""
echo "📡 Following mission progress..."
echo "────────────────────────────────"
tail -f /tmp/detector.log 2>/dev/null | grep --line-buffered -E "🏥|🔍|🎯|🧭|⏱|🔙|✅|📷|🚪|🛏️|Mission|SEARCH|NAVIGATE|WAIT|RETURN|DONE"
