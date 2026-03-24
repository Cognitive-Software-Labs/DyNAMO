#!/bin/bash
set -e

# ── FIX: Curățăm "veninul" injectat de VS Code Snap ──
export GTK_PATH=""
export GTK_EXE_PREFIX=""
export GIO_MODULE_DIR=""
unset SNAP SNAP_USER_DATA SNAP_COMMON LD_LIBRARY_PATH
export XDG_DATA_DIRS="/usr/local/share:/usr/share"

# ── Source ROS2 & Workspace ──
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Configurații Căi
PKG_NAME="ridgeback_frontier_exploration"
WORLD=$(ros2 pkg prefix $PKG_NAME)/share/$PKG_NAME/worlds/hospital.sdf
URDF_XACRO=$(ros2 pkg prefix $PKG_NAME)/share/$PKG_NAME/urdf/ridgeback.urdf.xacro
NAV2_PARAMS=$(ros2 pkg prefix $PKG_NAME)/share/$PKG_NAME/config/nav2_params.yaml
SLAM_PARAMS=$(ros2 pkg prefix $PKG_NAME)/share/$PKG_NAME/config/slam_params.yaml
RVIZ_CFG=$(ros2 pkg prefix $PKG_NAME)/share/$PKG_NAME/rviz/nav2_view.rviz

# ── Cleanup ──
echo "Curățăm procesele vechi..."
pkill -9 -f "gz sim|rviz2|parameter_bridge|odom_tf|slam_toolbox|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|lifecycle_manager_navigation|robot_state_publisher|explore" || true
sleep 2

# 1. Gazebo
echo "Pornim Gazebo..."
gz sim -r "$WORLD" > /dev/null 2>&1 &
sleep 5

## 2. Robot State Publisher (Îmbunătățit)
echo "Pornim Robot State Publisher..."
# Generăm URDF-ul într-un fișier temporar pentru a fi siguri că e gata înainte de spawn
xacro "$URDF_XACRO" > /tmp/ridgeback.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/ridgeback.urdf --ros-args -p use_sim_time:=true > /tmp/rsp.log 2>&1 &
sleep 5 # Îi dăm 5 secunde pline să propage topicul

# 3. Spawn Robot (Adăugăm un mic check)
echo "Spawn Ridgeback..."
ros2 run ros_gz_sim create -name ridgeback -file /tmp/ridgeback.urdf -x 0.0 -y 0.0 -z 0.2
sleep 3

# 4. Bridge (ROS <-> Gazebo)
echo "Pornim Bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
    /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    --ros-args -p use_sim_time:=true &
sleep 3

# 5. TF Fix & SLAM
echo "Pornim TF Fix și SLAM..."
ros2 run $PKG_NAME odom_tf_pub --ros-args -p use_sim_time:=true &
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:="$SLAM_PARAMS" > /dev/null 2>&1 &
sleep 4

# 6. Nav2 Stack
echo "[6/9] Pornim Nav2 (Modul Debug activat)..."
ros2 run nav2_controller controller_server --ros-args --params-file "$NAV2_PARAMS" &
ros2 run nav2_planner planner_server --ros-args --params-file "$NAV2_PARAMS" &
ros2 run nav2_behaviors behavior_server --ros-args --params-file "$NAV2_PARAMS" &
ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file "$NAV2_PARAMS" &
sleep 2

ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -r __node:=lifecycle_manager_navigation \
    --params-file "$NAV2_PARAMS" &
sleep 8

# 7. RViz
echo "Pornim RViz..."
ros2 run rviz2 rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true &

# 8. Explore Lite (Corectat)
echo "[8/9] Pornim EXPLORAREA..."

# Așteptăm să apară global costmap-ul (care are formatul corect OccupancyGrid)
costmap_wait=10
while [ $costmap_wait -gt 0 ]; do
  if ros2 topic list | grep -qx "/global_costmap/costmap"; then
    break
  fi
  sleep 1
  costmap_wait=$((costmap_wait - 1))
done

if [ $costmap_wait -le 0 ]; then
  echo "WARNING: /global_costmap/costmap nu a fost găsit. Posibil ca robotul să nu se miște."
else
  echo "INFO: /global_costmap/costmap găsit! Preluăm datele..."
fi

# Calea către fișierul tău corectat explore.yaml
EXPLORE_PARAMS=$(ros2 pkg prefix $PKG_NAME)/share/$PKG_NAME/config/explore.yaml

# Pornim explore_lite citind STRICT din fișierul YAML, fără să mai suprascriem nimic din CLI!
ros2 run explore_lite explore --ros-args \
    --params-file "$EXPLORE_PARAMS" \
    -r /move_base:=/navigate_to_pose &

echo "SISTEM PORNIT! Verifică dacă robotul începe să se miște."
wait