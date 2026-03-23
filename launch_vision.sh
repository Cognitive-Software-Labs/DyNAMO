#!/bin/bash
# ──────────────────────────────────────────────────────────────
# Launch script that cleans snap library paths so Gazebo GUI
# and rqt_image_view can start without the libpthread crash.
# ──────────────────────────────────────────────────────────────

# Remove any snap paths from LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')

# Prevent the loader from finding snap's old glibc
unset LOCPATH
export GTK_PATH=""
export GTK_EXE_PREFIX=""
export GIO_MODULE_DIR=""
export GTK_IM_MODULE_FILE=""
export GSETTINGS_SCHEMA_DIR=""

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source /home/pratham/vision_ws/install/setup.bash

echo "=== Launching Gazebo + Vision Navigator ==="
echo "  Snap paths cleaned from environment"
echo "  Gazebo GUI should open!"
echo ""

ros2 launch ridgeback_vision_detection simple_launch.py
