#!/bin/bash

# Ridgeback Vision Detection Launch Script
# This script launches the complete Ridgeback robot simulation with vision detection

echo "========================================="
echo "  Ridgeback Vision Detection System"
echo "========================================="
echo ""
echo "Starting components:"
echo "  ✓ Gazebo Simulation (Warehouse World)"
echo "  ✓ Ridgeback Robot with Camera & LiDAR"
echo "  ✓ ROS-Gazebo Bridge"
echo "  ✓ SLAM Toolbox"
echo "  ✓ Nav2 Navigation Stack"
echo "  ✓ RViz Visualization"
echo "  ✓ Vision Detection Node"
echo ""
echo "========================================="
echo ""

# Source the workspace
source /home/pratham/vision_ws/install/setup.bash

# Launch the complete system
ros2 launch ridgeback_vision_detection ridgeback_detection_launch.py
