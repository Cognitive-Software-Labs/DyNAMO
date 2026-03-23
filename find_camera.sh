#!/bin/bash

# Find the hidden camera topic
echo "Scanning ALL Gazebo topics..."
gz topic -l | grep camera
echo ""
echo "Checking Ridgeback Links & Sensors:"
gz model -m ridgeback -l
gz model -m ridgeback -s
echo ""

echo ""
echo "Attempting to echo '/camera'..."
timeout 2 gz topic -e -t /camera -n 1

echo ""
echo "Attempting to echo '/camera/image_raw'..."
timeout 2 gz topic -e -t /camera/image_raw -n 1

echo ""
echo "Attempting to echo '/camera/image'..."
timeout 2 gz topic -e -t /camera/image -n 1

echo ""
echo "Check ROS2 topics:"
ros2 topic list | grep camera
