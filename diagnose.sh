#!/bin/bash

# Diagnostic Script - Run this while system is running
# to see what's actually happening

echo "========================================="
echo "  SYSTEM DIAGNOSTICS"
echo "========================================="
echo ""

source /home/pratham/vision_ws/install/setup.bash

echo "1. RUNNING NODES:"
echo "----------------"
ros2 node list 2>/dev/null | head -20
echo ""

echo "2. CAMERA TOPICS (ROS2):"
echo "------------------------"
ros2 topic list 2>/dev/null | grep camera
echo ""

echo "3. CAMERA TOPICS (Gazebo):"
echo "--------------------------"
gz topic -l 2>/dev/null | grep camera
echo ""

echo "4. CAMERA INFO (ROS2):"
echo "----------------------"
ros2 topic info /camera/image_raw 2>/dev/null
echo ""

echo "5. BRIDGE NODE INFO:"
echo "--------------------"
ros2 node info /ros_gz_bridge 2>/dev/null | grep -A 20 "Subscribers:"
echo ""

echo "6. CHECK IF CAMERA PUBLISHING (Gazebo):"
echo "---------------------------------------"
timeout 2 gz topic -e -t /camera/image_raw -n 1 2>/dev/null | head -5
echo ""

echo "7. CHECK IF CAMERA PUBLISHING (ROS2):"
echo "--------------------------------------"
timeout 2 ros2 topic echo /camera/image_raw --once 2>/dev/null | head -5
echo ""

echo "8. DETECTOR NODE STATUS:"
echo "------------------------"
ros2 node info /vision_detector 2>/dev/null | grep -E "Subscribers:|Publishers:"
echo ""

echo "9. TF TREE:"
echo "-----------"
ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10 &
sleep 2
killall tf2_echo 2>/dev/null
echo ""

echo "10. GAZEBO MODELS:"
echo "------------------"
gz model --list 2>/dev/null
echo ""

echo "========================================="
echo "  DIAGNOSTICS COMPLETE"
echo "========================================="
