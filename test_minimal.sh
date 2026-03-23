#!/bin/bash

# MINIMAL TEST - Just Gazebo and Robot, Nothing Else
# This will help us see what's actually broken

echo "========================================="
echo "  MINIMAL SYSTEM TEST"
echo "========================================="
echo ""

# Kill everything first
echo "Stopping all processes..."
killall -9 ros2 gz python3 ruby 2>/dev/null
rm -rf /dev/shm/fastrtps_* 2>/dev/null
sleep 2

cd /home/pratham/vision_ws
source install/setup.bash

echo ""
echo "Step 1: Testing Gazebo alone..."
echo "-------------------------------"
echo "Starting Gazebo with empty world..."
echo "(This should open Gazebo window)"
echo ""

# Start Gazebo in background
gz sim empty.sdf &
GZ_PID=$!

sleep 5

# Check if Gazebo is running
if ps -p $GZ_PID > /dev/null; then
    echo "✓ Gazebo is running (PID: $GZ_PID)"
else
    echo "✗ Gazebo failed to start"
    exit 1
fi

echo ""
echo "Step 2: Testing Gazebo service..."
echo "----------------------------------"
gz service -l | head -10

echo ""
echo "Step 3: Killing Gazebo..."
kill $GZ_PID 2>/dev/null
sleep 2

echo ""
echo "========================================="
echo "  TEST COMPLETE"
echo "========================================="
echo ""
echo "If Gazebo opened, the problem is with the"
echo "robot spawning or ROS2 integration."
echo ""
echo "If Gazebo didn't open, you have a Gazebo"
echo "installation problem."
echo ""
