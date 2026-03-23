#!/bin/bash

# Complete System Restart Script
# This kills all processes and starts fresh

echo "========================================="
echo "  System Restart Script"
echo "========================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Stopping all ROS2 and Gazebo processes...${NC}"

# Kill ROS2 processes
killall -9 ros2 2>/dev/null
killall -9 python3 2>/dev/null

# Kill Gazebo processes
killall -9 gz 2>/dev/null
killall -9 gzserver 2>/dev/null
killall -9 gzclient 2>/dev/null
killall -9 ruby 2>/dev/null

# Clean up shared memory
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -rf /dev/shm/sem.* 2>/dev/null

sleep 2

echo -e "${GREEN}✓ All processes stopped${NC}"
echo ""

echo -e "${YELLOW}Rebuilding workspace...${NC}"
cd /home/pratham/vision_ws
colcon build --packages-select ridgeback_vision_detection --symlink-install

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}System ready to launch!${NC}"
echo ""
echo "To launch the system, run:"
echo "  cd ~/vision_ws"
echo "  source install/setup.bash"
echo "  ros2 launch ridgeback_vision_detection ridgeback_detection_launch.py"
echo ""
echo "========================================="
