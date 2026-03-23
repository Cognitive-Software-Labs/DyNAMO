#!/bin/bash

# System Verification Script for Ridgeback Vision Detection
# Checks if all required dependencies are installed

echo "========================================="
echo "  System Verification"
echo "========================================="
echo ""

# Source workspace if it exists
if [ -f "/home/pratham/vision_ws/install/setup.bash" ]; then
    source /home/pratham/vision_ws/install/setup.bash
fi

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_package() {
    if ros2 pkg list | grep -q "^$1$"; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 (MISSING)"
        return 1
    fi
}

check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 (MISSING)"
        return 1
    fi
}

missing=0

echo "Checking ROS2 Packages:"
echo "----------------------"
packages=(
    "ridgeback_vision_detection"
    "ros_gz_sim"
    "ros_gz_bridge"
    "nav2_bringup"
    "slam_toolbox"
    "robot_state_publisher"
)

for pkg in "${packages[@]}"; do
    check_package "$pkg" || ((missing++))
done

echo ""
echo "Checking System Commands:"
echo "------------------------"
commands=(
    "gz"
    "ros2"
    "colcon"
)

for cmd in "${commands[@]}"; do
    check_command "$cmd" || ((missing++))
done

echo ""
echo "Checking Python Modules:"
echo "-----------------------"
if python3 -c "import cv2" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} opencv-python (cv2)"
else
    echo -e "${RED}✗${NC} opencv-python (cv2) (MISSING)"
    ((missing++))
fi

if python3 -c "import rclpy" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} rclpy"
else
    echo -e "${RED}✗${NC} rclpy (MISSING)"
    ((missing++))
fi

if python3 -c "from cv_bridge import CvBridge" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} cv_bridge"
else
    echo -e "${RED}✗${NC} cv_bridge (MISSING)"
    ((missing++))
fi

echo ""
echo "Checking Workspace:"
echo "------------------"
if [ -f "/home/pratham/vision_ws/install/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} Workspace built"
else
    echo -e "${RED}✗${NC} Workspace not built"
    ((missing++))
fi

if [ -f "/home/pratham/vision_ws/src/ridgeback_vision_detection/package.xml" ]; then
    echo -e "${GREEN}✓${NC} Package source exists"
else
    echo -e "${RED}✗${NC} Package source missing"
    ((missing++))
fi

echo ""
echo "========================================="
if [ $missing -eq 0 ]; then
    echo -e "${GREEN}All checks passed! System is ready.${NC}"
    echo ""
    echo "To launch the system, run:"
    echo "  ./launch_ridgeback.sh"
else
    echo -e "${RED}Found $missing missing dependencies.${NC}"
    echo ""
    echo "To install missing ROS2 packages:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-<package-name>"
    echo ""
    echo "To rebuild workspace:"
    echo "  cd ~/vision_ws"
    echo "  colcon build --packages-select ridgeback_vision_detection"
fi
echo "========================================="
