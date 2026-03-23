#!/bin/bash

# Camera Test Script
# Run this AFTER launching the system to verify camera is working

echo "========================================="
echo "  Camera Feed Test"
echo "========================================="
echo ""
echo "This script checks if the camera is"
echo "publishing images correctly."
echo ""
echo "Make sure you have launched the system"
echo "with ./launch_ridgeback.sh first!"
echo ""
echo "========================================="
echo ""

# Source workspace
source /home/pratham/vision_ws/install/setup.bash

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "Checking if ROS2 is running..."
if ! pgrep -x "ros2" > /dev/null; then
    echo -e "${RED}✗ ROS2 not running${NC}"
    echo ""
    echo "Please launch the system first:"
    echo "  ./launch_ridgeback.sh"
    exit 1
fi
echo -e "${GREEN}✓ ROS2 is running${NC}"
echo ""

echo "Checking available topics..."
topics=$(ros2 topic list 2>/dev/null)

if echo "$topics" | grep -q "/camera/image_raw"; then
    echo -e "${GREEN}✓ /camera/image_raw topic exists${NC}"
else
    echo -e "${RED}✗ /camera/image_raw topic not found${NC}"
    echo ""
    echo "Available topics:"
    echo "$topics"
    exit 1
fi

if echo "$topics" | grep -q "/vision_detection/image_result"; then
    echo -e "${GREEN}✓ /vision_detection/image_result topic exists${NC}"
else
    echo -e "${YELLOW}⚠ /vision_detection/image_result topic not found${NC}"
    echo "  (Detector node may not be running)"
fi
echo ""

echo "Checking camera topic info..."
camera_info=$(ros2 topic info /camera/image_raw 2>/dev/null)
echo "$camera_info"
echo ""

echo "Checking if camera is publishing..."
echo "Waiting for messages (timeout: 5 seconds)..."

if timeout 5 ros2 topic echo /camera/image_raw --once > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Camera is publishing images!${NC}"
    echo ""
    
    # Get publishing rate
    echo "Measuring publish rate..."
    rate=$(timeout 3 ros2 topic hz /camera/image_raw 2>&1 | grep "average rate" | awk '{print $3}')
    if [ ! -z "$rate" ]; then
        echo -e "${GREEN}✓ Camera rate: $rate Hz${NC}"
    fi
    echo ""
    
    echo "========================================="
    echo -e "${GREEN}Camera Test: PASSED${NC}"
    echo "========================================="
    echo ""
    echo "To view the camera feed, run:"
    echo "  ros2 run rqt_image_view rqt_image_view /camera/image_raw"
    echo ""
    echo "To view annotated feed with detection:"
    echo "  ros2 run rqt_image_view rqt_image_view /vision_detection/image_result"
    echo ""
else
    echo -e "${RED}✗ No messages received from camera${NC}"
    echo ""
    echo "Possible issues:"
    echo "  1. Gazebo simulation not started"
    echo "  2. Bridge not running"
    echo "  3. Camera sensor not initialized"
    echo ""
    echo "Check Gazebo topics:"
    echo "  gz topic -l | grep camera"
    echo ""
    echo "Check bridge status:"
    echo "  ros2 node info /ros_gz_bridge"
    echo ""
    exit 1
fi

echo "Checking detector node..."
if ros2 node list 2>/dev/null | grep -q "vision_detector"; then
    echo -e "${GREEN}✓ Vision detector node is running${NC}"
    
    # Check if detector is processing images
    echo ""
    echo "Checking detector output..."
    if timeout 3 ros2 topic echo /vision_detection/image_result --once > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Detector is processing images${NC}"
    else
        echo -e "${YELLOW}⚠ Detector not publishing (may be starting up)${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Vision detector node not found${NC}"
    echo "  Available nodes:"
    ros2 node list 2>/dev/null | sed 's/^/    /'
fi

echo ""
echo "========================================="
echo "Test Complete!"
echo "========================================="
