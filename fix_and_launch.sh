#!/bin/bash

# SIMPLE FIX AND LAUNCH SCRIPT
# Run this to fix everything and launch

set -e  # Exit on error

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         RIDGEBACK VISION DETECTION - FIX & LAUNCH            ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Step 1: Kill everything
echo -e "${YELLOW}[1/5] Stopping all processes...${NC}"
killall -9 ros2 2>/dev/null || true
killall -9 gz 2>/dev/null || true
killall -9 python3 2>/dev/null || true
killall -9 ruby 2>/dev/null || true
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
sleep 2
echo -e "${GREEN}      ✓ All processes stopped${NC}"
echo ""

# Step 2: Rebuild
echo -e "${YELLOW}[2/5] Rebuilding workspace...${NC}"
cd /home/pratham/vision_ws
colcon build --packages-select ridgeback_vision_detection --symlink-install 2>&1 | grep -E "Starting|Finished|Summary|ERROR" || true
echo -e "${GREEN}      ✓ Build complete${NC}"
echo ""

# Step 3: Source
echo -e "${YELLOW}[3/5] Sourcing workspace...${NC}"
source /home/pratham/vision_ws/install/setup.bash
echo -e "${GREEN}      ✓ Workspace sourced${NC}"
echo ""

# Step 4: Verify
echo -e "${YELLOW}[4/5] Verifying configuration...${NC}"
if [ -f "src/ridgeback_vision_detection/urdf/camera_extras.urdf.xacro" ]; then
    if grep -q '<topic>camera</topic>' src/ridgeback_vision_detection/urdf/camera_extras.urdf.xacro; then
        echo -e "${GREEN}      ✓ Camera configuration correct${NC}"
    else
        echo -e "${RED}      ✗ Camera configuration incorrect${NC}"
        exit 1
    fi
fi
echo ""

# Step 5: Launch
echo -e "${YELLOW}[5/5] Launching system...${NC}"
echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  System is starting...                                       ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  What to expect:                                             ║${NC}"
echo -e "${BLUE}║  • Gazebo will open with warehouse world                     ║${NC}"
echo -e "${BLUE}║  • Robot spawns at left side                                 ║${NC}"
echo -e "${BLUE}║  • Red sphere visible in center                              ║${NC}"
echo -e "${BLUE}║  • RViz opens for visualization                              ║${NC}"
echo -e "${BLUE}║  • Robot waits 6 seconds, then starts scanning              ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  To view camera:                                             ║${NC}"
echo -e "${BLUE}║  Open new terminal and run:                                  ║${NC}"
echo -e "${BLUE}║  ros2 run rqt_image_view rqt_image_view \\                   ║${NC}"
echo -e "${BLUE}║    /vision_detection/image_result                            ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  Press Ctrl+C here to stop everything                        ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
sleep 2

ros2 launch ridgeback_vision_detection ridgeback_detection_launch.py
