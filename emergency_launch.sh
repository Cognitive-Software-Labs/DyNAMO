#!/bin/bash

# ULTIMATE FIX - When Nothing Works
# This script will fix and launch the SIMPLIFIED version

set -e

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              EMERGENCY FIX & LAUNCH                          ║"
echo "║         (Simplified Version - No Nav2/SLAM)                  ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Step 1: KILL EVERYTHING
echo -e "${RED}[1/6] Killing all processes...${NC}"
killall -9 ros2 2>/dev/null || true
killall -9 gz 2>/dev/null || true
killall -9 gzserver 2>/dev/null || true
killall -9 gzclient 2>/dev/null || true
killall -9 python3 2>/dev/null || true
killall -9 ruby 2>/dev/null || true
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
rm -rf /dev/shm/sem.* 2>/dev/null || true
sleep 3
echo -e "${GREEN}      ✓ All processes killed${NC}"
echo ""

# Step 2: Clean workspace
echo -e "${YELLOW}[2/6] Cleaning workspace...${NC}"
cd /home/pratham/vision_ws
rm -rf build/ridgeback_vision_detection install/ridgeback_vision_detection log/ 2>/dev/null || true
echo -e "${GREEN}      ✓ Workspace cleaned${NC}"
echo ""

# Step 3: Rebuild
echo -e "${YELLOW}[3/6] Rebuilding package...${NC}"
colcon build --packages-select ridgeback_vision_detection --symlink-install 2>&1 | \
  grep -E "Starting|Finished|Summary|ERROR|WARNING" || true
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo -e "${GREEN}      ✓ Build successful${NC}"
else
    echo -e "${RED}      ✗ Build failed${NC}"
    exit 1
fi
echo ""

# Step 4: Source
echo -e "${YELLOW}[4/6] Sourcing workspace...${NC}"
source /home/pratham/vision_ws/install/setup.bash
echo -e "${GREEN}      ✓ Workspace sourced${NC}"
echo ""

# Step 5: Launch file check (Skipping manual copy)
echo -e "${YELLOW}[5/6] Verifying launch file...${NC}"
if [ -f "install/ridgeback_vision_detection/share/ridgeback_vision_detection/launch/simple_launch.py" ]; then
    echo -e "${GREEN}      ✓ simple_launch.py ready${NC}"
else
    echo -e "${RED}      ✗ simple_launch.py missing! Rebuilding...${NC}"
    colcon build --packages-select ridgeback_vision_detection --symlink-install
fi
echo ""

# Step 6: Launch
echo -e "${YELLOW}[6/6] Launching simplified system...${NC}"
echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                    LAUNCHING SYSTEM                          ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  This is a SIMPLIFIED version without Nav2/SLAM              ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  Timeline:                                                   ║${NC}"
echo -e "${BLUE}║  • 0-5 sec:  Gazebo starts                                   ║${NC}"
echo -e "${BLUE}║  • 5-10 sec: Robot spawns                                    ║${NC}"
echo -e "${BLUE}║  • 10+ sec:  Detector starts                                 ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  What you should see:                                        ║${NC}"
echo -e "${BLUE}║  ✓ Gazebo window with warehouse                              ║${NC}"
echo -e "${BLUE}║  ✓ Ridgeback robot on left side                             ║${NC}"
echo -e "${BLUE}║  ✓ Red sphere in center                                      ║${NC}"
echo -e "${BLUE}║  ✓ Robot starts rotating after ~16 seconds                   ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  To view camera (in NEW terminal):                           ║${NC}"
echo -e "${BLUE}║  source ~/vision_ws/install/setup.bash                       ║${NC}"
echo -e "${BLUE}║  ros2 run rqt_image_view rqt_image_view \\                   ║${NC}"
echo -e "${BLUE}║    /vision_detection/image_result                            ║${NC}"
echo -e "${BLUE}║                                                              ║${NC}"
echo -e "${BLUE}║  Press Ctrl+C to stop                                        ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
sleep 2

ros2 launch ridgeback_vision_detection simple_launch.py
