#!/bin/bash
set -e

cd /home/pratham/Desktop/DyNAMO
source /opt/ros/jazzy/setup.bash

colcon build --packages-select dynamo_minimal_sim --symlink-install
source install/setup.bash


ros2 launch dynamo_minimal_sim minimal_mapping_launch.py
