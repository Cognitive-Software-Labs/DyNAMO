# DyNAMO Minimal Mapping Project

This project launches only:
- Gazebo world
- Ridgeback robot
- LiDAR (`/scan`, `/scan_rear`)
- SLAM occupancy grid (`/map`)

It does **not** launch navigation (Nav2) or mission logic.

## Project Layout
- `src/dynamo_minimal_sim/worlds/warehouse.sdf`: Gazebo world
- `src/dynamo_minimal_sim/urdf/ridgeback.urdf.xacro`: robot model + LiDAR include
- `src/dynamo_minimal_sim/config/slam_params.yaml`: SLAM Toolbox params
- `src/dynamo_minimal_sim/launch/minimal_mapping_launch.py`: minimal launch

## Run
```bash
cd /home/pratham/Desktop/DyNAMO
chmod +x run_minimal.sh
./run_minimal.sh
```

## Manual Run
```bash
cd /home/pratham/Desktop/DyNAMO
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dynamo_minimal_sim --symlink-install
source install/setup.bash
ros2 launch dynamo_minimal_sim minimal_mapping_launch.py
```
