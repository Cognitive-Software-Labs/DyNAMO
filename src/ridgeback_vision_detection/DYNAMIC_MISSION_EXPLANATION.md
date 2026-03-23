# Dynamic Autonomous Mission – Explanation Guide

This guide explains the dynamic mission flow (for viva/professor explanation) and points to exact code locations.

## 1) Where execution starts

- Entry point for dynamic node:
  - [dynamic_autonomous_mission.py](src/ridgeback_vision_detection/ridgeback_vision_detection/dynamic_autonomous_mission.py#L31)
- Dynamic class definition:
  - [dynamic_autonomous_mission.py](src/ridgeback_vision_detection/ridgeback_vision_detection/dynamic_autonomous_mission.py#L21)
- It inherits `HospitalMission` and disables static door-sweep:
  - [dynamic_autonomous_mission.py](src/ridgeback_vision_detection/ridgeback_vision_detection/dynamic_autonomous_mission.py#L23-L28)

## 2) Mission state-machine heartbeat

- Main periodic loop (10 Hz):
  - [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1375)
- Timer creation that calls this loop:
  - [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L316)

States handled in `state_machine_tick`:
- `INIT` → [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1444)
- `EXPLORE` → [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1496)
- `SCAN_360` → [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1831)
- `NAVIGATE` → [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1863)
- `WAIT` → [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1940)
- `RETURN` → [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1958)

## 3) Exactly where navigation begins

Inside `INIT` branch:
1. Wait for odometry.
2. Wait for Nav2 action server.
3. Wait for `map -> base_link` TF.
4. Save start pose in map frame.
5. Switch state to `EXPLORE`.

Code:
- [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1444-L1493)

## 4) How A* is used in your project

This node does not implement A* manually. It calls Nav2 `NavigateToPose`; Nav2 runs A* in planner server.

Planner config:
- `NavfnPlanner` plugin block:
  - [nav2_params.yaml](src/ridgeback_vision_detection/config/nav2_params.yaml#L161-L166)
- A* enabled:
  - [nav2_params.yaml](src/ridgeback_vision_detection/config/nav2_params.yaml#L165)
- Unknown-space planning enabled:
  - [nav2_params.yaml](src/ridgeback_vision_detection/config/nav2_params.yaml#L166)

Goal send from mission code:
- `send_nav2_goal` method:
  - [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L393)

Action lifecycle callbacks:
- Goal accepted/rejected callback:
  - [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L434)
- Goal result callback:
  - [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L450)

## 5) How dynamic frontier exploration works

Frontier extraction function:
- [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L985)

What it does:
1. Reads occupancy grid (`/map`).
2. Finds frontier cells = free cells adjacent to unknown cells.
3. Clusters frontier cells and filters tiny/noisy clusters.
4. Scores candidates by distance, cluster size, and heading bias.
5. Returns best frontier `(x, y)` in map frame.

The EXPLORE branch sends this dynamic frontier goal via Nav2:
- [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1807-L1827)

## 6) Dynamic detection and object approach

Image processing / red sphere detection:
- [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1163)

When sphere is confirmed during exploration/scan:
- Transition to `NAVIGATE`:
  - [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1832-L1840)

Visual servo behavior:
- In `NAVIGATE` state:
  - [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1863-L1937)

## 7) WAIT and RETURN-home logic

WAIT 10 seconds near sphere:
- [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1940-L1956)

Start RETURN sequence:
- [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1339)

RETURN using Nav2 A* to start pose:
- [detector.py](src/ridgeback_vision_detection/ridgeback_vision_detection/detector.py#L1958)

## 8) How to run this dynamic file without wiring it into setup/launch

From workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 -m ridgeback_vision_detection.dynamic_autonomous_mission
```

This keeps existing launcher and entry points unchanged.
