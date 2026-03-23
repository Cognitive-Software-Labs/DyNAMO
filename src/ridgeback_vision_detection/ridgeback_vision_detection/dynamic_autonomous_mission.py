#!/usr/bin/env python3
"""
Dynamic Autonomous Mission Node (separate file, not wired into launch/setup).

Purpose
- Keep your current mission file untouched.
- Provide a professor-friendly dynamic version that uses frontier exploration
  + Nav2 A* planning without static door-sweep waypoints.

How A* is used
- This node sends map-frame goals to Nav2's NavigateToPose action server.
- Nav2 planner (NavfnPlanner with use_astar=true) computes the A* global path.
- This file handles: mission state machine, frontier goal selection, detection,
  and return-home sequencing.
"""

import math

from .detector import HospitalMission


class DynamicAutonomousMission(HospitalMission):
    # Dynamic-only behavior: disable static room waypoint sweeping
    DOOR_SWEEP_ENABLE = False

    # Keep exploration fully dynamic from frontier extraction
    PATROL_START_IMMEDIATELY = False
    PATROL_ENABLE_AFTER = 9999.0

    # Keep the rest from HospitalMission; only dynamic toggles are overridden.

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            '🧠 Dynamic mode enabled: frontier-driven exploration + Nav2 A* '
            '(static door-sweep disabled)')


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = DynamicAutonomousMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
