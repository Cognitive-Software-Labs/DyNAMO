from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid

from ridgeback_vision_detection.mission_types import MapGoal, Pose2D


@dataclass
class FrontierConfig:
    min_cluster_cells: int = 4
    min_goal_dist_m: float = 1.1
    max_goal_dist_m: float = 15.0
    obstacle_clearance_m: float = 0.38
    info_gain_weight: float = 0.015
    preferred_goal_dist_m: float = 2.2


class ExplorationManager:
    """Generic frontier selection (cluster + score) for unknown maps."""

    def __init__(self, config: FrontierConfig | None = None) -> None:
        self.cfg = config or FrontierConfig()
        self._map: Optional[OccupancyGrid] = None

    def update_map(self, grid: OccupancyGrid) -> None:
        self._map = grid

    def _grid(self):
        if self._map is None:
            return None
        h = self._map.info.height
        w = self._map.info.width
        return np.array(self._map.data, dtype=np.int16).reshape(h, w)

    def _world_to_cell(self, x: float, y: float):
        info = self._map.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return None
        return row, col

    def _cell_to_world(self, row: int, col: int):
        info = self._map.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def next_frontier_goal(self, robot_pose: Pose2D) -> Optional[MapGoal]:
        if self._map is None:
            return None
        grid = self._grid()
        if grid is None:
            return None

        free = grid == 0
        unknown = grid < 0
        occupied = grid > 50

        clear_cells = max(1, int(self.cfg.obstacle_clearance_m / max(1e-6, self._map.info.resolution)))
        kernel = np.ones((2 * clear_cells + 1, 2 * clear_cells + 1), dtype=np.uint8)
        blocked = cv2.dilate(occupied.astype(np.uint8), kernel, iterations=1) > 0

        frontier = np.zeros_like(free, dtype=bool)
        frontier[1:-1, 1:-1] = free[1:-1, 1:-1] & (
            unknown[:-2, 1:-1] | unknown[2:, 1:-1] |
            unknown[1:-1, :-2] | unknown[1:-1, 2:]
        )
        frontier &= ~blocked

        if not np.any(frontier):
            return None

        robot_cell = self._world_to_cell(robot_pose.x, robot_pose.y)
        if robot_cell is None:
            return None
        rr, rc = robot_cell

        labels_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier.astype(np.uint8), connectivity=8
        )

        best_score = float("inf")
        best_goal = None

        for label in range(1, labels_count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.cfg.min_cluster_cells:
                continue

            cy, cx = centroids[label][1], centroids[label][0]
            row = int(round(cy))
            col = int(round(cx))

            if row < 0 or row >= grid.shape[0] or col < 0 or col >= grid.shape[1]:
                continue
            if blocked[row, col]:
                continue

            wx, wy = self._cell_to_world(row, col)
            dist = float(np.hypot(wx - robot_pose.x, wy - robot_pose.y))
            if dist < self.cfg.min_goal_dist_m or dist > self.cfg.max_goal_dist_m:
                continue

            # Lower score is better; prefer medium-range goals that create map growth.
            score = abs(dist - self.cfg.preferred_goal_dist_m) - self.cfg.info_gain_weight * float(area)
            if score < best_score:
                best_score = score
                best_goal = (wx, wy)

        if best_goal is not None:
            return best_goal

        # Fallback: if clustering is too sparse early in SLAM, sample raw
        # frontier cells so we still move and grow the map.
        rows, cols = np.where(frontier)
        if rows.size == 0:
            return None

        stride = max(1, rows.size // 250)
        for i in range(0, rows.size, stride):
            row = int(rows[i])
            col = int(cols[i])
            if blocked[row, col]:
                continue

            wx, wy = self._cell_to_world(row, col)
            dist = float(np.hypot(wx - robot_pose.x, wy - robot_pose.y))
            if dist < self.cfg.min_goal_dist_m or dist > self.cfg.max_goal_dist_m:
                continue

            score = abs(dist - self.cfg.preferred_goal_dist_m)
            if score < best_score:
                best_score = score
                best_goal = (wx, wy)

        return best_goal
