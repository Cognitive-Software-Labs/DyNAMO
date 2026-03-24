from __future__ import annotations

from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from ridgeback_vision_detection.mission_types import NavGoalStatus


@dataclass
class NavFeedback:
    x: float = 0.0
    y: float = 0.0
    stamp_sec: float = 0.0


class NavigationManager:
    """Race-safe Nav2 action wrapper with explicit lifecycle."""

    def __init__(self, node: Node, action_name: str = "navigate_to_pose") -> None:
        self.node = node
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.status = NavGoalStatus.IDLE
        self.feedback = NavFeedback()

        self._goal_handle = None
        self._active_goal_seq = 0

    def wait_until_ready(self, timeout_sec: float = 0.1) -> bool:
        return self.client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, x: float, y: float, yaw_quat, frame_id: str = "map") -> bool:
        if self.status == NavGoalStatus.ACTIVE:
            return False

        self._active_goal_seq += 1
        seq = self._active_goal_seq

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = yaw_quat

        self.status = NavGoalStatus.ACTIVE
        future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda f, seq_id=seq: self._on_goal_response(f, seq_id))
        return True

    def cancel(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self.status = NavGoalStatus.IDLE

    def _on_goal_response(self, future, seq_id: int) -> None:
        if seq_id != self._active_goal_seq:
            return
        handle = future.result()
        if not handle.accepted:
            self.status = NavGoalStatus.FAILED
            self._goal_handle = None
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, seq=seq_id: self._on_result(f, seq))

    def _on_result(self, future, seq_id: int) -> None:
        if seq_id != self._active_goal_seq:
            return

        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.status = NavGoalStatus.SUCCEEDED
        else:
            self.status = NavGoalStatus.FAILED
        self._goal_handle = None

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        pose = fb.current_pose.pose.position
        self.feedback = NavFeedback(
            x=float(pose.x),
            y=float(pose.y),
            stamp_sec=self.node.get_clock().now().nanoseconds / 1e9,
        )

    def consume_success(self) -> bool:
        if self.status == NavGoalStatus.SUCCEEDED:
            self.status = NavGoalStatus.IDLE
            return True
        return False

    def consume_failure(self) -> bool:
        if self.status == NavGoalStatus.FAILED:
            self.status = NavGoalStatus.IDLE
            return True
        return False
