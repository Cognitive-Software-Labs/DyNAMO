from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist

from ridgeback_vision_detection.mission_types import (
    NavGoalStatus,
    Pose2D,
    ReturnSubState,
    ReturnUpdate,
)
from ridgeback_vision_detection.managers.navigation_manager import NavigationManager


@dataclass
class ReturnConfig:
    home_dist_tol: float = 0.22
    align_tol_rad: float = 0.12
    stall_time_sec: float = 6.0
    stall_dist_m: float = 0.08
    direct_home_trigger_dist: float = 0.9
    direct_home_speed: float = 0.15
    direct_home_yaw_kp: float = 1.6
    direct_turn_only_rad: float = 0.55
    diverge_dist_m: float = 0.20
    diverge_time_sec: float = 2.0
    recovery_rotate_sec: float = 2.0
    recovery_backup_sec: float = 1.2
    max_replans: int = 10


class ReturnHomeManager:
    """Dedicated RETURN_HOME sub-state machine."""

    def __init__(self, config: ReturnConfig | None = None) -> None:
        self.cfg = config or ReturnConfig()
        self.substate = ReturnSubState.NAVIGATE_HOME
        self.home_pose: Pose2D | None = None

        self._stall_ref_pose: Pose2D | None = None
        self._stall_start = 0.0
        self._recovery_until = 0.0
        self._recovery_phase = ""
        self._replan_count = 0
        self._best_dist = float("inf")
        self._diverge_start = 0.0

    @staticmethod
    def _norm(a: float) -> float:
        import math

        return math.atan2(math.sin(a), math.cos(a))

    def start(self, home_pose: Pose2D, now_sec: float) -> None:
        self.home_pose = home_pose
        self.substate = ReturnSubState.NAVIGATE_HOME
        self._stall_ref_pose = None
        self._stall_start = now_sec
        self._replan_count = 0
        self._recovery_until = 0.0
        self._recovery_phase = ""
        self._best_dist = float("inf")
        self._diverge_start = 0.0

    def _distance_to_home(self, pose: Pose2D) -> float:
        import math

        return math.hypot(self.home_pose.x - pose.x, self.home_pose.y - pose.y)

    def update(self, now_sec: float, pose: Pose2D, nav: NavigationManager, yaw_to_quat_fn) -> ReturnUpdate:
        if self.home_pose is None:
            return ReturnUpdate(state=self.substate, done=False, debug_text="home-not-set")

        dist = self._distance_to_home(pose)

        if self.substate == ReturnSubState.NAVIGATE_HOME:
            if dist < self._best_dist:
                self._best_dist = dist
                self._diverge_start = 0.0
            elif dist > (self._best_dist + self.cfg.diverge_dist_m):
                if self._diverge_start <= 0.0:
                    self._diverge_start = now_sec
                elif (now_sec - self._diverge_start) > self.cfg.diverge_time_sec:
                    nav.cancel()
                    self.substate = ReturnSubState.RECOVERY
                    self._recovery_phase = "ROTATE"
                    self._recovery_until = now_sec + self.cfg.recovery_rotate_sec
                    return ReturnUpdate(self.substate, debug_text="diverging-recovery")

            if dist <= self.cfg.home_dist_tol:
                nav.cancel()
                self.substate = ReturnSubState.FINAL_ALIGNMENT
                return ReturnUpdate(self.substate, debug_text="near-home-align")

            if dist <= self.cfg.direct_home_trigger_dist:
                nav.cancel()
                self.substate = ReturnSubState.DIRECT_HOME
                self._stall_ref_pose = Pose2D(pose.x, pose.y, pose.yaw)
                self._stall_start = now_sec
                return ReturnUpdate(self.substate, debug_text="switch-direct-home")

            if nav.status == NavGoalStatus.IDLE:
                nav.send_goal(self.home_pose.x, self.home_pose.y, yaw_to_quat_fn(self.home_pose.yaw))

            if nav.consume_success():
                if dist <= self.cfg.home_dist_tol:
                    self.substate = ReturnSubState.FINAL_ALIGNMENT
                    return ReturnUpdate(self.substate, debug_text="nav-success-align")
                if dist <= self.cfg.direct_home_trigger_dist:
                    self.substate = ReturnSubState.DIRECT_HOME
                    self._stall_ref_pose = Pose2D(pose.x, pose.y, pose.yaw)
                    self._stall_start = now_sec
                    return ReturnUpdate(self.substate, debug_text="nav-success-direct-home")
                # Nav2 may report success due loose controller tolerances; keep
                # trying toward exact home instead of finalizing far away.
                return ReturnUpdate(self.substate, require_replan=True, debug_text="nav-success-replan")

            if nav.consume_failure():
                nav.cancel()
                self.substate = ReturnSubState.RECOVERY
                self._recovery_phase = "ROTATE"
                self._recovery_until = now_sec + self.cfg.recovery_rotate_sec
                return ReturnUpdate(self.substate, debug_text="nav-failed-recovery")

            if self._stall_ref_pose is None:
                self._stall_ref_pose = Pose2D(pose.x, pose.y, pose.yaw)
                self._stall_start = now_sec
            else:
                moved = ((pose.x - self._stall_ref_pose.x) ** 2 + (pose.y - self._stall_ref_pose.y) ** 2) ** 0.5
                if moved >= self.cfg.stall_dist_m:
                    self._stall_ref_pose = Pose2D(pose.x, pose.y, pose.yaw)
                    self._stall_start = now_sec
                elif (now_sec - self._stall_start) > self.cfg.stall_time_sec:
                    nav.cancel()
                    self.substate = ReturnSubState.RECOVERY
                    self._recovery_phase = "BACKUP"
                    self._recovery_until = now_sec + self.cfg.recovery_backup_sec
                    return ReturnUpdate(self.substate, debug_text="nav-stalled-recovery")

            return ReturnUpdate(self.substate, debug_text="navigating")

        if self.substate == ReturnSubState.DIRECT_HOME:
            import math

            nav.cancel()

            if dist <= self.cfg.home_dist_tol:
                self.substate = ReturnSubState.FINAL_ALIGNMENT
                return ReturnUpdate(self.substate, debug_text="direct-near-home")

            yaw_to_home = math.atan2(self.home_pose.y - pose.y, self.home_pose.x - pose.x)
            yaw_err = self._norm(yaw_to_home - pose.yaw)

            cmd = Twist()
            if abs(yaw_err) > self.cfg.direct_turn_only_rad:
                cmd.linear.x = 0.0
                cmd.angular.z = max(-0.8, min(0.8, self.cfg.direct_home_yaw_kp * yaw_err))
            else:
                cmd.linear.x = max(0.06, min(self.cfg.direct_home_speed, 0.40 * dist))
                cmd.angular.z = max(-0.7, min(0.7, self.cfg.direct_home_yaw_kp * yaw_err))

            if self._stall_ref_pose is None:
                self._stall_ref_pose = Pose2D(pose.x, pose.y, pose.yaw)
                self._stall_start = now_sec
            else:
                moved = ((pose.x - self._stall_ref_pose.x) ** 2 + (pose.y - self._stall_ref_pose.y) ** 2) ** 0.5
                if moved >= self.cfg.stall_dist_m:
                    self._stall_ref_pose = Pose2D(pose.x, pose.y, pose.yaw)
                    self._stall_start = now_sec
                elif (now_sec - self._stall_start) > self.cfg.stall_time_sec:
                    self.substate = ReturnSubState.RECOVERY
                    self._recovery_phase = "BACKUP"
                    self._recovery_until = now_sec + self.cfg.recovery_backup_sec
                    return ReturnUpdate(self.substate, debug_text="direct-stalled-recovery")

            return ReturnUpdate(
                state=self.substate,
                command_linear=cmd.linear.x,
                command_angular=cmd.angular.z,
                debug_text="direct-home",
            )

        if self.substate == ReturnSubState.RECOVERY:
            import math

            nav.cancel()
            cmd = Twist()
            yaw_to_home = math.atan2(self.home_pose.y - pose.y, self.home_pose.x - pose.x)
            yaw_err = self._norm(yaw_to_home - pose.yaw)
            turn_sign = 1.0 if yaw_err >= 0.0 else -1.0
            if self._recovery_phase == "ROTATE":
                cmd.angular.z = 0.7 * turn_sign
            else:
                cmd.linear.x = -0.14
                cmd.angular.z = 0.35 * turn_sign

            if now_sec >= self._recovery_until:
                self._replan_count += 1
                if self._replan_count > self.cfg.max_replans:
                    # Do not finish far from home. Reset recovery budget and keep
                    # trying navigation with a fresh cycle.
                    self._replan_count = 0

                self.substate = ReturnSubState.NAVIGATE_HOME
                self._best_dist = dist
                self._diverge_start = 0.0
                return ReturnUpdate(self.substate, require_replan=True, debug_text="replan-nav")

            return ReturnUpdate(
                state=self.substate,
                command_linear=cmd.linear.x,
                command_angular=cmd.angular.z,
                debug_text=f"recovery-{self._recovery_phase.lower()}",
            )

        if self.substate == ReturnSubState.FINAL_ALIGNMENT:
            if dist > self.cfg.home_dist_tol:
                if dist <= self.cfg.direct_home_trigger_dist:
                    self.substate = ReturnSubState.DIRECT_HOME
                    return ReturnUpdate(self.substate, debug_text="align-needs-direct-home")
                self.substate = ReturnSubState.NAVIGATE_HOME
                return ReturnUpdate(self.substate, require_replan=True, debug_text="align-too-far-replan")

            yaw_err = self._norm(self.home_pose.yaw - pose.yaw)
            if abs(yaw_err) <= self.cfg.align_tol_rad:
                self.substate = ReturnSubState.COMPLETE
                return ReturnUpdate(self.substate, done=True, debug_text="aligned")

            cmd = Twist()
            cmd.angular.z = max(-0.7, min(0.7, 2.0 * yaw_err))
            return ReturnUpdate(
                state=self.substate,
                command_linear=0.0,
                command_angular=cmd.angular.z,
                debug_text="final-align",
            )

        return ReturnUpdate(ReturnSubState.COMPLETE, done=True, debug_text="complete")
