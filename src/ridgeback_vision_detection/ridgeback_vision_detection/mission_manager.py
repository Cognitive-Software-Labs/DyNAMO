from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener

from ridgeback_vision_detection.managers.exploration_manager import ExplorationManager
from ridgeback_vision_detection.managers.navigation_manager import NavigationManager
from ridgeback_vision_detection.managers.perception_manager import PerceptionManager
from ridgeback_vision_detection.managers.return_home_manager import ReturnHomeManager
from ridgeback_vision_detection.managers.safety_controller import SafetyController
from ridgeback_vision_detection.mission_types import LaserSnapshot, MissionState, Pose2D, ReturnSubState


@dataclass
class MissionConfig:
    loop_hz: float = 10.0
    wait_sec: float = 10.0
    scan_sec: float = 8.0
    nav_collision_front_m: float = 0.72
    nav_collision_side_m: float = 0.40
    target_preempt_front_clear_m: float = 0.70
    scan_track_yaw_kp: float = 1.4
    scan_track_max_yaw: float = 0.7
    target_nav_min_step_m: float = 1.2
    target_nav_max_step_m: float = 3.0
    target_nav_replan_sec: float = 1.0

    approach_speed: float = 0.16
    approach_crawl_speed: float = 0.08
    approach_yaw_kp: float = 1.3
    approach_max_yaw: float = 0.8
    approach_center_deadband: float = 0.08
    approach_slowdown_dist_m: float = 1.3
    approach_min_standoff_m: float = 0.60
    approach_wall_clearance_buffer_m: float = 0.20
    approach_replan_cooldown_sec: float = 2.5
    approach_entry_max_dist_m: float = 2.8
    approach_arrival_area_px: float = 2800.0
    approach_lost_timeout_sec: float = 4.0

    # Odom-anchored return controller (robust to map drift).
    return_odom_xy_tol: float = 0.18
    return_odom_yaw_tol: float = 0.12
    return_odom_speed: float = 0.22
    return_odom_yaw_kp: float = 1.8
    return_odom_turn_only_rad: float = 0.55


class MissionManager(Node):
    """Production-style mission orchestrator with explicit state entry/exit."""

    def __init__(self) -> None:
        super().__init__("hospital_mission")
        self.cfg = MissionConfig()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav = NavigationManager(self)
        self.explorer = ExplorationManager()
        self.perception = PerceptionManager()
        self.safety = SafetyController()
        self.return_home = ReturnHomeManager()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.result_pub = self.create_publisher(Image, "/vision_detection/image_result", img_qos)

        self.create_subscription(Image, "/camera/image_raw", self._on_image, img_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, "/scan", lambda m: self._on_scan("front", m), scan_qos)
        self.create_subscription(LaserScan, "/scan_rear", lambda m: self._on_scan("rear", m), scan_qos)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        self.state = MissionState.INIT
        self._state_enter_time = self.now_sec()

        self.start_pose_map: Optional[Pose2D] = None
        self.start_pose_odom: Optional[Pose2D] = None
        self.latest_odom: Optional[Odometry] = None

        self.scan_snapshot = LaserSnapshot()
        self._scan_by_source = {"front": None, "rear": None}

        self._pose_cache_tick: Optional[Pose2D] = None
        self._tick_start_sec = 0.0

        self._scan_start_sec = 0.0
        self._wait_start_sec = 0.0
        self._approach_lost_since = 0.0
        self._ignore_target_until_sec = 0.0
        self._last_target_nav_plan_sec = 0.0

        self._active_explore_goal = None

        self.create_timer(1.0 / self.cfg.loop_hz, self._tick)
        self.get_logger().info("MissionManager initialized with modular architecture")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        return q

    @staticmethod
    def quat_to_yaw(q: Quaternion) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.explorer.update_map(msg)

    def _compute_sector_mins(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float64)
        valid = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        gmin = float(np.min(valid)) if valid.size else float("inf")

        n = len(ranges)
        if n == 0 or abs(msg.angle_increment) < 1e-9:
            return {"front": float("inf"), "rear": float("inf"), "left": float("inf"), "right": float("inf"), "global": gmin}

        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        half_w = math.radians(35.0)

        def sec_min(center):
            d = np.arctan2(np.sin(angles - center), np.cos(angles - center))
            idx = np.abs(d) <= half_w
            sec = ranges[idx]
            v = sec[(sec > msg.range_min) & (sec < msg.range_max)]
            return float(np.min(v)) if v.size else float("inf")

        return {
            "front": sec_min(0.0),
            "rear": sec_min(math.pi),
            "left": sec_min(math.pi / 2.0),
            "right": sec_min(-math.pi / 2.0),
            "global": gmin,
        }

    def _on_scan(self, source: str, msg: LaserScan) -> None:
        self._scan_by_source[source] = self._compute_sector_mins(msg)

        stats = [v for v in self._scan_by_source.values() if v is not None]
        if not stats:
            return

        self.scan_snapshot = LaserSnapshot(
            front=min(s["front"] for s in stats),
            rear=min(s["rear"] for s in stats),
            left=min(s["left"] for s in stats),
            right=min(s["right"] for s in stats),
            global_min=min(s["global"] for s in stats),
        )

    def _on_image(self, msg: Image) -> None:
        now = self.now_sec()
        det = self.perception.update_from_image(msg, now)

        if self.perception.last_annotated is not None:
            frame = self.perception.last_annotated.copy()
            cv2.putText(
                frame,
                f"STATE: {self.state.name}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"TARGET stable={det.stable} area={det.area:.0f} dist={det.est_distance_m:.1f}m",
                (20, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
            )
            self.result_pub.publish(self.perception.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

    def _lookup_map_pose(self) -> Optional[Pose2D]:
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.15),
            )
            return Pose2D(
                x=float(t.transform.translation.x),
                y=float(t.transform.translation.y),
                yaw=self.quat_to_yaw(t.transform.rotation),
            )
        except Exception:
            return None

    def _pose_cached_this_tick(self) -> Optional[Pose2D]:
        if self._pose_cache_tick is None:
            self._pose_cache_tick = self._lookup_map_pose()
        return self._pose_cache_tick

    def _latest_odom_pose(self) -> Optional[Pose2D]:
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        return Pose2D(x=float(p.x), y=float(p.y), yaw=self.quat_to_yaw(q))

    def _home_pose_map_from_start_odom(self, map_pose_now: Pose2D) -> Optional[Pose2D]:
        """Project stored start pose in odom frame into current map frame."""
        if self.start_pose_odom is None:
            return None

        odom_now = self._latest_odom_pose()
        if odom_now is None:
            return None

        # Compute map<-odom from current base poses:
        # map_T_odom = map_T_base * inverse(odom_T_base)
        th = self.normalize_angle(map_pose_now.yaw - odom_now.yaw)
        c = math.cos(th)
        s = math.sin(th)
        tx = map_pose_now.x - (c * odom_now.x - s * odom_now.y)
        ty = map_pose_now.y - (s * odom_now.x + c * odom_now.y)

        sx = self.start_pose_odom.x
        sy = self.start_pose_odom.y
        home_x = tx + c * sx - s * sy
        home_y = ty + s * sx + c * sy
        home_yaw = self.normalize_angle(th + self.start_pose_odom.yaw)
        return Pose2D(home_x, home_y, home_yaw)

    def _enter_state(self, new_state: MissionState) -> None:
        self.state = new_state
        self._state_enter_time = self.now_sec()

        if new_state == MissionState.SCAN:
            self.nav.cancel()
            self._scan_start_sec = self._state_enter_time
        elif new_state == MissionState.WAIT:
            self._wait_start_sec = self._state_enter_time
            self._publish_stop()
        elif new_state == MissionState.RETURN_HOME:
            self.nav.cancel()
            pose = self._pose_cached_this_tick()
            if pose is not None:
                home_goal = self._home_pose_map_from_start_odom(pose)
                if home_goal is None:
                    home_goal = self.start_pose_map
                if home_goal is not None:
                    self.return_home.start(home_goal, self._state_enter_time)
        elif new_state == MissionState.DONE:
            self.nav.cancel()
            self._publish_stop()

        self.get_logger().info(f"State -> {new_state.name}")

    def _publish_safe(self, cmd: Twist, allow_spin: bool = False) -> None:
        safe = self.safety.apply(cmd, self.scan_snapshot, allow_spin=allow_spin)
        self.cmd_pub.publish(safe)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        self._tick_start_sec = self.now_sec()
        self._pose_cache_tick = None

        if self.state == MissionState.INIT:
            self._run_init()
        elif self.state == MissionState.EXPLORE:
            self._run_explore()
        elif self.state == MissionState.SCAN:
            self._run_scan()
        elif self.state == MissionState.APPROACH_TARGET:
            self._run_approach_target()
        elif self.state == MissionState.WAIT:
            self._run_wait()
        elif self.state == MissionState.RETURN_HOME:
            self._run_return_home()
        elif self.state == MissionState.RECOVERY:
            self._run_recovery()
        elif self.state == MissionState.DONE:
            self._publish_stop()

    def _run_init(self) -> None:
        if self.latest_odom is None:
            return
        if not self.nav.wait_until_ready(timeout_sec=0.05):
            return

        pose = self._pose_cached_this_tick()
        if pose is None:
            return

        if self.start_pose_map is None:
            self.start_pose_map = Pose2D(pose.x, pose.y, pose.yaw)
            self.get_logger().info(
                f"Start pose saved: ({pose.x:.2f}, {pose.y:.2f}, yaw={pose.yaw:.2f})"
            )

        if self.start_pose_odom is None:
            odom_pose = self._latest_odom_pose()
            if odom_pose is not None:
                self.start_pose_odom = Pose2D(odom_pose.x, odom_pose.y, odom_pose.yaw)
                self.get_logger().info(
                    f"Start odom pose saved: ({odom_pose.x:.2f}, {odom_pose.y:.2f}, yaw={odom_pose.yaw:.2f})"
                )

        self._enter_state(MissionState.EXPLORE)

    def _run_explore(self) -> None:
        now = self.now_sec()
        pose = self._pose_cached_this_tick()
        if pose is None:
            return

        det = self.perception.last_detection
        if (
            det.detected
            and now >= self._ignore_target_until_sec
            and self.scan_snapshot.front > self.cfg.target_preempt_front_clear_m
        ):
            # Target-first policy: pause frontier navigation as soon as red is
            # visible and lock heading in SCAN before final approach.
            self.nav.cancel()
            self._active_explore_goal = None
            self._enter_state(MissionState.SCAN)
            return

        can_approach = (
            det.stable
            and det.est_distance_m <= self.cfg.approach_entry_max_dist_m
            and self.scan_snapshot.front > (self.cfg.approach_min_standoff_m + 0.25)
        )
        if can_approach and now >= self._ignore_target_until_sec:
            self.nav.cancel()
            self._active_explore_goal = None
            self._enter_state(MissionState.APPROACH_TARGET)
            return

        if self.nav.consume_failure() or self.nav.consume_success():
            self._active_explore_goal = None

        if self.nav.status.name == "ACTIVE":
            if (
                self.scan_snapshot.front < self.cfg.nav_collision_front_m
                or min(self.scan_snapshot.left, self.scan_snapshot.right) < self.cfg.nav_collision_side_m
            ):
                self.nav.cancel()
                self._active_explore_goal = None
                self._enter_state(MissionState.RECOVERY)
            return

        goal = self.explorer.next_frontier_goal(pose)
        if goal is None:
            self._enter_state(MissionState.RECOVERY)
            return

        gx, gy = goal
        goal_yaw = math.atan2(gy - pose.y, gx - pose.x)
        if self.nav.send_goal(gx, gy, self.yaw_to_quat(goal_yaw)):
            self._active_explore_goal = goal

    def _run_scan(self) -> None:
        det = self.perception.last_detection
        now = self.now_sec()
        can_approach = (
            det.stable
            and det.est_distance_m <= self.cfg.approach_entry_max_dist_m
            and self.scan_snapshot.front > (self.cfg.approach_min_standoff_m + 0.25)
        )
        if can_approach and now >= self._ignore_target_until_sec:
            self.nav.cancel()
            self._enter_state(MissionState.APPROACH_TARGET)
            return

        if self.nav.consume_failure() or self.nav.consume_success():
            self._last_target_nav_plan_sec = 0.0

        if self.nav.status.name == "ACTIVE":
            if (
                self.scan_snapshot.front < self.cfg.nav_collision_front_m
                or min(self.scan_snapshot.left, self.scan_snapshot.right) < self.cfg.nav_collision_side_m
            ):
                self.nav.cancel()
                self._enter_state(MissionState.RECOVERY)
            return

        if det.stable and np.isfinite(det.est_distance_m) and det.est_distance_m > self.cfg.approach_entry_max_dist_m:
            pose = self._pose_cached_this_tick()
            if pose is None:
                return

            # Convert image offset to yaw offset and place a conservative
            # waypoint along that bearing; Nav2 handles obstacle-aware routing.
            half = max(1.0, self.perception.cfg.image_width / 2.0)
            error = (half - (det.cx if det.cx is not None else half)) / half
            yaw_offset = error * (self.perception.cfg.camera_hfov_rad / 2.0)
            bearing = pose.yaw + yaw_offset

            step = det.est_distance_m - self.cfg.approach_entry_max_dist_m
            step = max(self.cfg.target_nav_min_step_m, min(self.cfg.target_nav_max_step_m, step))

            if (now - self._last_target_nav_plan_sec) >= self.cfg.target_nav_replan_sec:
                gx = pose.x + step * math.cos(bearing)
                gy = pose.y + step * math.sin(bearing)
                goal_yaw = math.atan2(gy - pose.y, gx - pose.x)
                if self.nav.send_goal(gx, gy, self.yaw_to_quat(goal_yaw)):
                    self._last_target_nav_plan_sec = now
            return

        if det.detected:
            half = max(1.0, self.perception.cfg.image_width / 2.0)
            error = (half - (det.cx if det.cx is not None else half)) / half
            cmd = Twist()
            cmd.angular.z = max(
                -self.cfg.scan_track_max_yaw,
                min(self.cfg.scan_track_max_yaw, self.cfg.scan_track_yaw_kp * error),
            )
            self._publish_safe(cmd, allow_spin=True)
            return

        if (now - self._scan_start_sec) >= self.cfg.scan_sec:
            self._publish_stop()
            self._enter_state(MissionState.EXPLORE)
            return

        cmd = Twist()
        cmd.angular.z = 0.7
        self._publish_safe(cmd, allow_spin=True)

    def _run_approach_target(self) -> None:
        det = self.perception.last_detection
        now = self.now_sec()

        if not det.detected:
            if self._approach_lost_since <= 0.0:
                self._approach_lost_since = now

            if (now - self._approach_lost_since) > self.cfg.approach_lost_timeout_sec:
                self._publish_stop()
                self._enter_state(MissionState.RECOVERY)
                return

            cmd = Twist()
            cmd.angular.z = 0.5
            self._publish_safe(cmd, allow_spin=True)
            return

        self._approach_lost_since = 0.0

        desired_standoff = max(
            self.cfg.approach_min_standoff_m,
            self.safety.cfg.min_front_stop + self.cfg.approach_wall_clearance_buffer_m,
        )
        near_wall = (
            self.scan_snapshot.front <= desired_standoff
            or min(self.scan_snapshot.left, self.scan_snapshot.right)
            <= (self.safety.cfg.min_side_stop + 0.08)
        )

        if det.est_distance_m <= desired_standoff or near_wall:
            # Hold distance and force a short re-plan window so exploration can
            # choose a safer approach angle instead of pushing into a wall.
            self._publish_stop()
            self._ignore_target_until_sec = now + self.cfg.approach_replan_cooldown_sec
            self._enter_state(MissionState.RECOVERY)
            return

        if det.area >= self.cfg.approach_arrival_area_px and self.scan_snapshot.front <= 0.9:
            self._publish_stop()
            self._enter_state(MissionState.WAIT)
            return

        half = max(1.0, self.perception.cfg.image_width / 2.0)
        error = (half - (det.cx if det.cx is not None else half)) / half
        if abs(error) < self.cfg.approach_center_deadband:
            error = 0.0

        speed = self.cfg.approach_speed
        if det.est_distance_m < self.cfg.approach_slowdown_dist_m:
            speed = self.cfg.approach_crawl_speed

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = max(-self.cfg.approach_max_yaw, min(self.cfg.approach_max_yaw, self.cfg.approach_yaw_kp * error))
        self._publish_safe(cmd)

    def _run_wait(self) -> None:
        self._publish_stop()
        if (self.now_sec() - self._wait_start_sec) >= self.cfg.wait_sec:
            self._enter_state(MissionState.RETURN_HOME)

    def _run_return_home(self) -> None:
        # Primary return path: odom-anchored direct controller to exact start pose.
        if self.start_pose_odom is not None:
            odom_pose = self._latest_odom_pose()
            if odom_pose is not None:
                home = self.start_pose_odom
                dx = home.x - odom_pose.x
                dy = home.y - odom_pose.y
                dist = math.hypot(dx, dy)

                if dist > self.cfg.return_odom_xy_tol:
                    yaw_to_home = math.atan2(dy, dx)
                    yaw_err = self.normalize_angle(yaw_to_home - odom_pose.yaw)

                    cmd = Twist()
                    if abs(yaw_err) > self.cfg.return_odom_turn_only_rad:
                        cmd.linear.x = 0.0
                        cmd.angular.z = max(-0.8, min(0.8, self.cfg.return_odom_yaw_kp * yaw_err))
                    else:
                        cmd.linear.x = max(0.07, min(self.cfg.return_odom_speed, 0.45 * dist))
                        cmd.angular.z = max(-0.75, min(0.75, self.cfg.return_odom_yaw_kp * yaw_err))

                    self._publish_safe(cmd, allow_spin=True)
                    return

                final_yaw_err = self.normalize_angle(home.yaw - odom_pose.yaw)
                if abs(final_yaw_err) <= self.cfg.return_odom_yaw_tol:
                    self._publish_stop()
                    self._enter_state(MissionState.DONE)
                    return

                cmd = Twist()
                cmd.angular.z = max(-0.7, min(0.7, 2.0 * final_yaw_err))
                self._publish_safe(cmd, allow_spin=True)
                return

        # Fallback return path if odom is unavailable.
        pose = self._pose_cached_this_tick()
        if pose is None or self.start_pose_map is None:
            return

        update = self.return_home.update(
            now_sec=self.now_sec(),
            pose=pose,
            nav=self.nav,
            yaw_to_quat_fn=self.yaw_to_quat,
        )

        if update.state in (ReturnSubState.DIRECT_HOME, ReturnSubState.RECOVERY, ReturnSubState.FINAL_ALIGNMENT):
            cmd = Twist()
            cmd.linear.x = update.command_linear
            cmd.angular.z = update.command_angular
            self._publish_safe(cmd, allow_spin=True)
        # In NAVIGATE_HOME, Nav2 owns /cmd_vel and this node must stay silent.

        if update.done:
            self._enter_state(MissionState.DONE)

    def _run_recovery(self) -> None:
        # Standardized mission-level recovery: rotate -> short forward arc -> explore.
        elapsed = self.now_sec() - self._state_enter_time
        turn_sign = -1.0 if self.scan_snapshot.left < self.scan_snapshot.right else 1.0
        if elapsed < 2.0:
            cmd = Twist()
            cmd.angular.z = 0.7 * turn_sign
            self._publish_safe(cmd, allow_spin=True)
            return

        if elapsed < 3.4:
            cmd = Twist()
            cmd.linear.x = 0.18
            cmd.angular.z = 0.22 * turn_sign
            self._publish_safe(cmd)
            return

        self._publish_stop()
        self._enter_state(MissionState.EXPLORE)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
