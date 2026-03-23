#!/usr/bin/env python3
"""
Hospital Mission Controller – Autonomous Frontier Exploration & Retrieve
=========================================================================
State machine:
  INIT             wait for odometry + Nav2
  EXPLORE          frontier-based autonomous exploration (reads SLAM map)
  SCAN_360         rotate 360° in place, scanning for the red sphere
  NAVIGATE         visual-servo toward the detected sphere (with obstacle check)
  WAIT             wait 10 s at the sphere
  RETURN           Nav2 path-planned return to start pose (obstacle-free)
  DONE             mission complete

Key features:
  - Frontier exploration: robot finds unexplored edges in the SLAM map
    and navigates to them autonomously — works for any sphere position
  - Sphere detected in camera at ANY time → immediately switches to NAVIGATE
  - Nav2 path planning for EXPLORE and RETURN (avoids walls)
  - Visual servo for NAVIGATE with LiDAR safety stop
  - Time-based 360° scan (immune to skid-steer odometry drift)
  - LiDAR obstacle check in all driving states
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

import cv2
import numpy as np
import math
import heapq


class HospitalMission(Node):

    # ── Tuning ───────────────────────────────────────────────
    SAFE_DISTANCE       = 0.42    # safer clearance from walls (m)
    SIDE_SAFE_DISTANCE  = 0.34    # lateral safety distance from nearby obstacles (m)
    CRITICAL_STOP_DIST  = 0.24    # emergency stop if any direction is closer than this (m)
    SEARCH_YAW_SPEED    = 0.8     # faster scan / reacquire
    SCAN_DURATION       = 12.0    # seconds for full 360°
    SCAN_TARGET_RAD     = 2.0 * math.pi  # true 360° target angle
    MOVE_SPEED          = 0.3
    MIN_DETECT_AREA     = 80      # detect sphere earlier at longer range
    WAIT_SECONDS        = 10.0
    SPHERE_RADIUS_M     = 0.3
    CAMERA_HFOV_RAD     = 1.047
    IMAGE_WIDTH         = 640
    CONFIRM_FRAMES      = 2       # quicker lock once sphere appears
    ANGLE_TOLERANCE     = 0.15
    DIST_TOLERANCE      = 0.3

    # Visual-servoing
    SERVO_SPEED         = 0.2     # faster approach speed
    SERVO_KP            = 2.0     # stronger correction to center the sphere
    SERVO_MAX_YAW       = 1.0
    BLOB_CLOSE_AREA     = 2500    # consider close sooner
    ARRIVAL_LIDAR_BLOB_AREA = 1000  # allow lidar-stop arrival even if blob smaller
    SERVO_LOST_TIMEOUT  = 6.0  # allow more time to reacquire the sphere while turning
    NAV_LIDAR_STOP      = 0.8    # balanced stop distance near sphere/walls (lower so it can approach closer)

    # Return (  via Nav2)
    NAV2_TIMEOUT        = 90.0    # seconds before Nav2 goal considered stuck
    RETURN_MAX_RETRIES  = 5       # keep retries bounded before ending mission safely
    RETURN_RETRY_DELAY  = 1.0     # seconds between retries
    HOME_REACHED_DIST   = 0.65    # must be within this map distance to finish
    HOME_REACHED_ODOM_DIST = 1.8  # odom sanity gate for DONE; relaxed to avoid false non-completion
    HOME_STABLE_SEC     = 2.0     # must stay near home this long (manual fallback)
    STOP_ON_FIRST_SUCCESS = True  # stop mission when sphere found OR map fully explored
    MAP_COMPLETE_RATIO  = 0.92    # known-map ratio considered fully explored
    MAP_COMPLETE_HOLD_S = 12.0    # keep no-frontier this long before declaring completion

    # Frontier exploration
    FRONTIER_MIN_DIST   = 1.5     # ignore frontiers closer than this (m)
    FRONTIER_MAX_DIST   = 12.0    # ignore frontiers farther than this (m)
    FRONTIER_TIMEOUT    = 90.0    # give up on a frontier after this many seconds
    FRONTIER_MAX_FAILS  = 5       # trigger recovery after this many consecutive Nav2 failures
    FRONTIER_MIN_CLUSTER_CELLS = 8    # ignore tiny/noisy frontier blobs
    FRONTIER_CLEARANCE_M = 0.25       # keep frontier goals away from walls/obstacles
    FRONTIER_BLOCK_RADIUS = 2.0       # avoid recently failed frontier neighbourhood
    FRONTIER_BLOCK_TTL = 120.0        # seconds to remember blocked frontier goals
    NO_FRONTIER_MOVE_SPEED = 0.30     # active exploration speed when no frontiers are available
    FRONTIER_FORWARD_BIAS = 1.5        # penalty weight against backwards goals
    FRONTIER_MIN_FORWARD_M = -0.1      # reject goals too far behind start heading
    STALL_DIST_THRESH = 0.25           # min movement to be considered progress (m)
    STALL_TIME_THRESH = 12.0           # trigger unstick sooner when progress is poor
    IDLE_STALL_DIST_THRESH = 0.10      # no-goal mode: movement needed to count progress (m)
    IDLE_STALL_TIME_THRESH = 9.0       # no-goal mode: trigger unstick if stationary this long
    UNSTICK_TURN_S = 2.2
    UNSTICK_BACKUP_S = 1.2
    UNSTICK_BACKUP_SPEED = 0.14
    UNSTICK_ARC_YAW = 0.55
    UNSTICK_FORWARD_S = 1.8
    UNSTICK_FORWARD_SPEED = 0.32

    #   probe-goal recovery (used when frontier goals keep failing)
    PROBE_OFFSETS = [
        (4.0, 0.0),
        (8.0, 0.0),
        (12.0, 0.0),
        (16.0, 0.0),
        (14.0, 1.8),
        (9.0, 1.8),
        (2.0, 1.8),
        (14.0, -1.8),
        (9.0, -1.8),
        (2.0, -1.8),
        (3.0, 4.5),
        (10.0, 4.5),
        (16.0, 4.5),
        (3.0, 7.5),
        (10.0, 7.5),
        (16.0, 7.5),
        (3.0, -4.5),
        (10.0, -4.5),
        (3.0, -7.5),
        (10.0, -7.5),
    ]
    PROBE_REBASE_DIST = 6.0          # rebuild probe pattern if robot moved this far (m)
    PROBE_ROAM_SECONDS = 4.0         # roam locally this long after repeated probe failures
    NO_FRONTIER_BLOCKED_TIMEOUT = 6.0  # if blocked this long, start backup-rotate escape
    NO_FRONTIER_BACKUP_SPEED = 0.10    # m/s backup speed during local escape
    EXPLORE_SCAN_INTERVAL = 35.0       # force periodic 360 scan while exploring
    SCAN_MIN_MOVE_M = 1.2              # require this map displacement before next 360 scan
    PATROL_ENABLE_AFTER = 35.0         # switch to deterministic   patrol after this explore time
    PATROL_FAIL_TRIGGER = 3            # or earlier if repeated frontier/nav failures
    PATROL_START_IMMEDIATELY = False   # disabled when door-sweep mode is active
    PATROL_FAIL_ESCAPE_TRIGGER = 3     # only escape after repeated patrol failures
    PATROL_BACKTRACK_MARGIN = 0.0      # disallow backtracking during normal patrol
    ROOM_TRAP_Y = 3.0                  # |y| beyond this is considered inside side room
    CORRIDOR_X_MIN = -4.8              # clamp escape x into corridor bounds
    CORRIDOR_X_MAX = 14.5
    MAP_SAFE_X_MIN = -5.0              # hard safety bounds for exploration recovery
    MAP_SAFE_X_MAX = 15.0
    MAP_SAFE_Y_MAX = 7.2
    ESCAPE_RETRY_COOLDOWN = 6.0        # don't retry nearly identical failed escape too soon
    DOOR_SWEEP_ENABLE = False          # disable static sweep; prefer dynamic frontier/A* search
    FORCE_LEFT_ROOM_SEQUENCE = True    # enforce: left room 1 -> left room 2 (sphere room)
    SECOND_LEFT_ROOM_GOAL_IDX = 4      # index in DOOR_SWEEP_GOALS of second-left-room scan point
    # (x, y, should_scan) -- forced order: first left room scan, then second left room scan
    DOOR_SWEEP_GOALS = [
        (2.0, 1.0, False),
        (3.8, 1.2, False),
        (4.8, 1.8, True),
        (9.2, 1.2, False),
        (10.2, 3.6, True),
    ]
    DOOR_SWEEP_RETRY_COOLDOWN = 18.0   # cooldown before retrying skipped waypoint
    DOOR_SWEEP_MAX_RETRIES = 1         # retries per waypoint before skip
    DOOR_SWEEP_STALL_DIST = 0.10       # required movement to avoid stall
    DOOR_SWEEP_STALL_TIME = 18.0       # seconds with low movement before cancel
    DOOR_SWEEP_GOAL_TIMEOUT = 45.0     # timeout per door sweep waypoint
    DOOR_SWEEP_REACHED_DIST = 0.55     # treat waypoint as reached when this close in map frame
    DOOR_SWEEP_MAX_ABS_Y = 5.2         # allow true room-entry waypoints during door sweep
    DOOR_SWEEP_CORRIDOR_ADAPT = 0.20   # keep corridor goals near safe bench-avoid offset
    DOOR_SWEEP_RECOVERY_COOLDOWN = 5.0  # pause re-goaling briefly after unstick recovery
    COSTMAP_CLEAR_COOLDOWN = 2.5       # throttle repeated clear-costmap calls
    EXPLORE_MAX_ABS_Y = 6.5            # allow planned room entry while avoiding deep dead-ends
    FRONTIER_REACHED_DIST = 0.45       # close-enough fallback when Nav2 does not report success
    DYNAMIC_EXPLORE_ENABLE = True      # use SLAM-grid A* lookahead for dynamic exploration
    DYNAMIC_ONLY_MODE = True           # avoid static waypoint patrol/probe fallbacks
    DYNAMIC_PROBE_RADII = [1.8, 3.0, 4.2, 5.6]  # radial candidate distances for dynamic probe goals (m)
    DYNAMIC_PROBE_ANGLES_DEG = [0, 15, -15, 30, -30, 60, -60, 90, -90, 120, -120, 180]
    DYNAMIC_PROBE_FORWARD_CONE_DEG = 80.0   # prefer goals inside this heading cone first
    DYNAMIC_PROBE_WIDE_CONE_DEG = 130.0     # second-pass cone before allowing rear goals
    ASTAR_LOOKAHEAD_M = 1.8            # send short waypoint along A* path (m)
    ASTAR_MAX_EXPANSIONS = 25000       # bound A* search work on large maps
    PATROL_GOALS = [                   # corridor sweep + doorway peeks + target room entry
        (-1.0, 0.0),
        (2.0, 0.0),
        (4.0, 0.0),
        (4.0, 1.1),
        (4.0, -1.1),
        (6.5, 0.0),
        (8.5, 0.0),
        (10.0, 0.0),
        (10.0, 1.2),
        (10.0, -1.2),
        (10.0, 2.6),
        (10.2, 4.2),
        (10.2, 5.0),
    ]

    def __init__(self):
        super().__init__('hospital_mission')

        # TF2 for map-frame lookups
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        result_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.result_pub = self.create_publisher(
            Image, '/vision_detection/image_result', result_qos)
        self.pred_path_pub = self.create_publisher(Path, '/navigate_path', 10)
        self._est_sphere_dist = float('inf')  # estimated distance to sphere

        # Subscribers
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, '/camera/image_raw',
                                 self.image_callback, cam_qos)
        self.create_subscription(Odometry, '/odom',
                                 self.odom_callback, 10)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, '/scan',
                                 self.scan_callback, lidar_qos)

        # SLAM map (TRANSIENT_LOCAL so we get the latest even if we subscribe late)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.create_subscription(OccupancyGrid, '/map',
                                 self.map_callback, map_qos)

        # State
        self.bridge        = CvBridge()
        self.state         = 'INIT'
        self.start_pose    = None       # (x, y, yaw) in MAP frame
        self.start_odom_pose = None     # (x, y, yaw) in ODOM frame
        self.robot_x       = 0.0        # odom frame
        self.robot_y       = 0.0
        self.robot_yaw     = 0.0
        self.odom_received = False

        # Frontier exploration
        self._map_data          = None   # latest OccupancyGrid from SLAM
        self._frontier_goal     = None   # current target (x, y) or None
        self._frontier_count    = 0      # total frontiers visited
        self._frontier_fail_cnt = 0      # consecutive Nav2 failures
        self._no_frontier_since = None   # time when map had no frontiers
        self._blocked_frontiers = []     # [(x, y, t_sec), ...] failed goals to avoid
        self._probe_goals        = []     # transformed PROBE_OFFSETS in map frame
        self._probe_cursor       = 0
        self._probe_anchor       = None   # (x, y) map pose used to build probe goals
        self._probe_roam_until   = 0.0
        self._next_explore_scan  = 0.0
        self._last_scan_map_pose = None   # (x, y) of last completed/started scan
        self._explore_start_time = 0.0
        self._patrol_mode        = False
        self._patrol_idx         = 0
        self._escape_to_corridor = False
        self._force_probe_mode   = False
        self._stall_ref_pose     = None   # (x, y) map pose when stall watch starts
        self._stall_start_time   = 0.0
        self._idle_stall_ref_pose = None  # (x, y) map pose for no-goal stall watch
        self._idle_stall_start_time = 0.0
        self._unstick_phase      = None   # None | TURN | BACKUP | FORWARD
        self._unstick_phase_until = 0.0
        self._unstick_turn_dir   = 1.0    # alternate turn direction between recoveries
        self._no_frontier_blocked_since = 0.0
        self._last_failed_goal = None      # (x, y)
        self._last_failed_goal_time = 0.0
        self._door_sweep_idx = 0
        self._door_sweep_goal_idx = None
        self._door_sweep_goal = None
        self._door_scan_pending = False
        self._door_goal_attempts = 0
        self._door_failed_until = {}
        self._door_stall_ref_pose = None
        self._door_stall_start_time = 0.0
        self._door_recovery_until = 0.0
        self._last_costmap_clear_time = 0.0

        self.scan_start_time    = None
        self.scan_last_yaw      = None
        self.scan_accumulated   = 0.0
        self.nav2_goal_active   = False
        self.nav2_goal_handle   = None
        self._nav2_goal_seq     = 0       # increments every new Nav2 goal
        self.nav2_reached       = False
        self.nav2_failed        = False
        self._nav2_ready        = False
        self._nav2_goal_time    = 0.0
        self._nav2_goal_raw_xy  = None   # high-level requested goal (before lookahead)
        self._nav2_goal_sent_xy = None   # actual Nav2 sent goal
        self._return_phase      = 'NAV2'
        self._return_start_time = 0.0
        self._return_nav2_sent  = False
        self._return_retry_count = 0
        self._return_retry_after = 0.0
        self._last_progress_log = 0.0
        self._home_near_since   = None
        self._map_complete_since = None

        # Detection / servo
        self.confirm_count      = 0
        self.blob_cx            = None
        self.blob_area          = 0
        self.blob_detected      = False
        self.sphere_confirmed   = False
        self._target_locked     = False   # once sphere is confirmed, don't fall back to exploration
        self._mission_complete  = False   # hard latch after reaching home
        self.servo_lost_time    = None

        # LiDAR — front + rear
        self.front_min_range    = float('inf')
        self.rear_min_range     = float('inf')
        self.left_min_range     = float('inf')
        self.right_min_range    = float('inf')
        self.global_min_range   = float('inf')
        self._scan_base_yaw_in_lidar = 0.0
        self._last_scan_tf_lookup = 0.0

        # Wait
        self.wait_start_time    = None
        self._sphere_reached    = False

        # Misc
        self.init_start    = None
        self.init_logged   = False
        self._img_count    = 0

        # Costmap clearing services
        from nav2_msgs.srv import ClearEntireCostmap
        self._clear_local  = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap')

        # 10 Hz state machine
        self.create_timer(0.1, self.state_machine_tick)
        self.get_logger().info('🏥 Hospital Mission – Autonomous Frontier Exploration mode')

    # ── Helpers ──────────────────────────────────────────────
    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def yaw_to_quat(yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        q.x = q.y = 0.0
        return q

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _estimate_distance(self, pixel_diameter):
        if pixel_diameter <= 0:
            return float('inf')
        focal_px = (self.IMAGE_WIDTH / 2.0) / math.tan(
            self.CAMERA_HFOV_RAD / 2.0)
        return (2.0 * self.SPHERE_RADIUS_M * focal_px) / pixel_diameter

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def clear_costmaps(self):
        """Clear both local and global costmaps for fresh planning."""
        from nav2_msgs.srv import ClearEntireCostmap
        req = ClearEntireCostmap.Request()
        if self._clear_local.wait_for_service(timeout_sec=1.0):
            self._clear_local.call_async(req)
        if self._clear_global.wait_for_service(timeout_sec=1.0):
            self._clear_global.call_async(req)
        self.get_logger().info('🧹 Costmaps cleared')

    def _complete_mission(self, reason):
        self.cancel_nav2_goal()
        self.stop_robot()
        self._mission_complete = True
        self.state = 'DONE'
        self.get_logger().info(f'✅ Mission complete: {reason}')

    def _known_map_ratio(self):
        if self._map_data is None:
            return 0.0
        data = np.array(self._map_data.data, dtype=np.int16)
        if data.size == 0:
            return 0.0
        known = np.count_nonzero(data != -1)
        return float(known) / float(data.size)

    def _is_map_complete(self):
        return self._known_map_ratio() >= self.MAP_COMPLETE_RATIO

    def _apply_safety_to_cmd(self, cmd: Twist):
        """Clamp cmd_vel to keep safe distance from obstacles in all directions."""
        # Emergency clamp first.
        if self.global_min_range < self.CRITICAL_STOP_DIST:
            cmd.linear.x = 0.0
            cmd.angular.z = self.SEARCH_YAW_SPEED * (1.0 if self.left_min_range > self.right_min_range else -1.0)
            return cmd

        # Forward safety.
        if cmd.linear.x > 0.0:
            if self.front_min_range < self.SAFE_DISTANCE:
                cmd.linear.x = 0.0
                if abs(cmd.angular.z) < 1e-3:
                    cmd.angular.z = self.SEARCH_YAW_SPEED * (1.0 if self.left_min_range > self.right_min_range else -1.0)
            else:
                # Slow down as we approach obstacles.
                safety_margin = max(0.0, self.front_min_range - self.SAFE_DISTANCE)
                if safety_margin < 0.25:
                    cmd.linear.x = min(cmd.linear.x, 0.10 + 0.8 * safety_margin)

                # Side-wall protection while moving forward.
                if self.left_min_range < self.SIDE_SAFE_DISTANCE:
                    cmd.linear.x = min(cmd.linear.x, 0.08)
                    cmd.angular.z -= 0.35
                if self.right_min_range < self.SIDE_SAFE_DISTANCE:
                    cmd.linear.x = min(cmd.linear.x, 0.08)
                    cmd.angular.z += 0.35

        # Reverse safety.
        if cmd.linear.x < 0.0 and self.rear_min_range < self.SAFE_DISTANCE:
            cmd.linear.x = 0.0
            if abs(cmd.angular.z) < 1e-3:
                cmd.angular.z = self.SEARCH_YAW_SPEED * (1.0 if self.left_min_range > self.right_min_range else -1.0)

        return cmd

    def _start_unstick(self, reason=''):
        """Start deterministic unstick sequence with alternating turn direction."""
        self.cancel_nav2_goal()
        self._unstick_turn_dir = -self._unstick_turn_dir
        self._unstick_phase = 'TURN'
        self._unstick_phase_until = self.now_sec() + self.UNSTICK_TURN_S
        self._stall_ref_pose = None
        self._stall_start_time = 0.0
        self._idle_stall_ref_pose = None
        self._idle_stall_start_time = 0.0
        self._frontier_goal = None
        self._force_probe_mode = True
        self.clear_costmaps()
        if reason:
            self.get_logger().warn(f'♻️ Unstick started: {reason}')
        else:
            self.get_logger().warn('♻️ Unstick started')

    def _publish_active_search_cmd(self):
        """Publish local exploration motion when no reliable Nav2 target exists."""
        cmd = Twist()
        now = self.now_sec()

        # Clear path ahead -> move forward smoothly.
        if self.front_min_range > (self.SAFE_DISTANCE + 0.25):
            self._no_frontier_blocked_since = 0.0
            cmd.linear.x = self.NO_FRONTIER_MOVE_SPEED
            cmd.angular.z = 0.0
            self.cmd_pub.publish(self._apply_safety_to_cmd(cmd))
            return

        # Blocked ahead -> rotate; if blocked for long, backup+turn to escape pocket.
        if self._no_frontier_blocked_since <= 0.0:
            self._no_frontier_blocked_since = now
        blocked_for = now - self._no_frontier_blocked_since

        # Slightly blocked but not critically close -> crawl forward with gentle arc
        # to avoid spinning forever in place.
        if self.front_min_range > (self.SAFE_DISTANCE - 0.05):
            cmd.linear.x = min(0.10, self.NO_FRONTIER_MOVE_SPEED)
            cmd.angular.z = self.SEARCH_YAW_SPEED * 0.45
        elif blocked_for > self.NO_FRONTIER_BLOCKED_TIMEOUT and \
                self.rear_min_range > (self.SAFE_DISTANCE + 0.1):
            cmd.linear.x = -self.NO_FRONTIER_BACKUP_SPEED
            cmd.angular.z = self.SEARCH_YAW_SPEED * 0.6
        else:
            cmd.angular.z = self.SEARCH_YAW_SPEED * 1.15

        self.cmd_pub.publish(self._apply_safety_to_cmd(cmd))

    # ── Nav2 goal sending ────────────────────────────────────
    def send_nav2_goal(self, x, y, yaw=0.0):
        """Send a NavigateToPose goal to Nav2."""
        if self.nav2_goal_active:
            self.get_logger().warn(
                '⚠️ Nav2 goal already active; ignoring overlapping goal request')
            return

        raw_x, raw_y = float(x), float(y)
        if self.state == 'EXPLORE' and self.DYNAMIC_EXPLORE_ENABLE:
            lookahead = self._astar_lookahead_goal(raw_x, raw_y, allow_unknown=True)
            if lookahead is not None:
                x, y = lookahead
                if math.hypot(x - raw_x, y - raw_y) > 0.10:
                    self.get_logger().info(
                        f'🧠 A* lookahead -> ({x:.1f}, {y:.1f}) '
                        f'toward ({raw_x:.1f}, {raw_y:.1f})')
        self._nav2_goal_raw_xy = (raw_x, raw_y)
        self._nav2_goal_sent_xy = (float(x), float(y))

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self.yaw_to_quat(yaw)

        self.nav2_goal_active = True
        self.nav2_reached     = False
        self.nav2_failed      = False
        self._nav2_goal_time  = self.now_sec()
        self._nav2_goal_seq  += 1
        goal_seq = self._nav2_goal_seq

        self.get_logger().info(
            f'🗺️  Nav2 goal → ({x:.1f}, {y:.1f}, yaw={yaw:.2f})')

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav2_feedback)
        send_future.add_done_callback(
            lambda fut, seq=goal_seq: self._nav2_goal_response(fut, seq))

    def cancel_nav2_goal(self):
        """Cancel any active Nav2 goal."""
        if self.nav2_goal_handle is not None:
            try:
                self.nav2_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.nav2_goal_active = False
        self.nav2_goal_handle = None
        self._nav2_goal_raw_xy = None
        self._nav2_goal_sent_xy = None

    def _nav2_goal_response(self, future, goal_seq):
        if goal_seq != self._nav2_goal_seq:
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Nav2 goal rejected!')
            self.nav2_failed = True
            self.nav2_goal_active = False
            return
        self.nav2_goal_handle = goal_handle
        self.get_logger().info('✅ Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, seq=goal_seq: self._nav2_result(fut, seq))

    def _nav2_result(self, future, goal_seq):
        if goal_seq != self._nav2_goal_seq:
            return

        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav2_reached = True
            self.get_logger().info('✅ Nav2 goal reached!')
        else:
            self.get_logger().warn(
                f'⚠️  Nav2 goal ended with status {status}')
            self.nav2_failed = True
        self.nav2_goal_active = False
        self.nav2_goal_handle = None

    def _nav2_feedback(self, feedback_msg):
        # Log progress every 10 seconds
        now = self.now_sec()
        if now - self._last_progress_log >= 10.0:
            self._last_progress_log = now
            fb = feedback_msg.feedback
            pose = fb.current_pose.pose.position
            self.get_logger().info(
                f'📍 Nav2 progress: pos=({pose.x:.1f}, {pose.y:.1f})')

    def _get_scan_base_yaw_in_lidar(self, scan_frame, scan_stamp=None):
        """Yaw of base_link x-axis expressed in LiDAR scan frame."""
        now = self.now_sec()
        if (now - self._last_scan_tf_lookup) < 0.5:
            return self._scan_base_yaw_in_lidar

        self._last_scan_tf_lookup = now
        if not scan_frame:
            return self._scan_base_yaw_in_lidar

        try:
            query_time = rclpy.time.Time()
            if scan_stamp is not None:
                try:
                    query_time = rclpy.time.Time.from_msg(scan_stamp)
                except Exception:
                    query_time = rclpy.time.Time()

            t = self.tf_buffer.lookup_transform(
                scan_frame,
                'base_link',
                query_time,
                timeout=rclpy.duration.Duration(seconds=0.15))
            self._scan_base_yaw_in_lidar = self.quat_to_yaw(t.transform.rotation)
        except Exception:
            # Fallback to latest available transform if timestamped query fails.
            try:
                t = self.tf_buffer.lookup_transform(
                    scan_frame,
                    'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
                self._scan_base_yaw_in_lidar = self.quat_to_yaw(t.transform.rotation)
            except Exception:
                pass

        return self._scan_base_yaw_in_lidar

    # ── LiDAR callback ───────────────────────────────────────
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        n = len(ranges)
        if n == 0:
            return

        valid_all = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        self.global_min_range = float(np.min(valid_all)) if len(valid_all) > 0 else float('inf')

        angle_inc = msg.angle_increment
        if abs(angle_inc) < 1e-9:
            return

        angles = msg.angle_min + np.arange(n, dtype=np.float64) * angle_inc
        half_width = math.radians(45.0)
        base_yaw = self._get_scan_base_yaw_in_lidar(msg.header.frame_id,
                                msg.header.stamp)

        def sector_min(center_angle):
            delta = np.arctan2(np.sin(angles - center_angle),
                               np.cos(angles - center_angle))
            in_sector = np.abs(delta) <= half_width
            sector = ranges[in_sector]
            valid = sector[(sector > msg.range_min) & (sector < msg.range_max)]
            return float(np.min(valid)) if len(valid) > 0 else float('inf')

        self.front_min_range = sector_min(base_yaw)
        self.rear_min_range = sector_min(base_yaw + math.pi)
        self.left_min_range = sector_min(base_yaw + math.pi / 2.0)
        self.right_min_range = sector_min(base_yaw - math.pi / 2.0)

    # ── Odom callback ────────────────────────────────────────
    def get_map_pose(self):
        """Look up robot position in map frame via TF2."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = self.quat_to_yaw(t.transform.rotation)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def odom_callback(self, msg):
        self.robot_x   = msg.pose.pose.position.x
        self.robot_y   = msg.pose.pose.position.y
        self.robot_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        if self.start_odom_pose is None:
            self.start_odom_pose = (self.robot_x, self.robot_y, self.robot_yaw)
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'📡 Odom: ({self.robot_x:.2f}, {self.robot_y:.2f})')

    # ── Map callback (SLAM occupancy grid) ───────────────────
    def map_callback(self, msg: OccupancyGrid):
        """Store the latest SLAM occupancy grid for frontier detection."""
        self._map_data = msg

    def _prune_blocked_frontiers(self):
        now = self.now_sec()
        self._blocked_frontiers = [
            (x, y, ts) for (x, y, ts) in self._blocked_frontiers
            if (now - ts) <= self.FRONTIER_BLOCK_TTL
        ]

    def _is_frontier_blocked(self, wx, wy):
        self._prune_blocked_frontiers()
        for bx, by, _ in self._blocked_frontiers:
            if math.hypot(wx - bx, wy - by) <= self.FRONTIER_BLOCK_RADIUS:
                return True
        return False

    def _mark_current_frontier_blocked(self):
        if self._frontier_goal is None:
            return
        fx, fy = self._frontier_goal
        self._blocked_frontiers.append((fx, fy, self.now_sec()))
        self._prune_blocked_frontiers()

    def _build_probe_goals(self, base_pose=None):
        """Build map-frame probe goals from offsets along a reference pose."""
        if base_pose is None:
            base_pose = self.start_pose
        if base_pose is None:
            self._probe_goals = []
            self._probe_anchor = None
            return
        sx, sy, syaw = base_pose

        if self.DYNAMIC_ONLY_MODE:
            goals = []
            for radius in self.DYNAMIC_PROBE_RADII:
                for ang_deg in self.DYNAMIC_PROBE_ANGLES_DEG:
                    ang = syaw + math.radians(ang_deg)
                    gx = sx + radius * math.cos(ang)
                    gy = sy + radius * math.sin(ang)
                    goals.append((gx, gy))

            goals.sort(key=lambda pt: math.hypot(pt[0] - sx, pt[1] - sy))
            self._probe_goals = goals
            self._probe_anchor = (sx, sy)
            if self._probe_cursor >= len(self._probe_goals):
                self._probe_cursor = 0
            return

        cos_y = math.cos(syaw)
        sin_y = math.sin(syaw)
        goals = []
        for dx, dy in self.PROBE_OFFSETS:
            gx = sx + dx * cos_y - dy * sin_y
            gy = sy + dx * sin_y + dy * cos_y
            goals.append((gx, gy))
        self._probe_goals = goals
        self._probe_anchor = (sx, sy)
        self._probe_cursor = 0

    def _next_probe_goal(self):
        """Get next probe goal in round-robin order, skipping blocked neighborhoods."""
        mpose = self.get_map_pose()
        if mpose is not None:
            mx, my, _ = mpose
            if self.DYNAMIC_ONLY_MODE:
                self._build_probe_goals(base_pose=mpose)
            elif (not self._probe_goals) or (self._probe_anchor is None) or \
                    (math.hypot(mx - self._probe_anchor[0],
                                my - self._probe_anchor[1]) > self.PROBE_REBASE_DIST):
                self._build_probe_goals(base_pose=mpose)

        if self.DYNAMIC_ONLY_MODE:
            if not self._probe_goals or mpose is None:
                return None

            mx, my, myaw = mpose
            candidates = []
            for gx, gy in self._probe_goals:
                if self._is_frontier_blocked(gx, gy):
                    continue
                if not self._is_goal_navigable(gx, gy, allow_unknown=True):
                    continue

                dx = gx - mx
                dy = gy - my
                dist = math.hypot(dx, dy)
                if dist < 0.6:
                    continue

                goal_bearing = math.atan2(dy, dx)
                ang_err = abs(self.normalize_angle(goal_bearing - myaw))

                # Deterministic forward-priority score:
                # short distance + low steering effort.
                score = dist + 1.2 * ang_err
                candidates.append((score, ang_err, gx, gy))

            if not candidates:
                return None

            fwd_cone = math.radians(self.DYNAMIC_PROBE_FORWARD_CONE_DEG)
            wide_cone = math.radians(self.DYNAMIC_PROBE_WIDE_CONE_DEG)

            pass1 = [c for c in candidates if c[1] <= fwd_cone]
            if pass1:
                pass1.sort(key=lambda item: item[0])
                _, _, gx, gy = pass1[0]
                return (gx, gy)

            pass2 = [c for c in candidates if c[1] <= wide_cone]
            if pass2:
                pass2.sort(key=lambda item: item[0])
                _, _, gx, gy = pass2[0]
                return (gx, gy)

            candidates.sort(key=lambda item: item[0])
            _, _, gx, gy = candidates[0]
            return (gx, gy)

        if not self._probe_goals:
            return None
        for _ in range(len(self._probe_goals)):
            gx, gy = self._probe_goals[self._probe_cursor]
            self._probe_cursor = (self._probe_cursor + 1) % len(self._probe_goals)
            if (not self._is_frontier_blocked(gx, gy)) and \
                    self._is_goal_navigable(gx, gy, allow_unknown=self.DYNAMIC_ONLY_MODE):
                return (gx, gy)
        # Fallback: first map-navigable probe even if blocked-history says no.
        for gx, gy in self._probe_goals:
            if self._is_goal_navigable(gx, gy, allow_unknown=self.DYNAMIC_ONLY_MODE):
                return (gx, gy)
        return None

    def _next_patrol_goal(self):
        """Pick next patrol goal, preferring forward progress along corridor x."""
        if self.DYNAMIC_ONLY_MODE:
            return None
        if not self.PATROL_GOALS:
            return None
        mpose = self.get_map_pose()
        mx, my = (mpose[0], mpose[1]) if mpose is not None else (None, None)

        # Pass 1: prefer goals that are not significantly behind current x.
        for _ in range(len(self.PATROL_GOALS)):
            gx, gy = self.PATROL_GOALS[self._patrol_idx]
            self._patrol_idx = (self._patrol_idx + 1) % len(self.PATROL_GOALS)
            if mx is not None and math.hypot(gx - mx, gy - my) < 0.8:
                continue
            if mx is not None and gx < (mx - self.PATROL_BACKTRACK_MARGIN):
                continue
            if self._is_frontier_blocked(gx, gy):
                continue
            if self._is_goal_navigable(gx, gy, allow_unknown=True):
                return (gx, gy)

        # Pass 2 fallback: if no forward candidate exists, allow backtracking.
        for _ in range(len(self.PATROL_GOALS)):
            gx, gy = self.PATROL_GOALS[self._patrol_idx]
            self._patrol_idx = (self._patrol_idx + 1) % len(self.PATROL_GOALS)
            if mx is not None and math.hypot(gx - mx, gy - my) < 0.8:
                continue
            if self._is_frontier_blocked(gx, gy):
                continue
            if self._is_goal_navigable(gx, gy, allow_unknown=True):
                return (gx, gy)
        return None

    def _reset_patrol_index_from_pose(self, pose=None):
        """Start patrol from the closest forward waypoint to current map pose."""
        if not self.PATROL_GOALS:
            self._patrol_idx = 0
            return

        if pose is None:
            pose = self.get_map_pose()
        if pose is None:
            self._patrol_idx = 0
            return

        mx, my, _ = pose

        best_idx = None
        best_score = float('inf')

        # Prefer forward (or near-forward) patrol points first.
        for idx, (gx, gy) in enumerate(self.PATROL_GOALS):
            if gx < (mx - self.PATROL_BACKTRACK_MARGIN):
                continue
            score = math.hypot(gx - mx, gy - my)
            if score < best_score:
                best_score = score
                best_idx = idx

        # If no forward candidates exist, fall back to nearest overall.
        if best_idx is None:
            for idx, (gx, gy) in enumerate(self.PATROL_GOALS):
                score = math.hypot(gx - mx, gy - my)
                if score < best_score:
                    best_score = score
                    best_idx = idx

        self._patrol_idx = best_idx if best_idx is not None else 0

    def _corridor_escape_goal(self):
        """Pick a reachable escape target (doorway/corridor), avoiding blocked repeats."""
        mpose = self.get_map_pose()
        if mpose is None:
            return None
        mx, my, _ = mpose

        candidates = []

        trapped_in_room = abs(my) > self.ROOM_TRAP_Y
        room_sign = 1.0 if my >= 0.0 else -1.0

        # If stuck deep in a side room, route through known doorway x positions first
        # (doorway mouth -> corridor centerline) instead of straight crossing walls.
        if trapped_in_room:
            doorway_x = []
            seen_x = set()
            for px, py in self.PATROL_GOALS:
                if abs(py) < 0.9 or abs(py) > 1.6:
                    continue
                key = round(px, 2)
                if key in seen_x:
                    continue
                seen_x.add(key)
                doorway_x.append(px)

            doorway_x.sort(key=lambda px: abs(px - mx))
            for px in doorway_x:
                gx = max(self.CORRIDOR_X_MIN, min(self.CORRIDOR_X_MAX, px))
                candidates.append((gx, room_sign * 1.0))
                candidates.append((gx, 0.0))

        # Immediate corridor projection from current x (useful when already near corridor).
        if not trapped_in_room:
            proj_x = max(self.CORRIDOR_X_MIN, min(self.CORRIDOR_X_MAX, mx))
            candidates.append((proj_x, 0.0))

        # Add corridor-center anchors derived from patrol goals.
        for px, py in self.PATROL_GOALS:
            if abs(py) <= 1.6:
                gx = max(self.CORRIDOR_X_MIN, min(self.CORRIDOR_X_MAX, px))
                candidates.append((gx, 0.0))

        # De-duplicate while preserving order.
        uniq = []
        seen = set()
        for gx, gy in candidates:
            key = (round(gx, 2), round(gy, 2))
            if key in seen:
                continue
            seen.add(key)
            uniq.append((gx, gy))

        # Prefer nearby candidates, but skip recently blocked and too-close goals.
        def score(goal):
            gx, gy = goal
            backtrack_penalty = 2.0 if gx < (mx - self.PATROL_BACKTRACK_MARGIN) else 0.0
            return math.hypot(gx - mx, gy - my) + 0.3 * abs(gy) + backtrack_penalty

        sorted_candidates = sorted(uniq, key=score)
        now = self.now_sec()

        for gx, gy in sorted_candidates:
            if math.hypot(gx - mx, gy - my) < 0.8:
                continue
            # When trapped in a room, allow backtracking to nearest doorway.
            if (not trapped_in_room) and gx < (mx - 0.05):
                continue
            if self._is_frontier_blocked(gx, gy):
                continue
            if self._last_failed_goal is not None and \
                    (now - self._last_failed_goal_time) < self.ESCAPE_RETRY_COOLDOWN and \
                    math.hypot(gx - self._last_failed_goal[0],
                               gy - self._last_failed_goal[1]) < 0.8:
                continue
            if self._is_goal_navigable(gx, gy, allow_unknown=True):
                return (gx, gy)

        # Fallback pass: if everything is blocked, still pick a map-navigable one.
        for gx, gy in sorted_candidates:
            if math.hypot(gx - mx, gy - my) < 0.8:
                continue
            if (not trapped_in_room) and gx < (mx - 0.05):
                continue
            if self._last_failed_goal is not None and \
                    (now - self._last_failed_goal_time) < self.ESCAPE_RETRY_COOLDOWN and \
                    math.hypot(gx - self._last_failed_goal[0],
                               gy - self._last_failed_goal[1]) < 0.8:
                continue
            if self._is_goal_navigable(gx, gy, allow_unknown=True):
                return (gx, gy)

        return None

    def _is_goal_navigable(self, wx, wy, allow_unknown=False):
        """Check whether goal cell is inside current map and on/near free space."""
        if self._map_data is None:
            return False
        info = self._map_data.info
        w, h = info.width, info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        if col < 1 or col >= (w - 1) or row < 1 or row >= (h - 1):
            return False

        grid = np.array(self._map_data.data, dtype=np.int16).reshape(h, w)
        patch = grid[row - 1:row + 2, col - 1:col + 2]
        pad = 3
        big_patch = grid[row - pad:row + pad + 1, col - pad:col + pad + 1]
        if grid[row, col] > 50:
            return False

        if allow_unknown:
            occ_ratio = float(np.count_nonzero(patch > 50)) / float(patch.size)
            return occ_ratio <= 0.20

        # Accept if center is free or majority of small neighborhood is free.
        center_free = grid[row, col] == 0
        free_ratio = float(np.count_nonzero(patch == 0)) / float(patch.size)
        big_occ_ratio = float(np.count_nonzero(big_patch > 50)) / float(big_patch.size)
        if big_occ_ratio > 0.12:
            return False
        return center_free or free_ratio >= 0.60

    def _world_to_grid(self, wx, wy):
        if self._map_data is None:
            return None
        info = self._map_data.info
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return None
        return (row, col)

    def _grid_to_world(self, row, col):
        if self._map_data is None:
            return None
        info = self._map_data.info
        wx = info.origin.position.x + (col + 0.5) * info.resolution
        wy = info.origin.position.y + (row + 0.5) * info.resolution
        return (wx, wy)

    def _astar_path(self, start_rc, goal_rc, allow_unknown=False):
        """A* on the SLAM occupancy grid. Returns list[(row, col)] or None."""
        if self._map_data is None:
            return None

        info = self._map_data.info
        h, w = info.height, info.width
        grid = np.array(self._map_data.data, dtype=np.int16).reshape(h, w)

        occupied = (grid > 50)
        unknown = (grid < 0)
        blocked = occupied.copy()
        if not allow_unknown:
            blocked |= unknown

        clearance_cells = max(1, int(self.FRONTIER_CLEARANCE_M / max(info.resolution, 1e-6)))
        kernel_size = 2 * clearance_cells + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        blocked = cv2.dilate(blocked.astype(np.uint8), kernel, iterations=1) > 0

        sr, sc = start_rc
        gr, gc = goal_rc
        if sr < 0 or sr >= h or sc < 0 or sc >= w:
            return None
        if gr < 0 or gr >= h or gc < 0 or gc >= w:
            return None
        if blocked[sr, sc] or blocked[gr, gc]:
            return None

        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142),
        ]

        def heuristic(r, c):
            return math.hypot(gr - r, gc - c)

        g_score = np.full((h, w), np.inf, dtype=np.float64)
        g_score[sr, sc] = 0.0
        parent = {}
        closed = np.zeros((h, w), dtype=bool)
        heap = [(heuristic(sr, sc), 0.0, sr, sc)]

        found = False
        expansions = 0
        while heap:
            _, g, r, c = heapq.heappop(heap)
            if closed[r, c]:
                continue
            closed[r, c] = True

            if (r, c) == (gr, gc):
                found = True
                break

            expansions += 1
            if expansions >= self.ASTAR_MAX_EXPANSIONS:
                break

            for dr, dc, step_cost in neighbors:
                nr, nc = r + dr, c + dc
                if nr < 0 or nr >= h or nc < 0 or nc >= w:
                    continue
                if blocked[nr, nc] or closed[nr, nc]:
                    continue

                candidate_g = g + step_cost
                if candidate_g < g_score[nr, nc]:
                    g_score[nr, nc] = candidate_g
                    parent[(nr, nc)] = (r, c)
                    f = candidate_g + heuristic(nr, nc)
                    heapq.heappush(heap, (f, candidate_g, nr, nc))

        if not found:
            return None

        path = [(gr, gc)]
        cur = (gr, gc)
        while cur != (sr, sc):
            cur = parent.get(cur)
            if cur is None:
                return None
            path.append(cur)
        path.reverse()
        return path

    def _astar_lookahead_goal(self, goal_x, goal_y, allow_unknown=True):
        """Return short-horizon world waypoint along an A* path to goal."""
        if self._map_data is None:
            return None

        mpose = self.get_map_pose()
        if mpose is None:
            return None

        start_rc = self._world_to_grid(mpose[0], mpose[1])
        goal_rc = self._world_to_grid(goal_x, goal_y)
        if start_rc is None or goal_rc is None:
            return None

        path = self._astar_path(start_rc, goal_rc, allow_unknown=allow_unknown)
        if not path:
            return None
        if len(path) == 1:
            return (goal_x, goal_y)

        lookahead = max(0.4, self.ASTAR_LOOKAHEAD_M)
        res = self._map_data.info.resolution
        accumulated = 0.0
        chosen = path[-1]
        for i in range(1, len(path)):
            pr, pc = path[i - 1]
            cr, cc = path[i]
            accumulated += math.hypot(cr - pr, cc - pc) * res
            if accumulated >= lookahead:
                chosen = (cr, cc)
                break

        wp = self._grid_to_world(chosen[0], chosen[1])
        if wp is None:
            return None
        return wp

    def _clear_costmaps_throttled(self):
        now = self.now_sec()
        if (now - self._last_costmap_clear_time) < self.COSTMAP_CLEAR_COOLDOWN:
            return
        self._last_costmap_clear_time = now
        self.clear_costmaps()

    def _next_door_sweep_goal(self):
        if not self.DOOR_SWEEP_GOALS:
            return None

        mpose = self.get_map_pose()
        if mpose is None:
            return None
        mx, my = mpose[0], mpose[1]

        # Compensate for SLAM/corridor lateral drift so fixed sweep goals remain reachable.
        if abs(my) <= 1.6:
            y_shift = max(-self.DOOR_SWEEP_CORRIDOR_ADAPT,
                          min(self.DOOR_SWEEP_CORRIDOR_ADAPT, my))
        else:
            y_shift = 0.0

        now = self.now_sec()
        total = len(self.DOOR_SWEEP_GOALS)
        for offset in range(total):
            idx = (self._door_sweep_idx + offset) % total
            gx, gy_base, should_scan = self.DOOR_SWEEP_GOALS[idx]
            if abs(gy_base) <= 1.2:
                gy = gy_base + y_shift
            else:
                gy = gy_base

            # Hard safety envelope: don't command deep room goals that can
            # appear to push through walls when localization is imperfect.
            if abs(gy) > self.DOOR_SWEEP_MAX_ABS_Y:
                continue

            cool_until = self._door_failed_until.get(idx, 0.0)
            if now < cool_until:
                continue

            if math.hypot(gx - mx, gy - my) < 0.55:
                if offset == 0:
                    self._door_sweep_idx = (idx + 1) % total
                continue

            # Allow partially-unknown room interior while still rejecting occupied cells.
            if self._is_goal_navigable(gx, gy, allow_unknown=True):
                return (idx, gx, gy, should_scan)

        # Fallback corridor anchors with scan disabled.
        anchors = [(2.0, 0.0), (4.0, 0.0), (6.5, 0.0), (8.5, 0.0), (10.0, 0.0)]
        anchors.sort(key=lambda pt: math.hypot(pt[0] - mx, pt[1] - my))
        for gx, gy in anchors:
            if self._is_goal_navigable(gx, gy, allow_unknown=False):
                return (None, gx, gy, False)
        return None

    def _run_door_sweep(self):
        """Deterministic room search: corridor -> door -> room point -> 360 scan."""
        mpose = self.get_map_pose()
        now = self.now_sec()

        # Goal reached
        if self.nav2_reached:
            self.nav2_reached = False
            self._frontier_fail_cnt = 0
            reached_goal = self._door_sweep_goal
            reached_idx = self._door_sweep_goal_idx
            self._door_sweep_goal = None
            self._door_sweep_goal_idx = None
            self._door_goal_attempts = 0

            if reached_idx is not None:
                if self.FORCE_LEFT_ROOM_SEQUENCE and \
                        reached_idx >= self.SECOND_LEFT_ROOM_GOAL_IDX and \
                        (not self.sphere_confirmed) and (not self._sphere_reached):
                    self._door_sweep_idx = self.SECOND_LEFT_ROOM_GOAL_IDX
                else:
                    self._door_sweep_idx = (reached_idx + 1) % len(self.DOOR_SWEEP_GOALS)

            if self._door_scan_pending:
                self._door_scan_pending = False
                if reached_goal is not None:
                    self.get_logger().info(
                        f'🛏️ Room checkpoint reached at ({reached_goal[0]:.1f}, {reached_goal[1]:.1f}) – scanning room')
                self.begin_scan()
                return

        # Goal failed
        if self.nav2_failed:
            self.nav2_failed = False
            self._door_stall_ref_pose = None
            self._door_stall_start_time = 0.0
            if self._door_sweep_goal is not None and self._door_sweep_goal_idx is not None:
                self._last_failed_goal = self._door_sweep_goal
                self._last_failed_goal_time = self.now_sec()

                if self._door_goal_attempts < self.DOOR_SWEEP_MAX_RETRIES:
                    self._door_goal_attempts += 1
                    gx, gy = self._door_sweep_goal
                    self._clear_costmaps_throttled()
                    self.get_logger().warn(
                        f'🔁 Door goal retry {self._door_goal_attempts}/{self.DOOR_SWEEP_MAX_RETRIES} '
                        f'for ({gx:.1f}, {gy:.1f})')
                    self.send_nav2_goal(gx, gy)
                    return

                now = self.now_sec()
                if self.FORCE_LEFT_ROOM_SEQUENCE and \
                        self._door_sweep_goal_idx >= self.SECOND_LEFT_ROOM_GOAL_IDX:
                    self._door_failed_until[self._door_sweep_goal_idx] = 0.0
                    self._door_sweep_idx = self.SECOND_LEFT_ROOM_GOAL_IDX
                else:
                    self._door_failed_until[self._door_sweep_goal_idx] = now + self.DOOR_SWEEP_RETRY_COOLDOWN
                    self._door_sweep_idx = (self._door_sweep_goal_idx + 1) % len(self.DOOR_SWEEP_GOALS)

            self._door_sweep_goal = None
            self._door_sweep_goal_idx = None
            scan_after_fail = self._door_scan_pending
            self._door_scan_pending = False
            self._door_goal_attempts = 0
            self._frontier_fail_cnt += 1
            self._clear_costmaps_throttled()

            if scan_after_fail:
                self.get_logger().warn(
                    '🔄 Door room goal failed – scanning at current pose before continuing')
                self.begin_scan()
                return

            self._publish_active_search_cmd()
            return

        # Active goal still running
        if self.nav2_goal_active:
            if self._door_sweep_goal is not None and mpose is not None:
                dist = math.hypot(self._door_sweep_goal[0] - mpose[0],
                                  self._door_sweep_goal[1] - mpose[1])
                if dist <= self.DOOR_SWEEP_REACHED_DIST:
                    self.get_logger().info(
                        f'✅ Door goal close-enough ({dist:.2f}m) – advancing sweep')
                    self.cancel_nav2_goal()
                    self.nav2_reached = True
                    return

                if self._door_stall_ref_pose is None:
                    self._door_stall_ref_pose = (mpose[0], mpose[1])
                    self._door_stall_start_time = now
                else:
                    moved = math.hypot(mpose[0] - self._door_stall_ref_pose[0],
                                       mpose[1] - self._door_stall_ref_pose[1])
                    if moved >= self.DOOR_SWEEP_STALL_DIST:
                        self._door_stall_ref_pose = (mpose[0], mpose[1])
                        self._door_stall_start_time = now
                    elif (now - self._door_stall_start_time) > self.DOOR_SWEEP_STALL_TIME:
                        self.get_logger().warn(
                            f'🧱 Door-sweep stall: moved {moved:.2f}m in '
                            f'{(now - self._door_stall_start_time):.0f}s – unstick recovery')
                        self.cancel_nav2_goal()
                        self._clear_costmaps_throttled()
                        self._door_recovery_until = now + self.DOOR_SWEEP_RECOVERY_COOLDOWN
                        self._door_stall_ref_pose = None
                        self._door_stall_start_time = 0.0
                        self._start_unstick('door-sweep stall')
                        self.nav2_failed = True
                        return

            elapsed = self.now_sec() - self._nav2_goal_time
            if elapsed > self.DOOR_SWEEP_GOAL_TIMEOUT:
                self.get_logger().warn(
                    f'⏰ Door-sweep timeout ({elapsed:.0f}s) – cancel and replan')
                self.cancel_nav2_goal()
                self.nav2_failed = True
            return

        if now < self._door_recovery_until:
            self._publish_active_search_cmd()
            return

        if mpose is not None and abs(mpose[1]) > self.ROOM_TRAP_Y:
            # Hard escape preference: nearest known doorway mouth first.
            room_sign = 1.0 if mpose[1] >= 0.0 else -1.0
            doorway_candidates = [
                (4.0, room_sign * 1.2),
                (10.0, room_sign * 1.2),
            ]
            doorway_candidates.sort(key=lambda pt: abs(pt[0] - mpose[0]))
            for ex, ey in doorway_candidates:
                if math.hypot(ex - mpose[0], ey - mpose[1]) < 0.8:
                    continue
                if self._is_goal_navigable(ex, ey, allow_unknown=True):
                    self._door_sweep_goal = (ex, ey)
                    self._door_sweep_goal_idx = None
                    self._door_scan_pending = False
                    self._door_goal_attempts = 1
                    self.get_logger().warn(
                        f'🚪 Doorway escape goal: ({ex:.1f}, {ey:.1f})')
                    self.send_nav2_goal(ex, ey)
                    return

            esc = self._corridor_escape_goal()
            if esc is not None:
                ex, ey = esc
                self._door_sweep_goal = (ex, ey)
                self._door_sweep_goal_idx = None
                self._door_scan_pending = False
                self._door_goal_attempts = 1
                self.get_logger().warn(
                    f'🚪 Door-sweep escape goal: ({ex:.1f}, {ey:.1f})')
                self.send_nav2_goal(ex, ey)
                return

        # No active goal -> pick next sweep target
        goal = self._next_door_sweep_goal()
        if goal is None:
            self._clear_costmaps_throttled()
            self._publish_active_search_cmd()
            return

        idx, gx, gy, should_scan = goal
        self._door_sweep_goal = (gx, gy)
        self._door_sweep_goal_idx = idx
        self._door_scan_pending = should_scan
        self._door_goal_attempts = 1
        self.get_logger().info(
            f'🚪 Door-sweep goal: ({gx:.1f}, {gy:.1f})' +
            (' [scan]' if self._door_scan_pending else ''))
        self.send_nav2_goal(gx, gy)

    # ── Frontier detection ────────────────────────────────────
    def get_best_frontier(self):
        """
        Find a robust frontier target in the SLAM occupancy grid.
        Frontier = a free cell (value 0) adjacent to an unknown cell (value -1).
        Uses connected-component clustering to avoid noisy single-cell goals,
        applies obstacle clearance, and avoids recently failed frontiers.
        Returns (world_x, world_y) or None.
        """
        if self._map_data is None:
            return None

        info = self._map_data.info
        w, h = info.width, info.height
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        # Reshape flat array -> 2-D grid
        data = np.array(self._map_data.data, dtype=np.int8).reshape(h, w)

        free = (data == 0)       # navigable
        unknown = (data == -1)   # not yet mapped
        occupied = (data > 50)   # occupied in occupancy grid

        # Add obstacle clearance margin so chosen frontier goals are reachable
        clearance_cells = max(1, int(self.FRONTIER_CLEARANCE_M / max(res, 1e-6)))
        kernel_size = 2 * clearance_cells + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        occupied_inflated = cv2.dilate(occupied.astype(np.uint8), kernel, iterations=1) > 0

        # Frontier: free cell with ≥1 unknown neighbour (4-connectivity)
        frontier = np.zeros_like(free, dtype=bool)
        frontier[1:-1, 1:-1] = (
            free[1:-1, 1:-1] & (
                unknown[:-2, 1:-1] | unknown[2:, 1:-1] |   # N / S
                unknown[1:-1, :-2] | unknown[1:-1, 2:]     # W / E
            )
        )
        frontier &= ~occupied_inflated

        if not np.any(frontier):
            return None

        # Robot position in map grid coordinates
        mpose = self.get_map_pose()
        if mpose is None:
            return None
        rx, ry, _ = mpose
        robot_col = int((rx - ox) / res)
        robot_row = int((ry - oy) / res)
        if robot_col < 0 or robot_col >= w or robot_row < 0 or robot_row >= h:
            return None

        frontier_u8 = frontier.astype(np.uint8)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_u8, connectivity=8)

        def pick_candidate(ignore_blocklist=False, relax_max_dist=False,
                           min_cluster_cells=None):
            if min_cluster_cells is None:
                min_cluster_cells = self.FRONTIER_MIN_CLUSTER_CELLS
            candidates = []
            for label_id in range(1, num_labels):
                cluster_size = int(stats[label_id, cv2.CC_STAT_AREA])
                if cluster_size < min_cluster_cells:
                    continue

                center_col = centroids[label_id][0]
                center_row = centroids[label_id][1]
                target_col = int(round(center_col))
                target_row = int(round(center_row))

                if target_col < 0 or target_col >= w or target_row < 0 or target_row >= h:
                    continue

                if not frontier[target_row, target_col]:
                    rows, cols = np.where(labels == label_id)
                    if len(rows) == 0:
                        continue
                    nearest_idx = int(np.argmin((rows - center_row) ** 2 +
                                                (cols - center_col) ** 2))
                    target_row = int(rows[nearest_idx])
                    target_col = int(cols[nearest_idx])

                dist_m = math.hypot(target_row - robot_row,
                                    target_col - robot_col) * res
                if dist_m < self.FRONTIER_MIN_DIST:
                    continue
                if (not relax_max_dist) and dist_m > self.FRONTIER_MAX_DIST:
                    continue

                world_x = ox + (target_col + 0.5) * res
                world_y = oy + (target_row + 0.5) * res

                if abs(world_y) > self.EXPLORE_MAX_ABS_Y:
                    continue

                if (not ignore_blocklist) and self._is_frontier_blocked(world_x, world_y):
                    continue

                # Bias exploration toward the mission's forward direction
                # (along start heading), so the robot doesn't keep backtracking.
                forward_penalty = 0.0
                if self.start_pose is not None:
                    sx, sy, syaw = self.start_pose
                    fx = world_x - sx
                    fy = world_y - sy
                    forward_m = fx * math.cos(syaw) + fy * math.sin(syaw)
                    if forward_m < self.FRONTIER_MIN_FORWARD_M:
                        continue
                    if forward_m < 0.0:
                        forward_penalty = abs(forward_m) * self.FRONTIER_FORWARD_BIAS

                # Score: prefer closer frontiers, but slightly prefer larger clusters.
                score = dist_m - 0.02 * float(cluster_size) + forward_penalty
                candidates.append((score, world_x, world_y))

            if not candidates:
                return None
            candidates.sort(key=lambda item: item[0])
            _, best_x, best_y = candidates[0]
            return (best_x, best_y)

        # Try strict first; then gradually relax constraints.
        frontier_goal = pick_candidate(ignore_blocklist=False, relax_max_dist=False)
        if frontier_goal is None:
            frontier_goal = pick_candidate(ignore_blocklist=False, relax_max_dist=True)
        if frontier_goal is None:
            frontier_goal = pick_candidate(ignore_blocklist=True, relax_max_dist=True)
        # Early-map fallback: allow tiny clusters to bootstrap exploration.
        if frontier_goal is None:
            frontier_goal = pick_candidate(ignore_blocklist=False,
                                           relax_max_dist=True,
                                           min_cluster_cells=1)
        return frontier_goal

    # ── Image callback ───────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        h_img, w_img = frame.shape[:2]
        self._img_count += 1

        # HSV red detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo1, hi1 = np.array([0, 35, 25]),   np.array([10, 255, 255])
        lo2, hi2 = np.array([160, 35, 25]),  np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)

        if self._img_count % 50 == 1:
            rpx = int(np.count_nonzero(mask))
            mbgr = frame.mean(axis=(0, 1))
            self.get_logger().info(
                f'📷 Frame #{self._img_count}: {w_img}x{h_img}, '
                f'mean_BGR=({mbgr[0]:.0f},{mbgr[1]:.0f},'
                f'{mbgr[2]:.0f}), red_px={rpx}, state={self.state}')

        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kern, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.blob_detected = False
        self.blob_cx       = None
        self.blob_area     = 0
        est_dist           = float('inf')

        if contours:
            biggest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest)
            if area > self.MIN_DETECT_AREA:
                x, y, w, h = cv2.boundingRect(biggest)
                cx = x + w // 2
                edge_margin_px = max(16, int(0.05 * w_img))
                near_edge = (cx < edge_margin_px) or \
                    (cx > (w_img - edge_margin_px))

                (mcx, mcy), mrad = cv2.minEnclosingCircle(biggest)
                est_dist = self._estimate_distance(2.0 * mrad)
                plausible_range = est_dist < 24.0
                edge_ok = (not near_edge) or (area >= 220.0)

                if edge_ok and plausible_range:
                    self.blob_detected = True
                    self.blob_cx = cx
                    self.blob_area = area
                    self._est_sphere_dist = est_dist

                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (int(mcx), int(mcy)), int(mrad),
                           (0, 255, 255), 2)
                cv2.drawMarker(frame, (cx, y + h//2), (0, 0, 255),
                               cv2.MARKER_CROSS, 20, 2)
                cv2.putText(frame,
                    f'SPHERE d={est_dist:.1f}m area={area:.0f}px',
                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.50, (0, 255, 0), 2)

        # Confirmations during SCAN_360 or EXPLORE
        if self.state in ('SCAN_360', 'EXPLORE'):
            if self.blob_detected and self.odom_received:
                self.confirm_count += 1
                if self.confirm_count >= self.CONFIRM_FRAMES:
                    self.sphere_confirmed = True
                    self._target_locked = True
                    self.get_logger().info(
                        f'🎯 Sphere FOUND! area={self.blob_area:.0f}px, '
                        f'cx={self.blob_cx}, d~{est_dist:.1f}m '
                        f'(frontier #{self._frontier_count}, state={self.state})')
            else:
                self.confirm_count = 0

        # HUD overlay
        state_colors = {
            'INIT':     (128, 128, 128),
            'EXPLORE':  (0, 200, 200),
            'SCAN_360': (0, 165, 255),
            'NAVIGATE': (255, 200, 0),
            'WAIT':     (0, 200, 255),
            'RETURN':   (255, 100, 255),
            'DONE':     (0, 255, 0),
        }
        col = state_colors.get(self.state, (255, 255, 255))
        cv2.putText(frame, f'STATE: {self.state}', (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

        # Return banner
        if self.state == 'RETURN':
            cv2.rectangle(frame, (0, h_img - 50), (w_img, h_img),
                          (0, 0, 0), -1)
            cv2.putText(frame,
                        '<< RETURNING HOME (Nav2  ) >>',
                        (w_img // 2 - 200, h_img - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 200, 255), 2)

        mpose = self.get_map_pose()
        if mpose:
            info = [f'Map: ({mpose[0]:.1f}, {mpose[1]:.1f})']
        else:
            info = [f'Odom: ({self.robot_x:.1f}, {self.robot_y:.1f})']
        if self.state == 'EXPLORE':
            info.append(f'Frontier #{self._frontier_count + 1}')
            if self._frontier_goal:
                fx, fy = self._frontier_goal
                info.append(f'Target: ({fx:.1f}, {fy:.1f})')
            if self.nav2_goal_active:
                info.append('Nav2: navigating...')
            elif self._frontier_goal is None:
                info.append('Scanning map...')
        elif self.state == 'SCAN_360' and self.scan_start_time:
            elapsed = self.now_sec() - self.scan_start_time
            deg = math.degrees(self.scan_accumulated)
            info.append(f'Scanning... {deg:.0f}/360° ({elapsed:.0f}s)')
            info.append(f'confirms={self.confirm_count}/'
                        f'{self.CONFIRM_FRAMES}')
        elif self.state == 'NAVIGATE':
            info.append(f'Servo area={self.blob_area:.0f}px')
            info.append(f'LiDAR front: {self.front_min_range:.2f}m')
        elif self.state == 'WAIT' and self.wait_start_time:
            rem = max(0, self.WAIT_SECONDS -
                      (self.now_sec() - self.wait_start_time))
            info.append(f'Waiting... {rem:.1f}s left')
        elif self.state == 'RETURN':
            mp = self.get_map_pose()
            if mp:
                ddx = self.start_pose[0] - mp[0]
                ddy = self.start_pose[1] - mp[1]
                md = math.sqrt(ddx*ddx + ddy*ddy)
                info.append(f'{self._return_phase}: {md:.1f}m to home')
                info.append(
                    f'  retry {self._return_retry_count}/{self.RETURN_MAX_RETRIES}')
            else:
                info.append(f'{self._return_phase}...')
        elif self.state == 'DONE':
            info.append('Mission complete!')

        for i, line in enumerate(info):
            cv2.putText(frame, line, (20, 60 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1)

        cv2.line(frame, (w_img // 2, 0), (w_img // 2, h_img),
                 (100, 100, 100), 1)

        try:
            self.result_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception:
            pass

    # ── Begin 360 scan ───────────────────────────────────────
    def begin_scan(self):
        self.state            = 'SCAN_360'
        self.scan_start_time  = self.now_sec()
        self.scan_last_yaw    = self.robot_yaw if self.odom_received else None
        self.scan_accumulated = 0.0
        self.sphere_confirmed = False
        self.confirm_count    = 0
        self._next_explore_scan = self.scan_start_time + self.EXPLORE_SCAN_INTERVAL

        mpose = self.get_map_pose()
        if mpose:
            self._last_scan_map_pose = (mpose[0], mpose[1])
        mx, my = (mpose[0], mpose[1]) if mpose else (0.0, 0.0)
        self.get_logger().info(
            f'🔍 360 scan at map '
            f'({mx:.1f}, {my:.1f}), frontier #{self._frontier_count}')

    def _start_return_home(self, reason=''):
        """Begin return-home sequence toward the original start pose."""
        if self.start_pose is None:
            self.get_logger().warn('⚠️ Cannot RETURN: start pose is unavailable.')
            return

        self._mission_complete = False
        self.state = 'RETURN'
        self._return_phase = 'NAV2'
        self._return_start_time = self.now_sec()
        self._return_nav2_sent = False
        self._return_retry_count = 0
        self._return_retry_after = self.now_sec()
        self._home_near_since = None
        self._sphere_reached = False

        sx, sy, _ = self.start_pose
        mpose = self.get_map_pose()
        if mpose:
            mx, my, _ = mpose
            map_dist = math.hypot(sx - mx, sy - my)
            if reason:
                self.get_logger().info(
                    f'🔙 {reason} -> RETURN home. map_dist={map_dist:.1f}m')
            else:
                self.get_logger().info(
                    f'🔙 RETURN home started. map_dist={map_dist:.1f}m')
        else:
            if reason:
                self.get_logger().info(f'🔙 {reason} -> RETURN home.')
            else:
                self.get_logger().info('🔙 RETURN home started.')

        self.clear_costmaps()

    # ── State machine (10 Hz) ────────────────────────────────
    def state_machine_tick(self):

        if self._mission_complete:
            self.cancel_nav2_goal()
            self.state = 'DONE'
            self.stop_robot()
            return

        # ── Publish prediction path (to sphere during NAVIGATE, to home during RETURN) ──
        if self.state == 'NAVIGATE' and self.blob_detected and \
                self.blob_cx is not None and self._est_sphere_dist < 50.0:
            mpose = self.get_map_pose()
            if mpose:
                mx, my, myaw = mpose
                half_w = self.IMAGE_WIDTH / 2.0
                ang_off = ((half_w - self.blob_cx) / half_w) * \
                          (self.CAMERA_HFOV_RAD / 2.0)
                sphere_yaw = myaw + ang_off
                sx = mx + self._est_sphere_dist * math.cos(sphere_yaw)
                sy = my + self._est_sphere_dist * math.sin(sphere_yaw)
                now_stamp = self.get_clock().now().to_msg()
                p1 = PoseStamped()
                p1.header.frame_id = 'map'
                p1.header.stamp = now_stamp
                p1.pose.position.x = mx
                p1.pose.position.y = my
                p1.pose.orientation = self.yaw_to_quat(myaw)
                p2 = PoseStamped()
                p2.header.frame_id = 'map'
                p2.header.stamp = now_stamp
                p2.pose.position.x = sx
                p2.pose.position.y = sy
                p2.pose.orientation = self.yaw_to_quat(sphere_yaw)
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = now_stamp
                path_msg.poses = [p1, p2]
                self.pred_path_pub.publish(path_msg)
        elif self.state == 'RETURN' and self.start_pose is not None:
            mpose = self.get_map_pose()
            if mpose:
                mx, my, myaw = mpose
                hx, hy, hyaw = self.start_pose
                now_stamp = self.get_clock().now().to_msg()
                p1 = PoseStamped()
                p1.header.frame_id = 'map'
                p1.header.stamp = now_stamp
                p1.pose.position.x = mx
                p1.pose.position.y = my
                p1.pose.orientation = self.yaw_to_quat(myaw)
                p2 = PoseStamped()
                p2.header.frame_id = 'map'
                p2.header.stamp = now_stamp
                p2.pose.position.x = hx
                p2.pose.position.y = hy
                p2.pose.orientation = self.yaw_to_quat(hyaw)
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = now_stamp
                path_msg.poses = [p1, p2]
                self.pred_path_pub.publish(path_msg)
        elif self.state not in ('NAVIGATE', 'RETURN'):
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = []
            self.pred_path_pub.publish(path_msg)

        # ─── INIT ───
        if self.state == 'INIT':
            if self.init_start is None:
                self.init_start = self.now_sec()
            if not self.odom_received:
                if not self.init_logged and \
                        self.now_sec() - self.init_start > 3.0:
                    self.get_logger().info('⏳ Waiting for odometry...')
                    self.init_logged = True
                return

            # Check Nav2 is available
            if not self._nav2_ready:
                if self.nav_client.wait_for_server(timeout_sec=0.1):
                    self._nav2_ready = True
                    self.get_logger().info('✅ Nav2 action server ready!')
                else:
                    if self.now_sec() - self.init_start > 5.0:
                        self.get_logger().info(
                            '⏳ Waiting for Nav2 action server...')
                    return

            # Look up start position in MAP frame via TF
            if self.start_pose is None:
                pose = self.get_map_pose()
                if pose is None:
                    if self.now_sec() - self.init_start > 8.0:
                        self.get_logger().info(
                            '⏳ Waiting for map→base_link TF...')
                    return
                self.start_pose = pose
                sx, sy, syaw = pose
                self.get_logger().info(
                    f'📍 Start pose (map frame): '
                    f'({sx:.2f}, {sy:.2f}, yaw={syaw:.2f})')
                self._build_probe_goals()
                self._reset_patrol_index_from_pose(pose)
                self.get_logger().info(
                    f'🧭 Built {len(self._probe_goals)}   probe goals')

            self.get_logger().info(
                '✅ Odom + Nav2 + TF ready – frontier exploration starts!')
            self._next_explore_scan = self.now_sec() + self.EXPLORE_SCAN_INTERVAL
            self._explore_start_time = self.now_sec()
            if self.PATROL_START_IMMEDIATELY:
                self._patrol_mode = True
                self._force_probe_mode = False
                self.get_logger().info(
                    '🧭 Deterministic   patrol mode enabled')
            self.state = 'EXPLORE'
            return

        # ─── EXPLORE (autonomous frontier exploration) ───
        if self.state == 'EXPLORE':
            # Unstick micro-sequence (executed before normal explore logic)
            if self._unstick_phase is not None:
                now = self.now_sec()
                if now >= self._unstick_phase_until:
                    if self._unstick_phase == 'TURN':
                        self._unstick_phase = 'BACKUP'
                        self._unstick_phase_until = now + self.UNSTICK_BACKUP_S
                    elif self._unstick_phase == 'BACKUP':
                        self._unstick_phase = 'FORWARD'
                        self._unstick_phase_until = now + self.UNSTICK_FORWARD_S
                    else:
                        self._unstick_phase = None
                        self.stop_robot()
                        self._stall_ref_pose = None
                        self._stall_start_time = 0.0
                        self._idle_stall_ref_pose = None
                        self._idle_stall_start_time = 0.0
                        self._force_probe_mode = True
                        return

                cmd = Twist()
                if self._unstick_phase == 'TURN':
                    cmd.angular.z = self.SEARCH_YAW_SPEED * 1.25 * self._unstick_turn_dir
                elif self._unstick_phase == 'BACKUP':
                    if self.rear_min_range > (self.SAFE_DISTANCE + 0.05):
                        cmd.linear.x = -self.UNSTICK_BACKUP_SPEED
                        cmd.angular.z = -self.UNSTICK_ARC_YAW * self._unstick_turn_dir
                    else:
                        cmd.angular.z = self.SEARCH_YAW_SPEED * 1.1 * self._unstick_turn_dir
                elif self._unstick_phase == 'FORWARD':
                    # Move out of local trap if front is clear, else keep turning.
                    if self.front_min_range > (self.SAFE_DISTANCE + 0.15):
                        cmd.linear.x = self.UNSTICK_FORWARD_SPEED
                        cmd.angular.z = self.UNSTICK_ARC_YAW * self._unstick_turn_dir
                    else:
                        cmd.angular.z = self.SEARCH_YAW_SPEED * 1.05 * self._unstick_turn_dir
                self.cmd_pub.publish(self._apply_safety_to_cmd(cmd))
                return

            # Sphere spotted at any time → drop everything and servo to it
            if self.sphere_confirmed:
                if self.STOP_ON_FIRST_SUCCESS:
                    self._complete_mission('sphere detected first')
                else:
                    self.cancel_nav2_goal()
                    self.stop_robot()
                    self.state = 'NAVIGATE'
                    self.servo_lost_time = None
                    self.get_logger().info(
                        '🧭 Sphere spotted while exploring → NAVIGATE')
                return

            if self.DOOR_SWEEP_ENABLE:
                self._run_door_sweep()
                return

            mpose = self.get_map_pose()
            if mpose is not None:
                mx, my, _ = mpose
                if mx < self.MAP_SAFE_X_MIN or mx > self.MAP_SAFE_X_MAX or abs(my) > self.MAP_SAFE_Y_MAX:
                    self._escape_to_corridor = True
                    self._force_probe_mode = True
                    self._patrol_mode = False
                    self._probe_roam_until = 0.0
                if abs(my) > self.ROOM_TRAP_Y and self._frontier_fail_cnt >= 2:
                    self._escape_to_corridor = True

            # No-goal freeze watchdog: if robot is in EXPLORE but not moving,
            # force an unstick micro-recovery instead of waiting indefinitely.
            now = self.now_sec()
            if (not self.nav2_goal_active) and (self._unstick_phase is None) and (mpose is not None):
                if self._idle_stall_ref_pose is None:
                    self._idle_stall_ref_pose = (mpose[0], mpose[1])
                    self._idle_stall_start_time = now
                else:
                    moved_idle = math.hypot(
                        mpose[0] - self._idle_stall_ref_pose[0],
                        mpose[1] - self._idle_stall_ref_pose[1])
                    if moved_idle >= self.IDLE_STALL_DIST_THRESH:
                        self._idle_stall_ref_pose = (mpose[0], mpose[1])
                        self._idle_stall_start_time = now
                    elif (now - self._idle_stall_start_time) > self.IDLE_STALL_TIME_THRESH:
                        self.get_logger().warn(
                            f'🧱 Explore idle-stall: moved {moved_idle:.2f}m in '
                            f'{(now - self._idle_stall_start_time):.0f}s – forcing unstick')
                        self._start_unstick('idle stall in EXPLORE')
                        return
            else:
                self._idle_stall_ref_pose = None
                self._idle_stall_start_time = 0.0

            # Honor local-roam fallback window before sending new Nav2 goals.
            if self.now_sec() < self._probe_roam_until:
                self._publish_active_search_cmd()
                return

            if self._next_explore_scan > 0.0 and \
                    self.now_sec() >= self._next_explore_scan and \
                    (not self.nav2_goal_active):
                do_scan = True
                mpose_for_scan = self.get_map_pose()
                if mpose_for_scan is not None and self._last_scan_map_pose is not None:
                    moved = math.hypot(
                        mpose_for_scan[0] - self._last_scan_map_pose[0],
                        mpose_for_scan[1] - self._last_scan_map_pose[1])
                    if moved < self.SCAN_MIN_MOVE_M:
                        do_scan = False

                if do_scan:
                    self.begin_scan()
                else:
                    self._next_explore_scan = self.now_sec() + self.EXPLORE_SCAN_INTERVAL
                return

            if (not self._patrol_mode) and (not self._force_probe_mode) and \
                    self._explore_start_time > 0.0 and (not self.DYNAMIC_ONLY_MODE):
                explore_elapsed = self.now_sec() - self._explore_start_time
                if explore_elapsed > self.PATROL_ENABLE_AFTER or \
                        self._frontier_fail_cnt >= self.PATROL_FAIL_TRIGGER:
                    self._patrol_mode = True
                    self._force_probe_mode = False
                    self._probe_roam_until = 0.0
                    self._frontier_goal = None
                    self.cancel_nav2_goal()
                    self.get_logger().warn(
                        '🧭 Enabling deterministic   patrol sweep for reliable search')
                    return

            # Nav2 reached frontier → do a 360 scan then pick next frontier
            if self.nav2_reached:
                if self.DYNAMIC_EXPLORE_ENABLE and self._frontier_goal is not None:
                    mpose_step = self.get_map_pose()
                    if mpose_step is not None:
                        fx, fy = self._frontier_goal
                        frontier_dist = math.hypot(fx - mpose_step[0], fy - mpose_step[1])
                        sent_xy = self._nav2_goal_sent_xy
                        sent_is_intermediate = (
                            sent_xy is not None and
                            math.hypot(sent_xy[0] - fx, sent_xy[1] - fy) > 0.20
                        )
                        if sent_is_intermediate and frontier_dist > self.FRONTIER_REACHED_DIST:
                            self.nav2_reached = False
                            self.get_logger().info(
                                f'➡️ A* step reached, {frontier_dist:.2f}m to frontier; '
                                'sending next dynamic step')
                            self.send_nav2_goal(fx, fy)
                            return

                self.nav2_reached = False
                self._last_failed_goal = None
                self._last_failed_goal_time = 0.0
                if self._escape_to_corridor:
                    self._escape_to_corridor = False
                    self._frontier_goal = None
                    self.get_logger().info(
                        '✅ Corridor escape succeeded – resuming exploration')
                self._frontier_count += 1
                self._frontier_fail_cnt = 0
                self._force_probe_mode = False
                if self._frontier_goal:
                    fx, fy = self._frontier_goal
                    self.get_logger().info(
                        f'📍 Frontier #{self._frontier_count} reached '
                        f'({fx:.1f}, {fy:.1f})')
                self._frontier_goal = None
                # Sphere visible right here? go immediately
                if self.blob_detected:
                    self.sphere_confirmed = True
                    self._target_locked = True
                    self.get_logger().info(
                        f'👁️  Sphere visible at frontier '
                        f'(area={self.blob_area:.0f}px) → NAVIGATE')
                    self.state = 'NAVIGATE'
                    self.servo_lost_time = None
                    return
                # Otherwise scan 360° at this spot
                do_scan = True
                if mpose is not None and self._last_scan_map_pose is not None:
                    moved = math.hypot(
                        mpose[0] - self._last_scan_map_pose[0],
                        mpose[1] - self._last_scan_map_pose[1])
                    if moved < self.SCAN_MIN_MOVE_M:
                        do_scan = False

                if do_scan:
                    self.begin_scan()
                else:
                    self._next_explore_scan = self.now_sec() + min(12.0, self.EXPLORE_SCAN_INTERVAL)
                return

            # Nav2 failed → abandon this frontier, pick another
            if self.nav2_failed:
                self.nav2_failed = False
                if self._frontier_goal is not None:
                    self._last_failed_goal = self._frontier_goal
                    self._last_failed_goal_time = self.now_sec()
                self._mark_current_frontier_blocked()
                self._frontier_goal = None
                self._frontier_fail_cnt += 1
                if mpose is not None and abs(mpose[1]) > self.ROOM_TRAP_Y:
                    self._escape_to_corridor = True

                if self._patrol_mode:
                    self.get_logger().warn(
                        f'⚠️  Patrol Nav2 failed '
                        f'(consecutive={self._frontier_fail_cnt})')
                    if self._frontier_fail_cnt >= self.PATROL_FAIL_ESCAPE_TRIGGER:
                        self._patrol_mode = False
                        self._force_probe_mode = True
                        self._probe_roam_until = 0.0
                        self._escape_to_corridor = True
                        self.clear_costmaps()
                        self.get_logger().warn(
                            '♻️ Repeated patrol failures – switching to   probe recovery')
                    return

                self.get_logger().warn(
                    f'⚠️  Nav2 failed for frontier '
                    f'(consecutive failures: {self._frontier_fail_cnt})')
                if self._frontier_fail_cnt >= self.FRONTIER_MAX_FAILS:
                    self._frontier_fail_cnt = 0
                    self.clear_costmaps()

                    if self._force_probe_mode:
                        self._force_probe_mode = False
                        self._probe_roam_until = self.now_sec() + self.PROBE_ROAM_SECONDS
                        self._no_frontier_since = self.now_sec()
                        self.get_logger().warn(
                            f'♻️ Probe recovery still failing – switching to '
                            f'local roam for {self.PROBE_ROAM_SECONDS:.0f}s')
                        self._publish_active_search_cmd()
                    else:
                        self._force_probe_mode = True
                        self.get_logger().warn(
                            '♻️ Too many Nav2 failures – enabling   probe recovery')
                    return
                return  # next tick will pick a new frontier

            # Nav2 still running → check per-frontier timeout
            if self.nav2_goal_active:
                mpose = self.get_map_pose()
                if self._frontier_goal is not None and mpose is not None:
                    fg_dist = math.hypot(self._frontier_goal[0] - mpose[0],
                                         self._frontier_goal[1] - mpose[1])
                    if fg_dist <= self.FRONTIER_REACHED_DIST:
                        self.get_logger().info(
                            f'✅ Frontier close-enough ({fg_dist:.2f}m) – treating as reached')
                        self.cancel_nav2_goal()
                        self.nav2_reached = True
                        return

                if mpose is not None:
                    mx, my, _ = mpose
                    now = self.now_sec()
                    if self._stall_ref_pose is None:
                        self._stall_ref_pose = (mx, my)
                        self._stall_start_time = now
                    else:
                        moved = math.hypot(mx - self._stall_ref_pose[0],
                                           my - self._stall_ref_pose[1])
                        if moved >= self.STALL_DIST_THRESH:
                            self._stall_ref_pose = (mx, my)
                            self._stall_start_time = now
                        elif (now - self._stall_start_time) > self.STALL_TIME_THRESH:
                            self.get_logger().warn(
                                f'🧱 Stall detected: moved {moved:.2f}m in '
                                f'{(now - self._stall_start_time):.0f}s '
                                f'– running unstick maneuver')
                            self.cancel_nav2_goal()
                            self._mark_current_frontier_blocked()
                            self._frontier_goal = None
                            self._frontier_fail_cnt += 1
                            if abs(my) > self.ROOM_TRAP_Y:
                                self._escape_to_corridor = True
                            self._start_unstick('frontier progress stall')
                            return

                elapsed = self.now_sec() - self._nav2_goal_time
                if elapsed > self.FRONTIER_TIMEOUT:
                    self.cancel_nav2_goal()
                    self._mark_current_frontier_blocked()
                    self._frontier_goal = None
                    self._frontier_fail_cnt += 1
                    if self._patrol_mode:
                        self.get_logger().warn(
                            f'⏰ Patrol timeout ({elapsed:.0f}s) – forcing corridor escape')
                        self._escape_to_corridor = True
                        self.clear_costmaps()
                        return
                    self._force_probe_mode = True
                    self.get_logger().warn(
                        f'⏰ Frontier timeout ({elapsed:.0f}s) – picking new one '
                        f'(fail_count={self._frontier_fail_cnt})')
                return

            # Reset stall watcher when not actively driving a Nav2 goal
            self._stall_ref_pose = None
            self._stall_start_time = 0.0

            if self._escape_to_corridor:
                esc = self._corridor_escape_goal()
                if esc is not None:
                    ex, ey = esc
                    self._frontier_goal = esc
                    self.get_logger().warn(
                        f'🚪 Escaping room trap -> corridor goal ({ex:.1f}, {ey:.1f})')
                    self.send_nav2_goal(ex, ey)
                    return
                self._escape_to_corridor = False

            # No active goal – ask SLAM map for the next frontier
            if self._patrol_mode:
                patrol = self._next_patrol_goal()
                if patrol is not None:
                    px, py = patrol
                    self._frontier_goal = patrol
                    self.get_logger().info(
                        f'🧭 Patrol goal: ({px:.1f}, {py:.1f})')
                    self.send_nav2_goal(px, py)
                    return
                self.get_logger().warn(
                    '⚠️ Patrol goal unavailable in current map – switching to   probe recovery')
                self._patrol_mode = False
                self._force_probe_mode = True
                self._probe_roam_until = 0.0
                return

            if self._force_probe_mode:
                if self.now_sec() < self._probe_roam_until:
                    self._publish_active_search_cmd()
                    return

                probe = self._next_probe_goal()
                if probe is not None:
                    px, py = probe
                    self._frontier_goal = probe
                    self.get_logger().info(
                        f'🧭   probe recovery goal: ({px:.1f}, {py:.1f})')
                    self.send_nav2_goal(px, py)
                    return

                self._force_probe_mode = False
                self._probe_roam_until = self.now_sec() + self.PROBE_ROAM_SECONDS
                self._no_frontier_since = self.now_sec()
                self.get_logger().warn(
                    f'⚠️ No valid probe goal in current map – local roam for '
                    f'{self.PROBE_ROAM_SECONDS:.0f}s')
                self._publish_active_search_cmd()
                return

            frontier = self.get_best_frontier()
            if frontier is None:
                # Map has no frontiers right now – keep searching indefinitely
                if self._no_frontier_since is None:
                    self._no_frontier_since = self.now_sec()
                    self.get_logger().info(
                        '🗺️  No frontiers in map – switching to active search motion...')
                    self._publish_active_search_cmd()
                    return
                waited = self.now_sec() - self._no_frontier_since

                # Stop condition #1: full coverage complete (before sphere).
                if self.STOP_ON_FIRST_SUCCESS and self._is_map_complete():
                    if self._map_complete_since is None:
                        self._map_complete_since = self.now_sec()
                    elif (self.now_sec() - self._map_complete_since) >= self.MAP_COMPLETE_HOLD_S:
                        self._complete_mission(
                            f'map fully explored (known={self._known_map_ratio():.2f})')
                        return
                else:
                    self._map_complete_since = None

                if waited > 12.0:
                    self.get_logger().warn(
                        f'🧭 No frontiers for {waited:.0f}s – enabling   probe recovery')
                    self._force_probe_mode = True
                    self._probe_roam_until = 0.0
                    return
                if waited > 30.0:
                    self.get_logger().warn(
                        f'♻️ No new frontiers for {waited:.0f}s – '
                        f'clearing costmaps and continuing active search')
                    self._no_frontier_since = self.now_sec()
                    self.clear_costmaps()
                self._publish_active_search_cmd()
                return

            # Got a frontier – navigate there
            self._no_frontier_since = None
            self._map_complete_since = None
            fx, fy = frontier
            self._frontier_goal = frontier
            mpose = self.get_map_pose()
            dist = math.sqrt((fx - mpose[0])**2 + (fy - mpose[1])**2) \
                if mpose else 0.0
            self.get_logger().info(
                f'🗺️  New frontier #{self._frontier_count + 1}: '
                f'({fx:.1f}, {fy:.1f}), dist={dist:.1f}m')
            self.send_nav2_goal(fx, fy)
            return

        # ─── SCAN_360 (angle-based true 360° rotation) ───
        if self.state == 'SCAN_360':
            if self.sphere_confirmed:
                if self.STOP_ON_FIRST_SUCCESS:
                    self._complete_mission('sphere detected during scan')
                else:
                    self.stop_robot()
                    self.state = 'NAVIGATE'
                    self.servo_lost_time = None
                    self.get_logger().info(
                        '🧭 Sphere detected → NAVIGATE (visual servo)')
                return

            if self.scan_last_yaw is None:
                self.scan_last_yaw = self.robot_yaw

            delta = self.normalize_angle(self.robot_yaw - self.scan_last_yaw)
            self.scan_accumulated += abs(delta)
            self.scan_last_yaw = self.robot_yaw

            elapsed = self.now_sec() - self.scan_start_time if self.scan_start_time else 0.0
            if self.scan_accumulated >= self.SCAN_TARGET_RAD or \
                    elapsed >= (self.SCAN_DURATION * 2.0):
                self.stop_robot()
                self.get_logger().info(
                    f'🔄 Scan done ({math.degrees(self.scan_accumulated):.0f}° '
                    f'in {elapsed:.0f}s) – not found here, continuing exploration')
                self.state = 'EXPLORE'  # frontier explorer picks the next goal
                return

            cmd = Twist()
            cmd.angular.z = self.SEARCH_YAW_SPEED
            self.cmd_pub.publish(self._apply_safety_to_cmd(cmd))
            return

        # ─── NAVIGATE (visual servo with obstacle avoidance) ───
        if self.state == 'NAVIGATE':
            # LiDAR safety — too close to anything in front
            if self.front_min_range < self.NAV_LIDAR_STOP:
                # Only treat this as "arrived" if camera also sees a large blob.
                if self.blob_detected and \
                        self.blob_area >= self.ARRIVAL_LIDAR_BLOB_AREA:
                    self.stop_robot()
                    self._sphere_reached = True
                    self.state = 'WAIT'
                    self.wait_start_time = None
                    self.get_logger().info(
                        f'⏱️  LiDAR+vision arrival: '
                        f'front={self.front_min_range:.2f}m, '
                        f'area={self.blob_area:.0f}px → WAIT')
                    return

                # Otherwise this is likely a wall/obstacle, not the sphere.
                cmd = Twist()
                cmd.angular.z = self.SEARCH_YAW_SPEED * 0.6
                self.cmd_pub.publish(self._apply_safety_to_cmd(cmd))
                return

            # Blob large enough → arrived at sphere
            if self.blob_detected and \
                    self.blob_area >= self.BLOB_CLOSE_AREA:
                self.stop_robot()
                self._sphere_reached = True
                self.state = 'WAIT'
                self.wait_start_time = None
                self.get_logger().info(
                    f'⏱️  Blob close! area={self.blob_area:.0f}px'
                    f' >= {self.BLOB_CLOSE_AREA}px → WAIT')
                return

            # Visual servo toward sphere
            if self.blob_detected and self.blob_cx is not None:
                self.servo_lost_time = None
                half_w = self.IMAGE_WIDTH / 2.0
                error = (half_w - self.blob_cx) / half_w

                cmd = Twist()
                cmd.linear.x = self.SERVO_SPEED
                cmd.angular.z = max(-self.SERVO_MAX_YAW,
                                    min(self.SERVO_MAX_YAW,
                                        error * self.SERVO_KP))
                self.cmd_pub.publish(self._apply_safety_to_cmd(cmd))
            else:
                # Lost the blob — slow rotate to re-acquire
                if self.servo_lost_time is None:
                    self.servo_lost_time = self.now_sec()
                    self.get_logger().warn(
                        '⚠️  Blob lost – rotating to re-acquire...')

                lost = self.now_sec() - self.servo_lost_time
                if lost > self.SERVO_LOST_TIMEOUT:
                    self.stop_robot()
                    if self._target_locked:
                        self.get_logger().warn(
                            '❌ Blob lost too long after target lock – RETURN home')
                        self.sphere_confirmed = False
                        self._start_return_home(
                            'Target lost during approach')
                    else:
                        self.get_logger().warn(
                            '❌ Blob lost too long – switching to 360° scan')
                        self.sphere_confirmed = False
                        self._sphere_reached = False
                        self._frontier_goal = None  # pick a fresh frontier after scan
                        self.begin_scan()
                    return

                cmd = Twist()
                cmd.angular.z = self.SEARCH_YAW_SPEED * 0.5
                self.cmd_pub.publish(self._apply_safety_to_cmd(cmd))
            return

        # ─── WAIT ───
        if self.state == 'WAIT':
            self.stop_robot()
            if self.wait_start_time is None:
                self.wait_start_time = self.now_sec()
            elapsed = self.now_sec() - self.wait_start_time
            if elapsed >= self.WAIT_SECONDS:
                if not self._sphere_reached:
                    self.get_logger().warn(
                        '⚠️ WAIT finished without verified sphere arrival '
                        '— forcing RETURN home.')
                    self.sphere_confirmed = False
                    self._start_return_home(
                        'WAIT timeout')
                    return
                self._start_return_home('Wait done – turning 180° then RETURN')
            return

        # ─── RETURN (Nav2   only) ───
        if self.state == 'RETURN':
            sx, sy, syaw = self.start_pose

            # ── Map-frame distance (SLAM is accurate, odom integration is NOT
            #    for skid-steer because twist.linear.x under-reports speed) ──
            mpose = self.get_map_pose()
            if mpose is None:
                self.stop_robot()
                return

            mx, my, myaw = mpose
            map_dx = sx - mx
            map_dy = sy - my
            map_dist = math.sqrt(map_dx * map_dx + map_dy * map_dy)

            odom_dist = float('inf')
            if self.start_odom_pose is not None:
                ox, oy, _ = self.start_odom_pose
                odom_dist = math.hypot(ox - self.robot_x, oy - self.robot_y)

            now = self.now_sec()
            odom_ok = True
            if self.start_odom_pose is not None:
                odom_ok = (odom_dist <= self.HOME_REACHED_ODOM_DIST)
            near_home = (map_dist <= self.HOME_REACHED_DIST) and odom_ok
            if near_home:
                if self._home_near_since is None:
                    self._home_near_since = now
            else:
                self._home_near_since = None
            home_stable = self._home_near_since is not None and \
                (now - self._home_near_since) >= self.HOME_STABLE_SEC

            # Nav2 path-planned return ( )
            if self._return_phase == 'NAV2':
                if self.now_sec() < self._return_retry_after:
                    return

                if not self._return_nav2_sent:
                    self._return_nav2_sent = True
                    self.nav2_reached = False
                    self.nav2_failed = False
                    home_yaw = syaw
                    self.send_nav2_goal(sx, sy, home_yaw)
                    self._return_start_time = self.now_sec()
                    self._return_retry_count += 1
                    self.get_logger().info(
                        f'🏠 Nav2   return goal [{self._return_retry_count}/'
                        f'{self.RETURN_MAX_RETRIES}]: map ({sx:.1f}, {sy:.1f}), '
                        f'map_dist={map_dist:.1f}m, odom_dist={odom_dist:.1f}m')
                    return

                if self.nav2_reached:
                    if near_home:
                        self.stop_robot()
                        self.cancel_nav2_goal()
                        self._mission_complete = True
                        self.sphere_confirmed = False
                        self._target_locked = False
                        self.state = 'DONE'
                        self.get_logger().info(
                            f'✅ Nav2 RETURN succeeded! map_dist={map_dist:.2f}m, '
                            f'odom_dist={odom_dist:.2f}m 🏠')
                    else:
                        self.nav2_reached = False
                        self._return_nav2_sent = False
                        self._return_retry_after = self.now_sec() + self.RETURN_RETRY_DELAY
                        self.get_logger().warn(
                            f'⚠️ Nav2 reported success but robot is still '
                            f'map={map_dist:.2f}m/odom={odom_dist:.2f}m from home; '
                            f'retrying   return...')
                        self.clear_costmaps()
                    return

                if self.nav2_failed:
                    self.nav2_failed = False
                    self._return_nav2_sent = False
                    self._return_retry_after = self.now_sec() + self.RETURN_RETRY_DELAY
                    self.get_logger().warn(
                        f'⚠️ Nav2   RETURN failed (attempt {self._return_retry_count}/'
                        f'{self.RETURN_MAX_RETRIES}) – clearing costmaps and retrying...')
                    self.clear_costmaps()
                    if self._return_retry_count >= self.RETURN_MAX_RETRIES:
                        self.stop_robot()
                        self.state = 'DONE'
                        self._mission_complete = True
                        self.get_logger().error(
                            '❌ RETURN aborted after maximum Nav2 retries.')
                    return

                elapsed = self.now_sec() - self._return_start_time
                if elapsed > self.NAV2_TIMEOUT:
                    self.cancel_nav2_goal()
                    self._return_nav2_sent = False
                    self._return_retry_after = self.now_sec() + self.RETURN_RETRY_DELAY
                    self.get_logger().warn(
                        f'⏰ Nav2 RETURN timeout ({elapsed:.0f}s) '
                        f'on attempt {self._return_retry_count}/{self.RETURN_MAX_RETRIES}; '
                        f'retrying   return...')
                    self.clear_costmaps()
                    if self._return_retry_count >= self.RETURN_MAX_RETRIES:
                        self.stop_robot()
                        self.state = 'DONE'
                        self._mission_complete = True
                        self.get_logger().error(
                            '❌ RETURN aborted after repeated Nav2 timeouts.')
                    return

                return
            return

        # ─── DONE ───
        if self.state == 'DONE':
            self.stop_robot()
            return


def main(args=None):
    rclpy.init(args=args)
    node = HospitalMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
