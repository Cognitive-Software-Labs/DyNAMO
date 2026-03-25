#!/usr/bin/env python3
"""
drive_forward.py  –  Drive the robot forward exactly 1 meter using raw
/cmd_vel commands (forceful, bypasses Nav2).

Distance is tracked via /odom.  The node publishes a constant linear
velocity until the target distance is reached, then sends a zero-velocity
stop command and exits.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class DriveForward(Node):
    TARGET_DISTANCE = 10.0   # metres
    LINEAR_SPEED = 0.5      # m/s
    PUBLISH_HZ = 20.0

    def __init__(self):
        super().__init__("drive_forward")
        self.get_logger().info(
            f"Drive-forward node started  –  target = {self.TARGET_DISTANCE} m "
            f"@ {self.LINEAR_SPEED} m/s"
        )

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, 10
        )

        # State
        self.start_x: float | None = None
        self.start_y: float | None = None
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.done = False

        # Timer drives the control loop
        self.timer = self.create_timer(1.0 / self.PUBLISH_HZ, self._control_loop)

    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry):
        """Cache current position; latch the first reading as origin."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(
                f"Start position latched: ({x:.3f}, {y:.3f})"
            )

        self.cur_x = x
        self.cur_y = y

    # ------------------------------------------------------------------ #
    def _control_loop(self):
        """Publish velocity or stop once target distance is reached."""
        if self.done:
            return

        # Still waiting for the first /odom message
        if self.start_x is None:
            return

        dx = self.cur_x - self.start_x
        dy = self.cur_y - self.start_y
        distance = math.hypot(dx, dy)

        if distance >= self.TARGET_DISTANCE:
            # ---- STOP ----
            self._publish_stop()
            self.get_logger().info(
                f"Target reached!  Travelled {distance:.3f} m — stopping."
            )
            self.done = True
            # Give Gazebo a moment to apply the zero-vel, then exit
            self.create_timer(0.5, self._shutdown)
            return

        # ---- DRIVE FORWARD ----
        twist = Twist()
        twist.linear.x = self.LINEAR_SPEED
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------ #
    def _publish_stop(self):
        """Send several zero-velocity commands to guarantee the robot stops."""
        stop = Twist()  # all zeros
        for _ in range(5):
            self.cmd_pub.publish(stop)

    def _shutdown(self):
        self._publish_stop()
        self.get_logger().info("Shutting down drive_forward node.")
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DriveForward()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
