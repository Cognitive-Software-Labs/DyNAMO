#!/usr/bin/env python3
"""
Odom → TF Publisher (integrated from /cmd_vel)
=============================================
This node generates a continuous /odom stream and corresponding TF (odom->base_link)
by integrating the commanded velocity (/cmd_vel). That keeps rviz/nav2 in sync with the
mission's motion plan even if the simulator's odometry is not available.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Command velocity -> integrated odometry
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Publish generated odom (used by Nav2 + RViz)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self._last_time = self.get_clock().now()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vth = 0.0

        self.create_timer(1.0 / 50.0, self._timer_cb)

        self.get_logger().info(
            'odom_tf_publisher: integrating /cmd_vel into /odom + TF')

    def cmd_vel_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._vth = msg.angular.z

    def _timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Integrate pose from commanded velocity in the odom frame.
        dx = (self._vx * math.cos(self._yaw) -
              self._vy * math.sin(self._yaw)) * dt
        dy = (self._vx * math.sin(self._yaw) +
              self._vy * math.cos(self._yaw)) * dt
        self._x += dx
        self._y += dy
        self._yaw += self._vth * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._vth

        self._odom_pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
