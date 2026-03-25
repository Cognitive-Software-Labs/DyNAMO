#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_pub')
        
        # authoritative /odom publisher with injected covariance
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to ground-truth from gazebo
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        
        self.get_logger().info("System started: odom_tf_pub is now the authoritative TF source.")

    def odom_callback(self, msg: Odometry):
        # 1. Standardize Header
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # 2. Inject Static Covariance 
        # (Allows SLAM to trust odom for heading in featureless corridors)
        cov = np.zeros(36)
        cov[0]  = 0.001 # x
        cov[7]  = 0.001 # y
        cov[14] = 1e9   # z
        cov[21] = 1e9   # r
        cov[28] = 1e9   # p
        cov[35] = 0.001 # yaw (high trust)
        msg.pose.covariance = cov.tolist()
        msg.twist.covariance = cov.tolist()

        # 3. Publish to ROS
        self.odom_pub.publish(msg)

        # 4. Final TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
