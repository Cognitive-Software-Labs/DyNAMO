#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the odometry topic coming from Gazebo
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        self.get_logger().info("Odom to TF broadcaster started! The robot spine is connected.")

    def odom_callback(self, msg):
        # Create a new Transform message
        t = TransformStamped()

        # IMPORTANT: Copy the exact timestamp from the simulation
        t.header.stamp = msg.header.stamp
        
        # Define the parent and child frames
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Copy the XYZ position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy the quaternion rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform to the rest of the ROS 2 system
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()