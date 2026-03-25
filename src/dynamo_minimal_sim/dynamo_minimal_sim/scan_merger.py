#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        self.get_logger().info("Scan Merger started - building 360 view.")

        self.target_frame = 'base_link'
        self.merged_topic = '/scan_merged'

        self.front_scan = None
        self.rear_scan = None

        self.create_subscription(LaserScan, '/scan', self.front_cb, 10)
        self.create_subscription(LaserScan, '/scan_rear', self.rear_cb, 10)
        self.pub = self.create_publisher(LaserScan, self.merged_topic, 10)

        # Higher frequency to keep up with simulation physics
        self.create_timer(0.02, self.merge_and_publish)

    def front_cb(self, msg):
        self.front_scan = msg

    def rear_cb(self, msg):
        self.rear_scan = msg

    def merge_and_publish(self):
        # Graceful handling of startup delay
        if self.front_scan is None or self.rear_scan is None:
            return

        merged = LaserScan()
        merged.header.stamp = self.front_scan.header.stamp
        merged.header.frame_id = self.target_frame
        
        merged.angle_min = -np.pi
        merged.angle_max = np.pi
        merged.angle_increment = 2 * np.pi / 720.0
        merged.range_min = min(self.front_scan.range_min, self.rear_scan.range_min)
        merged.range_max = max(self.front_scan.range_max, self.rear_scan.range_max)

        ranges = np.full(720, np.inf)

        def add_scan(scan, offset_x, offset_y, flip=False):
             angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
             for i, r in enumerate(scan.ranges):
                 if r < scan.range_min or r > scan.range_max or np.isnan(r) or np.isinf(r):
                     continue
                 
                 # Local coordinates in lidar frame
                 lx = r * np.cos(angles[i])
                 ly = r * np.sin(angles[i])

                 # Transform to base_link
                 if flip:
                      gx = -lx + offset_x
                      gy = -ly + offset_y
                 else:
                      gx = lx + offset_x
                      gy = ly + offset_y
                 
                 final_r = np.sqrt(gx**2 + gy**2)
                 final_a = np.arctan2(gy, gx)

                 idx = int((final_a - merged.angle_min) / merged.angle_increment)
                 if 0 <= idx < 720:
                     if final_r < ranges[idx]:
                         ranges[idx] = final_r

        # Offsets based on URDF: Front (0.35, 0.25), Rear (-0.35, -0.25)
        add_scan(self.front_scan, 0.35, 0.25, flip=False)
        add_scan(self.rear_scan, -0.35, -0.25, flip=True)

        merged.ranges = ranges.tolist()
        self.pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
