import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePub(Node):
    def __init__(self):
        super().__init__('simple_pub')
        self.pub = self.create_publisher(String, '/test_topic', 10)
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('SIMPLE PUB ONLINE')

    def timer_callback(self):
        msg = String()
        msg.data = 'HELLO FROM ROS 2'
        self.pub.publish(msg)
        self.get_logger().info('Published test message')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SimplePub())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
