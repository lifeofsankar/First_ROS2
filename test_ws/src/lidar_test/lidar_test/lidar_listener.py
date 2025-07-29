import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Min Distance: {min_distance:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
