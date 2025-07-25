import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.wheel_left_pub = self.create_publisher(Float64, '/tortoisebot/wheel_left_controller/command', 10)
        self.wheel_right_pub = self.create_publisher(Float64, '/tortoisebot/wheel_right_controller/command', 10)
        self.get_logger().info('Velocity controller started')

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Simple differential drive kinematics
        # This is a simplified model and might need tuning for your specific robot
        wheel_radius = 0.033 # meters
        wheel_separation = 0.160 # meters

        left_wheel_velocity = (linear_vel - angular_vel * wheel_separation / 2.0) / wheel_radius
        right_wheel_velocity = (linear_vel + angular_vel * wheel_separation / 2.0) / wheel_radius

        left_msg = Float64()
        left_msg.data = left_wheel_velocity
        self.wheel_left_pub.publish(left_msg)

        right_msg = Float64()
        right_msg.data = right_wheel_velocity
        self.wheel_right_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()