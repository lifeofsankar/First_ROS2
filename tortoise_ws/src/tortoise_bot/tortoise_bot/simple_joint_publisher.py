import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class SimpleJointPublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_publisher')
        
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0]
        
        self.angle = 0.0
        
    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # To Simulate rotating wheels
        self.angle += 0.1
        self.joint_state.position[0] = math.sin(self.angle)
        self.joint_state.position[1] = math.sin(self.angle + math.pi/4)
        
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = SimpleJointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()