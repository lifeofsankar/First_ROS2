#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import math
import time

class TortoiseController(Node):
    def __init__(self):
        super().__init__('tortoise_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Control parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.current_pose = None
        self.target_reached = False
        
        # Simple state machine
        self.state = 'forward'
        self.state_timer = 0
        
        self.get_logger().info('Tortoise Controller initialized')
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def joint_callback(self, msg):
        # Process joint states if needed
        pass
    
    def control_loop(self):
        cmd = Twist()
        
        # Simple behavior: move forward for 3 seconds, then turn
        if self.state == 'forward':
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.state_timer += 0.1
            
            if self.state_timer >= 3.0:
                self.state = 'turning'
                self.state_timer = 0.0
                
        elif self.state == 'turning':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.state_timer += 0.1
            
            if self.state_timer >= 2.0:
                self.state = 'forward'
                self.state_timer = 0.0
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = TortoiseController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()