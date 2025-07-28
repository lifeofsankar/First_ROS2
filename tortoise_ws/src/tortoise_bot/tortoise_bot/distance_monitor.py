#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

class DistanceMonitor(Node):
    def __init__(self):
        super().__init__('distance_monitor')
        
        # Subscriber to closest object distance
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/closest_object_distance',
            self.distance_callback,
            10
        )
        
        # Publisher for velocity commands (optional - for collision avoidance demo)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Parameters
        self.min_safe_distance = 0.5  # meters
        self.last_distance = None
        self.collision_avoidance_enabled = True  # Set to True for automatic stopping
        
        self.get_logger().info('Distance Monitor started. Monitoring closest object distance.')
        self.get_logger().info(f'Collision avoidance: {"ENABLED" if self.collision_avoidance_enabled else "DISABLED"}')

    def distance_callback(self, msg):
        """Process closest object distance"""
        distance = msg.data
        self.last_distance = distance
        
        # Create a visual representation
        bar_length = 50
        max_display_distance = 3.0  # meters
        
        # Calculate bar fill
        normalized_distance = min(distance / max_display_distance, 1.0)
        filled_length = int(bar_length * normalized_distance)
        
        # Create color-coded status
        if distance < 0.3:
            status = "🔴 CRITICAL"
            bar_char = "█"
            color = "\033[91m"  # Red
        elif distance < 0.8:
            status = "🟡 WARNING"
            bar_char = "▓"
            color = "\033[93m"  # Yellow
        else:
            status = "🟢 SAFE"
            bar_char = "░"
            color = "\033[92m"  # Green
        
        reset_color = "\033[0m"
        
        # Create the distance bar
        filled_bar = color + bar_char * filled_length + reset_color
        empty_bar = "·" * (bar_length - filled_length)
        distance_bar = filled_bar + empty_bar
        
        # Print the visualization
        print(f"\r{status} | Distance: {distance:.2f}m [{distance_bar}] ", end="", flush=True)
        
        # Optional collision avoidance
        if self.collision_avoidance_enabled:
            self.collision_avoidance(distance)
    
    def collision_avoidance(self, distance):
        """Simple collision avoidance by stopping the robot"""
        if distance < self.min_safe_distance:
            # Stop the robot
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(stop_msg)
            
            if distance < 0.2:
                self.get_logger().warn(f'EMERGENCY STOP! Object too close: {distance:.2f}m')

    def print_stats(self):
        """Print periodic statistics"""
        if self.last_distance is not None:
            print(f"\n--- Distance Statistics ---")
            print(f"Current distance: {self.last_distance:.2f}m")
            print(f"Safe distance threshold: {self.min_safe_distance:.2f}m")
            print(f"Status: {'SAFE' if self.last_distance > self.min_safe_distance else 'DANGER'}")
            print("----------------------------\n")

def main():
    rclpy.init()
    
    try:
        node = DistanceMonitor()  
        
        # Print initial message
        print("\n" + "="*60)
        print("  LIDAR DISTANCE MONITOR")
        print("="*60)
        print("Monitoring closest object distance in front of robot...")
        print("Press Ctrl+C to stop\n")
        
        # Start spinning
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down distance monitor...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()