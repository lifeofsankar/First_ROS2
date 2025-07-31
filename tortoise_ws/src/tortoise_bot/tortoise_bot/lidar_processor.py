#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import numpy as np
from collections import deque
import time

class LidarDistanceMonitor(Node):
    def __init__(self):
        super().__init__('lidar_distance_monitor')
        
        # Subscriber to LIDAR scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for closest object distance (optional - for other nodes)
        self.distance_publisher = self.create_publisher(
            Float32,
            '/closest_object_distance',
            10
        )
        
        # Publisher for velocity commands (for collision avoidance)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Declare parameters with default values
        self.declare_parameter('field_of_view_degrees', 15.0)
        self.declare_parameter('moving_avg_window', 5)
        self.declare_parameter('min_safe_distance', 0.5)
        self.declare_parameter('collision_avoidance_enabled', True)
        
        # Get the parameters
        self.field_of_view_degrees = self.get_parameter('field_of_view_degrees').get_parameter_value().double_value
        self.moving_avg_window = self.get_parameter('moving_avg_window').get_parameter_value().integer_value
        self.min_safe_distance = self.get_parameter('min_safe_distance').get_parameter_value().double_value
        self.collision_avoidance_enabled = self.get_parameter('collision_avoidance_enabled').get_parameter_value().bool_value
        
        # Use the parameters
        self.field_of_view_radians = math.radians(self.field_of_view_degrees)
        self.distance_history = deque(maxlen=self.moving_avg_window)
        self.last_distance = None
        
        # Initialize display
        self.get_logger().info(f'LIDAR Distance Monitor started. Monitoring ±{self.field_of_view_degrees}° field of view.')
        self.get_logger().info(f'Collision avoidance: {"ENABLED" if self.collision_avoidance_enabled else "DISABLED"}')
        
        # Print initial message
        print("\n" + "="*60)
        print("  LIDAR DISTANCE MONITOR")
        print("="*60)
        print("Monitoring closest object distance in front of robot...")
        print("Press Ctrl+C to stop\n")

    def is_valid_range(self, range_value, min_range, max_range):
        """Check if a range value is valid."""
        return math.isfinite(range_value) and range_value > min_range and range_value < max_range

    def scan_callback(self, msg):
        """Process incoming LIDAR scan data and monitor distance."""
        try:
            # Find the front-facing indices
            total_points = len(msg.ranges)
            center_index = total_points // 2
            angle_per_point = msg.angle_increment
            fov_in_radians = math.radians(self.field_of_view_degrees)
            points_per_side = int((fov_in_radians / 2) / angle_per_point)
            
            start_index = center_index - points_per_side
            end_index = center_index + points_per_side
            
            # Clamp indices to be safe
            start_index = max(0, start_index)
            end_index = min(total_points - 1, end_index)
            
            # Extract front-facing ranges
            front_ranges = msg.ranges[start_index:end_index + 1]
            
            # Filter valid ranges
            valid_ranges = [r for r in front_ranges if self.is_valid_range(r, msg.range_min, msg.range_max)]
            
            if valid_ranges:
                # Use the absolute minimum valid value for responsiveness
                closest_distance = min(valid_ranges)
                self.distance_history.append(closest_distance)
                smoothed_distance = np.mean(self.distance_history)
                
                # Store for visualization and collision avoidance
                self.last_distance = smoothed_distance
                
                # Publish distance for other nodes (optional)
                distance_msg = Float32()
                distance_msg.data = float(smoothed_distance)
                self.distance_publisher.publish(distance_msg)
                
                # Visualize and handle collision avoidance
                self.visualize_distance(smoothed_distance)
                if self.collision_avoidance_enabled:
                    self.collision_avoidance(smoothed_distance)
                    
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {e}')

    def visualize_distance(self, distance):
        """Create visual representation of distance"""
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
            print(f"Field of view: ±{self.field_of_view_degrees}°")
            print(f"Moving average window: {self.moving_avg_window}")
            print(f"Status: {'SAFE' if self.last_distance > self.min_safe_distance else 'DANGER'}")
            print("----------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LidarDistanceMonitor()
        
        # Start spinning
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down LIDAR distance monitor...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()