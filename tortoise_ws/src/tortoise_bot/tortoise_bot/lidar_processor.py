#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class LidarDistanceMonitor(Node):
    def __init__(self):
        super().__init__('lidar_distance_monitor')
        
        # Declare and get parameters
        self.declare_parameter('field_of_view_degrees', 15.0)
        self.declare_parameter('min_safe_distance', 0.5)
        self.field_of_view_degrees = self.get_parameter('field_of_view_degrees').get_parameter_value().double_value
        self.min_safe_distance = self.get_parameter('min_safe_distance').get_parameter_value().double_value

        # Subscribers and Publishers
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.distance_publisher = self.create_publisher(Float32, '/closest_object_distance', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        print(f"[LIDAR Monitor] Field of View: ±{self.field_of_view_degrees}° | Min Safe Distance: {self.min_safe_distance}m")

    def scan_callback(self, msg):
        try:
            # Step 1: Calculate scan indices in front of robot
            total_points = len(msg.ranges)
            center_index = total_points // 2
            angle_per_point = msg.angle_increment
            fov_radians = math.radians(self.field_of_view_degrees)
            points_per_side = int((fov_radians / 2) / angle_per_point)
            start_index = max(0, center_index - points_per_side)
            end_index = min(total_points - 1, center_index + points_per_side)

            # Step 2: Filter valid ranges in FOV
            front_ranges = msg.ranges[start_index:end_index + 1]
            valid_ranges = [r for r in front_ranges if math.isfinite(r) and msg.range_min < r < msg.range_max]

            # Step 3: Determine closest object
            if valid_ranges:
                closest_distance = min(valid_ranges)
                
                # Check if real object detected (not max range)
                if closest_distance < msg.range_max * 0.95:
                    # Object detected - publish distance and print
                    distance_msg = Float32()
                    distance_msg.data = float(closest_distance)
                    self.distance_publisher.publish(distance_msg)
                    
                    # Print distance
                    if closest_distance < 0.6:
                        print(f"🔴 {closest_distance:.2f}m")
                    elif closest_distance < 0.9:
                        print(f"🟡 {closest_distance:.2f}m")
                    else:
                        print(f"🟢 {closest_distance:.2f}m")
                    
            # Step 4: Collision avoidance
                    if closest_distance < self.min_safe_distance:
                        stop_msg = Twist()
                        self.cmd_vel_publisher.publish(stop_msg)
                        if closest_distance < 0.3:
                            print("⚠️ EMERGENCY STOP!")
                else:
                    # No object - publish zero
                    distance_msg = Float32()
                    distance_msg.data = 0.0
                    self.distance_publisher.publish(distance_msg)
            else:
                # No valid data - publish zero
                distance_msg = Float32()
                distance_msg.data = 0.0
                self.distance_publisher.publish(distance_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LidarDistanceMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exit")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()