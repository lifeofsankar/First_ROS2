#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import numpy as np
from collections import deque

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Subscriber to LIDAR data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for closest object distance
        self.distance_publisher = self.create_publisher(
            Float32,
            '/closest_object_distance',
            10
        )
        
        # Parameters for filtering
        self.field_of_view_degrees = 15.0  # ±15 degrees
        self.field_of_view_radians = math.radians(self.field_of_view_degrees)
        
        # Moving average filter parameters
        self.moving_avg_window = 5
        self.distance_history = deque(maxlen=self.moving_avg_window)
        
        # Spike filter parameters
        self.max_distance_change = 1.0  # Maximum allowed change in distance (meters)
        self.previous_distance = None
        
        self.get_logger().info(f'LIDAR Processor started. Monitoring ±{self.field_of_view_degrees}° field of view.')

    def is_valid_range(self, range_value, min_range, max_range):
        """Check if a range value is valid (not inf, nan, or zero)"""
        return (not math.isinf(range_value) and 
                not math.isnan(range_value) and 
                range_value > min_range and 
                range_value <= max_range)

    def get_front_facing_indices(self, scan_msg):
        """Get indices corresponding to the front-facing field of view"""
        total_points = len(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment
        
        # Calculate start and end angles for the field of view
        # Assuming 0 radians is directly in front
        start_angle = -self.field_of_view_radians
        end_angle = self.field_of_view_radians
        
        # Convert angles to indices
        # Index 0 corresponds to angle_min, so we need to offset
        center_index = int((0 - scan_msg.angle_min) / angle_increment)
        start_index = int((start_angle - scan_msg.angle_min) / angle_increment)
        end_index = int((end_angle - scan_msg.angle_min) / angle_increment)
        
        # Ensure indices are within bounds
        start_index = max(0, min(start_index, total_points - 1))
        end_index = max(0, min(end_index, total_points - 1))
        
        # Handle wrap-around if needed
        if start_index > end_index:
            indices = list(range(start_index, total_points)) + list(range(0, end_index + 1))
        else:
            indices = list(range(start_index, end_index + 1))
            
        return indices, center_index

    def filter_spikes(self, distance):
        """Remove sudden spikes in distance readings"""
        if self.previous_distance is None:
            self.previous_distance = distance
            return distance
            
        # Check if the change is too large (spike detection)
        distance_change = abs(distance - self.previous_distance)
        if distance_change > self.max_distance_change:
            # Keep the previous distance (filter out the spike)
            self.get_logger().debug(f'Spike detected: {distance:.2f}m (change: {distance_change:.2f}m)')
            return self.previous_distance
        else:
            self.previous_distance = distance
            return distance

    def apply_moving_average(self, distance):
        """Apply moving average filter"""
        self.distance_history.append(distance)
        return sum(self.distance_history) / len(self.distance_history)

    def scan_callback(self, msg):
        """Process incoming LIDAR scan data"""
        try:
            # Get the indices for the front-facing field of view
            front_indices, center_index = self.get_front_facing_indices(msg)
            
            if not front_indices:
                self.get_logger().warn('No valid front-facing indices found')
                return
            
            # Extract valid ranges from the front-facing area
            valid_ranges = []
            for idx in front_indices:
                if idx < len(msg.ranges):
                    range_value = msg.ranges[idx]
                    if self.is_valid_range(range_value, msg.range_min, msg.range_max):
                        valid_ranges.append(range_value)
            
            if not valid_ranges:
                self.get_logger().debug('No valid ranges found in front-facing area')
                return
            
            # Apply different filtering methods
            # Method 1: Minimum valid distance
            min_distance = min(valid_ranges)
            
            # Method 2: Median filter (removes outliers)
            median_distance = np.median(valid_ranges)
            
            # Method 3: Use median as our primary filtered distance
            filtered_distance = median_distance
            
            # Apply spike filter
            spike_filtered_distance = self.filter_spikes(filtered_distance)
            
            # Apply moving average
            final_distance = self.apply_moving_average(spike_filtered_distance)
            
            # Publish the result
            distance_msg = Float32()
            distance_msg.data = final_distance
            self.distance_publisher.publish(distance_msg)
            
            # Log information
            angle_deg = math.degrees(msg.angle_min + center_index * msg.angle_increment)
            self.get_logger().info(
                f'Closest object: {final_distance:.2f}m '
                f'(raw min: {min_distance:.2f}m, median: {median_distance:.2f}m) '
                f'| Valid points: {len(valid_ranges)}/{len(front_indices)} '
                f'| Center angle: {angle_deg:.1f}°'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing scan data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        lidar_processor = LidarProcessor()
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'lidar_processor' in locals():
            lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()