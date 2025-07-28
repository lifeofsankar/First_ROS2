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

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.distance_publisher = self.create_publisher(
            Float32,
            '/closest_object_distance',
            10
        )

        self.field_of_view_degrees = 15.0
        self.moving_avg_window = 5
        self.distance_history = deque(maxlen=self.moving_avg_window)

        self.get_logger().info(f'LIDAR Processor started. Monitoring ±{self.field_of_view_degrees}° field of view.')

    def is_valid_range(self, range_value, min_range, max_range):
        """Check if a range value is valid."""
        return math.isfinite(range_value) and range_value > min_range

    def scan_callback(self, msg):
        """Process incoming LIDAR scan data."""
        try:
            # This is a more direct way to find the front-facing indices
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

            front_ranges = msg.ranges[start_index:end_index + 1]

            valid_ranges = [r for r in front_ranges if self.is_valid_range(r, msg.range_min, msg.range_max)]

            if valid_ranges:
                # Use the absolute minimum valid value for simplicity and responsiveness
                closest_distance = min(valid_ranges)

                self.distance_history.append(closest_distance)
                smoothed_distance = np.mean(self.distance_history)

                distance_msg = Float32()
                distance_msg.data = float(smoothed_distance)
                self.distance_publisher.publish(distance_msg)

        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        lidar_processor = LidarProcessor()
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'lidar_processor' in locals() and rclpy.ok():
            lidar_processor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()