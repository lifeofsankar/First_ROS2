#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')

        # Publisher: sends velocity commands to robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: receives LIDAR data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Motion parameters
        self.tolerance = 0.1        # radians (~6°)
        self.stop_distance = 0.3    # meters
        self.turn_speed = 0.3       # rad/s
        self.forward_speed = 0.1    # m/s

        self.get_logger().info("Ball Follower Node has started.")

    # === LIDAR processing ===
    def find_closest_object(self, ranges):
        cleaned_ranges = [r if r > 0.0 else 999.0 for r in ranges]
        min_distance = min(cleaned_ranges)
        min_index = cleaned_ranges.index(min_distance)
        return min_distance, min_index

    def calculate_angle(self, min_index, angle_min, angle_increment):
        angle_to_object = angle_min + (min_index * angle_increment)
        return angle_to_object

    # === Decision making ===
    def decide_movement(self, angle_to_object, min_distance):
        if min_distance <= self.stop_distance:
            return 0.0, 0.0  # stop
        elif angle_to_object > self.tolerance:
            return 0.0, self.turn_speed  # turn left
        elif angle_to_object < -self.tolerance:
            return 0.0, -self.turn_speed  # turn right
        else:
            return self.forward_speed, 0.0  # move forward

    # === Command publishing ===
    def publish_cmd(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    # === Main callback ===
    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        min_distance, min_index = self.find_closest_object(ranges)
        angle_to_object = self.calculate_angle(min_index, angle_min, angle_increment)
        linear_x, angular_z = self.decide_movement(angle_to_object, min_distance)

        self.publish_cmd(linear_x, angular_z)

        # Debug info
        self.get_logger().info(
            f"Dist: {min_distance:.2f} m | Angle: {angle_to_object:.2f} rad | "
            f"Cmd: lin {linear_x:.2f}, ang {angular_z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
