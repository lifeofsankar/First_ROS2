#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class TurtleBotTeleop(Node):
    def __init__(self):
        super().__init__('turtle_bot_teleop')
        
        # Publisher for cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        
        # Key bindings
        self.key_bindings = {
            'w': (1, 0, 0, 0),     # Move forward
            's': (-1, 0, 0, 0),    # Move backward
            'a': (0, 0, 0, 1),     # Turn left
            'd': (0, 0, 0, -1),    # Turn right
            'q': (1, 0, 0, 1),     # Move forward and turn left
            'e': (1, 0, 0, -1),    # Move forward and turn right
            'z': (-1, 0, 0, 1),    # Move backward and turn left
            'c': (-1, 0, 0, -1),   # Move backward and turn right
            'x': (0, 0, 0, 0),     # Stop
            ' ': (0, 0, 0, 0),     # Stop (space bar)
        }
        
        # Speed control keys
        self.speed_keys = {
            '+': 'increase',
            '=': 'increase',
            '-': 'decrease',
            '_': 'decrease'
        }
        
        self.get_logger().info('Turtle Bot Teleop Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        print("\n" + "="*50)
        print("TURTLE BOT TELEOP CONTROL")
        print("="*50)
        print("Movement Controls:")
        print("  W - Move Forward")
        print("  S - Move Backward") 
        print("  A - Turn Left")
        print("  D - Turn Right")
        print("  Q - Forward + Left")
        print("  E - Forward + Right")
        print("  Z - Backward + Left")
        print("  C - Backward + Right")
        print("  X or SPACE - Stop")
        print("\nSpeed Controls:")
        print("  + or = - Increase Speed")
        print("  - or _ - Decrease Speed")
        print("\nPress CTRL+C to quit")
        print("="*50)
        print(f"Current Speed - Linear: {self.linear_speed:.1f} m/s, Angular: {self.angular_speed:.1f} rad/s")
    
    def get_key(self):
        """Get a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.cbreak(fd)
            # Check if input is available
            if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                key = sys.stdin.read(1)
            else:
                key = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def publish_twist(self, linear_x, linear_y, linear_z, angular_z):
        """Publish Twist message with given velocities"""
        twist = Twist()
        twist.linear.x = float(linear_x * self.linear_speed)
        twist.linear.y = float(linear_y * self.linear_speed)
        twist.linear.z = float(linear_z * self.linear_speed)
        twist.angular.z = float(angular_z * self.angular_speed)
        
        self.cmd_vel_pub.publish(twist)
        
        # Log current command
        if twist.linear.x != 0 or twist.angular.z != 0:
            self.get_logger().info(
                f'Linear: {twist.linear.x:.2f} m/s, Angular: {twist.angular.z:.2f} rad/s'
            )
    
    def adjust_speed(self, action):
        """Increase or decrease movement speeds"""
        if action == 'increase':
            self.linear_speed = min(self.linear_speed + self.speed_increment, 2.0)
            self.angular_speed = min(self.angular_speed + self.speed_increment, 3.0)
        elif action == 'decrease':
            self.linear_speed = max(self.linear_speed - self.speed_increment, 0.1)
            self.angular_speed = max(self.angular_speed - self.speed_increment, 0.1)
        
        print(f"Speed updated - Linear: {self.linear_speed:.1f} m/s, Angular: {self.angular_speed:.1f} rad/s")
    
    def run_teleop(self):
        """Main teleop loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key is None:
                    continue
                
                # Convert to lowercase for consistency
                key_lower = key.lower()
                
                # Check for quit command
                if key == '\x03':  # Ctrl+C
                    break
                
                # Handle movement keys
                if key_lower in self.key_bindings:
                    linear_x, linear_y, linear_z, angular_z = self.key_bindings[key_lower]
                    self.publish_twist(linear_x, linear_y, linear_z, angular_z)
                
                # Handle speed adjustment keys
                elif key in self.speed_keys:
                    self.adjust_speed(self.speed_keys[key])
                
                # Handle unknown keys
                elif key.isprintable():
                    print(f"Unknown key: '{key}' - Press 'x' to stop, Ctrl+C to quit")
                
                # Small delay to prevent overwhelming the system
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Send stop command before exiting
            self.publish_twist(0, 0, 0, 0)
            print("\nTeleop stopped. Robot should be stationary.")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = TurtleBotTeleop()
        teleop_node.run_teleop()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()