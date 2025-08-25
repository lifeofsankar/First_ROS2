#!/usr/bin/env python3
"""
Simple TurtleBot Web Control Bridge
Just copy, run, and go!
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading

# ROS2 Controller
class SimpleController(Node):
    def __init__(self):
        super().__init__('web_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move(self, command, speed=0.2):
        msg = Twist()
        
        if command == 'forward':
            msg.linear.x = speed
        elif command == 'backward':
            msg.linear.x = -speed
        elif command == 'left':
            msg.angular.z = speed * 2
        elif command == 'right':
            msg.angular.z = -speed * 2
        elif command == 'stop':
            pass  # All zeros
            
        self.publisher.publish(msg)
        return True

# Web API
app = Flask(__name__)
CORS(app)
controller = None

@app.route('/api/robot/command', methods=['POST'])
def command():
    data = request.get_json()
    cmd = data.get('command', 'stop')
    speed = data.get('parameters', {}).get('speed', 1.0) * 0.2
    
    controller.move(cmd, speed)
    return jsonify({'success': True, 'command': cmd})

@app.route('/api/robot/status', methods=['GET'])
def status():
    return jsonify({'status': 'active'})

def run_web():
    app.run(host='0.0.0.0', port=3000, debug=False)

# Main
def main():
    global controller
    rclpy.init()
    controller = SimpleController()
    
    # Start web server
    web_thread = threading.Thread(target=run_web, daemon=True)
    web_thread.start()
    
    print("🤖 TurtleBot Web Control Ready!")
    print("🌐 Open web interface, set API to: http://localhost:3000/api")
    print("⏹️  Ctrl+C to stop")
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
