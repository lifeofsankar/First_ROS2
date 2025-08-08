#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        
