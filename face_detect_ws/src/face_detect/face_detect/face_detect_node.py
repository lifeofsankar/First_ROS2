#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import time


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        #Rviz Marker
        self.marker_pub = self.create_publisher(MarkerArray, '/faces', 10)
        
        self.bridge = CvBridge()
        self.face_count = 0  # NEW: Simple counter
        
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        
        self.get_logger().info('FaceDetectNode started')
        
    def image_callback(self, msg):
        
        frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # IMPROVED: Better detection settings
        faces, _, levelWeights = self.face_cascade.detectMultiScale3(
            gray,
            scaleFactor = 1.1,
            minNeighbors = 8,
            minSize=(50, 50),
            outputRejectLevels = True
        )
        if len(levelWeights) > 0:
            # IMPROVED: Better face processing with confidence
            for (x, y, w, h), confidence in zip(faces, levelWeights):
                # Simple confidence based on face size
                normalized_confidence = min(100.0, (confidence / 4.0) * 100)
            
            # Choose color based on confidence
                if normalized_confidence > 75:
                    color = (0, 255, 0)  # Green for high confidence
                elif normalized_confidence > 40:
                    color = (0, 255, 255)  # Yellow for medium confidence
                else:
                    color = (0, 0, 255)    # Red for low confidence
            
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            cv2.putText(frame, f'{normalized_confidence:.0f}%', (x, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
        cv2.imshow("Face Detection", frame)
        cv2.waitKey(1)
        
        # IMPROVED: Count faces and log sometimes
        self.face_count += len(faces)
        if self.face_count % 50 == 0 and self.face_count > 0:
            self.get_logger().info(f'Total faces detected: {self.face_count}')
        
        # Your original marker code (keeping it simple)
        marker_array = MarkerArray()
        for i, (x,y,w,h) in enumerate(faces):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = w / 100
            marker.scale.y = h /100
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.pose.position.x = 0.5
            marker.pose.position.y = (x + w/2 - frame.shape[1]/2)/500.0
            marker.pose.position.z = (y + h/2 - frame.shape[0]/2)/500.0
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)
                    
def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # FIXED: The typo

if __name__ == '__main__':
    main()