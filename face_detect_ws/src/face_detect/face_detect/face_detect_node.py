#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/faces', 10)
        self.bridge = CvBridge()

        # DNN Model
        # path to your package's share directory
        package_share_directory = get_package_share_directory('face_detect')
        
        # paths to the model files
        prototxt_path = os.path.join(package_share_directory, 'models', 'deploy.prototxt.txt')
        model_path = os.path.join(package_share_directory, 'models', 'res10_300x300_ssd_iter_140000.caffemodel')
        
        # Loading model
        self.get_logger().info('Loading DNN face detector model...')
        self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
        self.get_logger().info('Model loaded successfully.')

        # filter weak detections
        self.confidence_threshold = 0.57

        self.get_logger().info('FaceDetectNode started.')
        
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        (h, w) = frame.shape[:2]

        # DNN Detection Logic
        # Create blob from image then pass it to network
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
                                     (300, 300), (104.0, 177.0, 123.0))
        
        self.net.setInput(blob)
        detections = self.net.forward()

        detected_faces = []

        # Looping
        for i in range(0, detections.shape[2]):
            # Get confidence of detection
            confidence = detections[0, 0, i, 2]

            # Filter out weak detections
            if confidence > self.confidence_threshold:
                # Compute coordinates of bounding box
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                
                # Detected face list
                face_w = endX - startX
                face_h = endY - startY
                detected_faces.append((startX, startY, face_w, face_h))

                # Draw the bounding box and confidence
                text = f"{confidence * 100:.2f}%"
                y = startY - 10 if startY - 10 > 10 else startY + 10
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, text, (startX, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Face Detection (DNN)", frame)
        cv2.waitKey(1)

        # Publish RViz markers
        marker_array = MarkerArray()
        for i, (x, y, w, h) in enumerate(detected_faces):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = w / 100.0
            marker.scale.y = h / 100.0
            marker.scale.z = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker.pose.position.x = 0.5
            marker.pose.position.y = -(x + w/2 - frame.shape[1]/2) / 500.0
            marker.pose.position.z = -(y + h/2 - frame.shape[0]/2) / 500.0
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
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()