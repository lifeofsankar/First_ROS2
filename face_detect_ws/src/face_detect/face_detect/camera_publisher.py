import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher = self.create_publisher(Image, '/camera/image_raw',10)
        self.bridge = CvBridge()
        self.get_logger().info("CameraPublisherNode started")

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open WebCam")
            exit()

        timer_period =1.0/30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()