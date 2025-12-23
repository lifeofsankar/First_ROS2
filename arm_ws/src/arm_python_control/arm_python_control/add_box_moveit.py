#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class AddBoxMoveIt(Node):

    def __init__(self):
        super().__init__('add_box_moveit')

        # Publisher to MoveIt planning scene
        self.collision_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            QoSProfile(depth=10)
        )

        # Wait a moment to ensure MoveIt is ready
        self.get_logger().info("Waiting for MoveIt...")
        self.create_timer(1.0, self.add_box_once)

        self.box_added = False

    def add_box_once(self):
        if self.box_added:
            return

        box = CollisionObject()
        box.id = "box1"
        box.header.frame_id = "base_link"

        # Define box shape
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]  # 5cm cube

        # Define box pose
        pose = Pose()
        pose.position.x = 0.35
        pose.position.y = 0.0
        pose.position.z = 0.025  # half height

        box.primitives.append(primitive)
        box.primitive_poses.append(pose)
        box.operation = CollisionObject.ADD

        self.collision_pub.publish(box)
        self.get_logger().info("✅ Box added to MoveIt planning scene")

        self.box_added = True


def main():
    rclpy.init()
    node = AddBoxMoveIt()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
