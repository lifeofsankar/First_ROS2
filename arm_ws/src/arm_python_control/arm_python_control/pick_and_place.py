#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    CollisionObject,
    AttachedCollisionObject
)

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class PickAndPlaceMoveIt(Node):

    def __init__(self):
        super().__init__('pick_and_place_moveit')

        # MoveIt action
        self.moveit = ActionClient(self, MoveGroup, '/move_action')

        # Planning scene publishers
        self.collision_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

        self.attach_pub = self.create_publisher(
            AttachedCollisionObject,
            '/attached_collision_object',
            10
        )

    # ------------------ MOVE ARM (JOINT SPACE) ------------------
    def move_joints(self, joints, wait=3.0):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.allowed_planning_time = 5.0

        constraints = Constraints()

        for name, pos in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.moveit.wait_for_server()
        future = self.moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(wait)

    # ------------------ ADD OBJECT ------------------
    def add_box(self):
        box = CollisionObject()
        box.id = "box1"
        box.header.frame_id = "base_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]

        pose = Pose()
        pose.position.x = 0.35
        pose.position.y = 0.0
        pose.position.z = 0.025

        box.primitives.append(primitive)
        box.primitive_poses.append(pose)
        box.operation = CollisionObject.ADD

        self.collision_pub.publish(box)
        self.get_logger().info("🟦 Box added to MoveIt")
        time.sleep(1.0)

    # ------------------ ATTACH (PICK) ------------------
    def attach_box(self):
        attach = AttachedCollisionObject()
        attach.link_name = "tool0"
        attach.object.id = "box1"
        attach.object.operation = attach.object.ADD
        attach.touch_links = [
            "tool0",
            "gripper_left_finger",
            "gripper_right_finger"
        ]

        self.attach_pub.publish(attach)
        self.get_logger().info("📎 Box attached")
        time.sleep(1.0)

    # ------------------ DETACH (PLACE) ------------------
    def detach_box(self):
        detach = AttachedCollisionObject()
        detach.object.id = "box1"
        detach.object.operation = detach.object.REMOVE

        self.attach_pub.publish(detach)
        self.get_logger().info("📦 Box detached")
        time.sleep(1.0)

    # ------------------ PICK AND PLACE SEQUENCE ------------------
    def execute(self):
        self.get_logger().info("▶ Starting MoveIt Pick & Place")

        # Add object
        self.add_box()

        # Pre-grasp
        self.move_joints({
            'j1': 0.0, 'j2': -1.0, 'j3': 1.2,
            'j4': 0.0, 'j5': 1.0, 'j6': 0.0
        })

        # Pick (attach)
        self.attach_box()

        # Lift
        self.move_joints({
            'j1': 0.0, 'j2': -0.6, 'j3': 0.8,
            'j4': 0.0, 'j5': 0.8, 'j6': 0.0
        })

        # Move to place
        self.move_joints({
            'j1': 1.57, 'j2': -1.0, 'j3': 1.2,
            'j4': 0.0, 'j5': 1.0, 'j6': 0.0
        })

        # Place (detach)
        self.detach_box()

        self.get_logger().info("✅ Pick & Place finished")


def main():
    rclpy.init()
    node = PickAndPlaceMoveIt()
    node.execute()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
