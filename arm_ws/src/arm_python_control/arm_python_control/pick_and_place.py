#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# MoveIt action + message types
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint,
    CollisionObject, AttachedCollisionObject
)

# Geometry + shapes
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Gripper controller
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')

        # MoveIt controller
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

        # Gripper controller
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

    # ----------------------------------------------------------------------
    # MOVE ARM USING JOINT SPACE
    # ----------------------------------------------------------------------
    def move_joints(self, targets, wait=3.0):

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.allowed_planning_time = 5.0
        
        goal.request.execute = True


        constraints = Constraints()

        for joint, position in targets.items():
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = float(position)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.moveit.wait_for_server()
        future = self.moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        time.sleep(wait)

    # ----------------------------------------------------------------------
    # ADD BOX INTO MOVEIT SCENE
    # ----------------------------------------------------------------------
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
        self.get_logger().info("🟦 Added box to planning scene")
        time.sleep(1)

    # ----------------------------------------------------------------------
    # ATTACH BOX TO g_base
    # ----------------------------------------------------------------------
    def attach_box(self):
        attach = AttachedCollisionObject()
        attach.link_name = "g_base"   # TOOL FRAME BASE

        # remove box from world and attach to robot
        attach.object.id = "box1"
        attach.object.operation = CollisionObject.REMOVE

        attach.touch_links = [
            "g_base",
            "gripper_left_finger",
            "gripper_right_finger"
        ]

        self.attach_pub.publish(attach)
        self.get_logger().info("📎 Box attached to g_base")
        time.sleep(1)

    # ----------------------------------------------------------------------
    # DETACH BOX
    # ----------------------------------------------------------------------
    def detach_box(self):
        # 1) Remove attachment
        detach = AttachedCollisionObject()
        detach.object.id = "box1"
        detach.object.operation = CollisionObject.REMOVE  # <-- FIXED

        self.attach_pub.publish(detach)
        self.get_logger().info("📦 Detached box from g_base")
        time.sleep(0.5)

        # 2) Re-add to world
        world = CollisionObject()
        world.id = "box1"
        world.header.frame_id = "base_link"
        world.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]

        pose = Pose()
        pose.position.x = 0.50
        pose.position.y = 0.00
        pose.position.z = 0.025

        world.primitives.append(primitive)
        world.primitive_poses.append(pose)

        self.collision_pub.publish(world)
        self.get_logger().info("📦 Box added back to world")
        time.sleep(1.0)



    # ----------------------------------------------------------------------
    # SET GRIPPER POSITION
    # ----------------------------------------------------------------------
    def set_gripper(self, pos):
        msg = JointTrajectory()
        msg.joint_names = ['gripper_right_finger_joint']

        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start.sec = 1

        msg.points.append(pt)
        self.gripper_pub.publish(msg)

        time.sleep(1.2)

    # ----------------------------------------------------------------------
    # EXECUTE PICK & PLACE
    # ----------------------------------------------------------------------
    def execute(self):
        self.get_logger().info("▶ Pick & Place START")

        # Add object to world
        self.add_box()

        # 1) Move to pre-grasp
        self.set_gripper(0.04)  # open
        self.move_joints({
            'j1': 0.0,
            'j2': -1.0,
            'j3': 1.2,
            'j4': 0.0,
            'j5': 1.0,
            'j6': 0.0
        })

        # 2) Close gripper
        self.set_gripper(0.0)

        # 3) Attach object
        self.attach_box()

        # 4) Lift
        self.move_joints({
            'j1': 0.0,
            'j2': -0.6,
            'j3': 1.0,
            'j4': 0.0,
            'j5': 1.0,
            'j6': 0.0
        })

        # 5) Move to place
        self.move_joints({
            'j1': 1.57,
            'j2': -1.0,
            'j3': 1.2,
            'j4': 0.0,
            'j5': 1.0,
            'j6': 0.0
        })

        # 6) Detach object
        self.detach_box()

        # Open gripper
        self.set_gripper(0.04)

        self.get_logger().info("✅ Pick & Place COMPLETE")


def main():
    rclpy.init()
    node = PickAndPlace()
    node.execute()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
