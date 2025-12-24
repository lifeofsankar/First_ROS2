#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_task_constructor_python import (
    PipelinePlanner,
    Task,
    stages
)

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class MTCPickPlace(Node):

    def __init__(self):
        super().__init__("mtc_pick_place")

        self.task = Task()
        self.task.auto_config()

        self.arm_group = "arm"
        self.eef = "tool0"
        self.object = "box1"

        self.create_task()

    def create_task(self):

        self.task.add(stages.CurrentState("current state"))

        # -------------------------------------------------------------
        # 1) ADD COLLISION OBJECT (THE BOX)
        # -------------------------------------------------------------
        add_box = stages.AddCollisionObject("add box")
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]

        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0.35
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.025

        add_box.add_primitive(box, box_pose, self.object)
        self.task.add(add_box)

        # -------------------------------------------------------------
        # 2) APPROACH OBJECT (CARTESIAN)
        # -------------------------------------------------------------
        approach = stages.MoveTo("approach object", PipelinePlanner())
        approach.group = self.arm_group

        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = 0.35
        target.pose.position.y = 0.0
        target.pose.position.z = 0.12

        approach.set_pose(target)
        self.task.add(approach)

        # -------------------------------------------------------------
        # 3) PICK STAGE (ATTACH OBJECT)
        # -------------------------------------------------------------
        pick_stage = stages.AttachObject("pick object", self.object)
        pick_stage.link = self.eef
        self.task.add(pick_stage)

        # -------------------------------------------------------------
        # 4) LIFT UP
        # -------------------------------------------------------------
        lift = stages.MoveRelative("lift", PipelinePlanner())
        lift.group = self.arm_group
        lift.set_direction({"z": 0.12})
        self.task.add(lift)

        # -------------------------------------------------------------
        # 5) MOVE TO PLACE POSITION
        # -------------------------------------------------------------
        move_to_place = stages.MoveTo("move to place", PipelinePlanner())
        move_to_place.group = self.arm_group

        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.pose.position.x = 0.10
        place_pose.pose.position.y = 0.30
        place_pose.pose.position.z = 0.20

        move_to_place.set_pose(place_pose)
        self.task.add(move_to_place)

        # -------------------------------------------------------------
        # 6) PLACE STAGE (DETACH)
        # -------------------------------------------------------------
        place_stage = stages.DetachObject("place object", self.object)
        place_stage.link = self.eef
        self.task.add(place_stage)

        # -------------------------------------------------------------
        # 7) RETRACT ARM
        # -------------------------------------------------------------
        retreat = stages.MoveRelative("retreat", PipelinePlanner())
        retreat.group = self.arm_group
        retreat.set_direction({"z": -0.10})
        self.task.add(retreat)

    def plan_and_execute(self):
        self.get_logger().info("⏳ Planning MTC task...")

        if not self.task.plan():
            self.get_logger().error("❌ Planning failed")
            return

        self.get_logger().info("▶ Executing task...")
        self.task.execute()


def main():
    rclpy.init()
    node = MTCPickPlace()
    node.plan_and_execute()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
