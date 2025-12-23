import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class ArmCartesianClient(Node):

    def __init__(self):
        super().__init__('arm_cartesian_client')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def send_cartesian_goal(self):
        goal = MoveGroup.Goal()

        # 🔹 Must match your MoveIt group
        goal.request.group_name = 'arm'

        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # ---- TARGET POSE ----
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'

        pose.pose.position.x = 0.35
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.35

        pose.pose.orientation.w = 1.0  # no rotation

        # ---- POSITION CONSTRAINT ----
        constraint = PositionConstraint()
        constraint.header.frame_id = 'base_link'
        constraint.link_name = 'tool0'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        constraint.constraint_region.primitives.append(box)
        constraint.constraint_region.primitive_poses.append(pose.pose)
        constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(constraint)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info('Waiting for MoveIt...')
        self.client.wait_for_server()

        self.get_logger().info('Sending Cartesian goal')
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = ArmCartesianClient()
    node.send_cartesian_goal()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
