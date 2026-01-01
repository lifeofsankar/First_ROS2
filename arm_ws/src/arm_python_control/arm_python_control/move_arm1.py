import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class ArmMoveClient(Node):

    def __init__(self):
        super().__init__('arm_move_client')

        # Connect to MoveIt
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def send_joint_goal(self):
        goal = MoveGroup.Goal()

        # 🔹 MUST match MoveIt group name
        goal.request.group_name = 'arm'

        # Planning parameters
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # ---- JOINT CONSTRAINTS ----
        constraints = Constraints()

        joint_targets = {
            'j1': 0.0,
            'j2': 1.2,   # SAFE (inside limits)
            'j3': 0.0,
            'j4': 0.0,
            'j5': 0.0,
            'j6': 0.0
        }

        for name, position in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info('Waiting for MoveIt...')
        self.client.wait_for_server()

        self.get_logger().info('Sending joint goal')
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = ArmMoveClient()
    node.send_joint_goal()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
