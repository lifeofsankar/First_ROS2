import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, CollisionObject, AttachedCollisionObject, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class ArmPickPlaceClient(Node):

    def __init__(self):
        super().__init__('arm_pick_place_client')
        
        callback_group = ReentrantCallbackGroup()
        
        self.move_client = ActionClient(
            self, 
            MoveGroup, 
            '/move_action',
            callback_group=callback_group
        )
        
        self.scene_client = self.create_client(
            ApplyPlanningScene, 
            '/apply_planning_scene',
            callback_group=callback_group
        )
        
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Waiting for services...')
        self.move_client.wait_for_server()
        self.scene_client.wait_for_service()
        self.get_logger().info('Services ready!')

    def control_gripper(self, position, duration=1.0):
        """Control gripper: 0.0 (closed) to 0.05 (open)"""
        msg = JointTrajectory()
        msg.joint_names = ['gripper_right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        msg.points.append(point)
        self.gripper_pub.publish(msg)
        
        status = "OPEN" if position > 0.02 else "CLOSED"
        self.get_logger().info(f'Gripper: {status}')

    def add_object_to_scene(self, obj_id, x, y, z, size_x=0.05, size_y=0.05, size_z=0.05):
        """Add a box to the planning scene"""
        collision_object = CollisionObject()
        collision_object.header = Header(frame_id='base_link')
        collision_object.id = obj_id
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size_x, size_y, size_z]
        
        box_pose = Pose()
        box_pose.position.x = x
        box_pose.position.y = y
        box_pose.position.z = z
        box_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        from moveit_msgs.msg import PlanningScene
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'✓ Added "{obj_id}" at ({x:.2f}, {y:.2f}, {z:.2f})')
            return True
        else:
            self.get_logger().error(f'✗ Failed to add "{obj_id}"')
            return False

    def attach_object(self, obj_id, link_name='tool0'):
        """Attach object to gripper"""
        from moveit_msgs.msg import PlanningScene
        
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = obj_id
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = ['tool0', 'g_base', 'gripper_left_finger', 'gripper_right_finger', 'le_1']
        
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'✓ Attached "{obj_id}" to gripper')
            return True
        else:
            self.get_logger().error(f'✗ Failed to attach "{obj_id}"')
            return False

    def detach_object(self, obj_id, link_name='tool0'):
        """Detach object from gripper"""
        from moveit_msgs.msg import PlanningScene
        
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.id = obj_id
        attached_object.object.operation = CollisionObject.REMOVE
        
        planning_scene = PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.is_diff = True
        
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'✓ Detached "{obj_id}" from gripper')
            return True
        else:
            self.get_logger().error(f'✗ Failed to detach "{obj_id}"')
            return False

    def remove_object(self, obj_id):
        """Remove object from scene"""
        from moveit_msgs.msg import PlanningScene
        
        collision_object = CollisionObject()
        collision_object.header = Header(frame_id='base_link')
        collision_object.id = obj_id
        collision_object.operation = CollisionObject.REMOVE
        
        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True
        
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'✓ Removed "{obj_id}"')
            return True
        return False

    def send_cartesian_goal(self, x, y, z, orientation_w=1.0):
        """Send cartesian goal"""
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.allowed_planning_time = 15.0
        goal.request.num_planning_attempts = 20
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = orientation_w

        constraint = PositionConstraint()
        constraint.header.frame_id = 'base_link'
        constraint.link_name = 'tool0'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03]

        constraint.constraint_region.primitives.append(box)
        constraint.constraint_region.primitive_poses.append(pose.pose)
        constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f'→ Target: ({x:.2f}, {y:.2f}, {z:.2f})')
        
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('  Planning...')
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
                
                if result_future.result():
                    result = result_future.result().result
                    error_code = result.error_code.val
                    
                    error_messages = {
                        1: "SUCCESS",
                        -1: "FAILURE",
                        -2: "PLANNING_FAILED",
                        -3: "INVALID_MOTION_PLAN",
                        -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                        -5: "CONTROL_FAILED",
                        -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                        -7: "TIMED_OUT",
                        -10: "NO_IK_SOLUTION",
                        -31: "GOAL_IN_COLLISION"
                    }
                    
                    if error_code == MoveItErrorCodes.SUCCESS:
                        self.get_logger().info('  ✓ Motion complete')
                        return True
                    else:
                        error_msg = error_messages.get(error_code, f"UNKNOWN_ERROR_{error_code}")
                        self.get_logger().error(f'  ✗ Failed: {error_msg}')
                        return False
        
        self.get_logger().error('  ✗ Goal rejected or timeout')
        return False

    def pick_and_place(self):
        """Execute pick and place with SAFER positions"""
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('    PICK AND PLACE DEMO')
        self.get_logger().info('='*60 + '\n')
        
        # More conservative positions
        pick_x, pick_y, pick_z = 0.35, 0.0, 0.1
        place_x, place_y, place_z = 0.30, 0.15, 0.1
        
        self.get_logger().info('SETUP: Adding objects')
        self.add_object_to_scene('table', 0.35, 0.0, -0.02, 0.6, 0.6, 0.04)
        time.sleep(0.5)
        self.add_object_to_scene('target_box', pick_x, pick_y, pick_z, 0.04, 0.04, 0.04)
        time.sleep(1.0)
        
        self.get_logger().info('\nSTEP 1: Opening gripper')
        self.control_gripper(0.05)
        time.sleep(1.5)
        
        self.get_logger().info('\nSTEP 2: Moving above object')
        if not self.send_cartesian_goal(pick_x, pick_y, pick_z + 0.15):
            self.get_logger().error('ABORT: Cannot reach above pick location')
            return
        time.sleep(1.5)
        
        self.get_logger().info('\nSTEP 3: Lowering to object')
        if not self.send_cartesian_goal(pick_x, pick_y, pick_z + 0.06):
            self.get_logger().error('ABORT: Cannot reach pick location')
            return
        time.sleep(1.5)
        
        self.get_logger().info('\nSTEP 4: Gripping')
        self.control_gripper(0.0)
        time.sleep(2.0)
        
        self.get_logger().info('\nSTEP 5: Attaching object')
        if not self.attach_object('target_box', 'tool0'):
            return
        time.sleep(1.0)
        
        self.get_logger().info('\nSTEP 6: Lifting')
        if not self.send_cartesian_goal(pick_x, pick_y, pick_z + 0.15):
            self.get_logger().error('ABORT: Cannot lift')
            return
        time.sleep(1.5)
        
        self.get_logger().info('\nSTEP 7: Moving to place')
        if not self.send_cartesian_goal(place_x, place_y, place_z + 0.15):
            self.get_logger().error('ABORT: Cannot reach place location')
            return
        time.sleep(1.5)
        
        self.get_logger().info('\nSTEP 8: Lowering')
        if not self.send_cartesian_goal(place_x, place_y, place_z + 0.06):
            self.get_logger().error('ABORT: Cannot lower')
            return
        time.sleep(1.5)
        
        self.get_logger().info('\nSTEP 9: Detaching')
        self.detach_object('target_box', 'tool0')
        time.sleep(1.0)
        
        self.get_logger().info('\nSTEP 10: Updating object position')
        self.remove_object('target_box')
        time.sleep(0.5)
        self.add_object_to_scene('target_box', place_x, place_y, place_z, 0.04, 0.04, 0.04)
        time.sleep(1.0)
        
        self.get_logger().info('\nSTEP 11: Releasing')
        self.control_gripper(0.05)
        time.sleep(2.0)
        
        self.get_logger().info('\nSTEP 12: Moving away')
        self.send_cartesian_goal(place_x, place_y, place_z + 0.15)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('    ✓✓✓ COMPLETE ✓✓✓')
        self.get_logger().info('='*60 + '\n')


def main():
    rclpy.init()
    node = ArmPickPlaceClient()
    
    try:
        node.pick_and_place()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()