import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ----------------------------------------------------------------------
    # LOAD URDF
    # ----------------------------------------------------------------------
    desc_pkg = get_package_share_directory('arm_description')
    urdf_file = os.path.join(desc_pkg, 'urdf', 'arm.urdf.xacro')

    robot_description = {
        "robot_description": Command(['xacro ', urdf_file])
    }

    # ----------------------------------------------------------------------
    # START GAZEBO HARMONIC (gz sim)
    # ----------------------------------------------------------------------
    world_file = os.path.join(
        get_package_share_directory("arm_gazebo"),
        "worlds",
        "empty_world.sdf"
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", world_file],
        output="screen"
    )

    # ----------------------------------------------------------------------
    # SPAWN ROBOT INTO GZ SIM
    # ----------------------------------------------------------------------
    spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-topic", "robot_description",
        "-name", "six_axis_arm",
        "-z", "0.2",
        "-P", "0.0",
        "-Y", "0.0"
    ],
    output="screen"
)


    # ----------------------------------------------------------------------
    # ROBOT STATE PUBLISHER
    # ----------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    # ----------------------------------------------------------------------
    # CONTROLLER SPAWNERS
    # ----------------------------------------------------------------------
    js_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen"
            )
        ]
    )

    arm_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
                output="screen"
            )
        ]
    )

    gripper_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller"],
                output="screen"
            )
        ]
    )

    # ----------------------------------------------------------------------
    # RETURN FINAL LAUNCH
    # ----------------------------------------------------------------------
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        js_broadcaster,
        arm_controller,
        gripper_controller,
    ])
