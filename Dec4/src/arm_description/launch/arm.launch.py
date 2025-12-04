from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():

    # Arguments
    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value='',
        description='/home/admin/Dec4/src/arm_description/urdf/arm.urdf.xacro'
    )

    urdf_path = LaunchConfiguration('urdf_path')

    # Read the URDF at launch time
    with open(os.path.expanduser(urdf_path.perform({})), 'r') as f:
        robot_description_content = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='both'
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        urdf_path_arg,
        robot_state_pub,
        joint_state_gui,
        rviz
    ])
