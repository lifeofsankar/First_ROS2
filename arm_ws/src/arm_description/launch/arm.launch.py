from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    # Default URDF path
    default_urdf = "/home/admin/First_ROS2/arm_ws/src/arm_description/urdf/arm.urdf.xacro"

    # Launch argument
    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=default_urdf,
        description='Absolute path to the URDF/XACRO file'
    )

    # Holds the runtime path
    urdf_path = LaunchConfiguration('urdf_path')

    robot_description = {
        'robot_description': Command(['xacro ', urdf_path])
    }
    
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        urdf_path_arg,
        robot_state_pub,
        joint_state_gui,
        rviz
    ])
