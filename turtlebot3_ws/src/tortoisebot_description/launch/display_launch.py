import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('tortoisebot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'turtlebot3_waffle.urdf')

    # Convert XACRO → URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'rviz2.rviz')

    return LaunchDescription([

        # 1. Joint State Publisher (publishes /joint_states)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # 2. Robot State Publisher (publishes TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # 3. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
