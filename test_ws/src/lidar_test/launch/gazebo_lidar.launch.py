from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get URDF path
    urdf_path = os.path.join(
        get_package_share_directory('lidar_test'),
        'lidar_test',
        'robot_with_lidar.urdf'
    )

    return LaunchDescription([
        # Launch Gazebo Harmonic with empty world
        ExecuteProcess(
            cmd=['gz', 'sim', '--verbose'],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        Node(
            package='ros_gz_sim',  # bridge package for Gazebo Harmonic
            executable='create',
            arguments=['-name', 'my_robot', '-file', urdf_path],
            output='screen'
        ),

        # Start the LIDAR listener node
        Node(
            package='lidar_test',  # ✅ Corrected from 'my_lidar_test'
            executable='lidar_listener',
            name='lidar_listener',
            output='screen'
        )
    ])
