#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get our package directory
    pkg_dir = get_package_share_directory('lidar_test')
    
    # Path to world file
    world_file = os.path.join(pkg_dir, 'worlds', 'lidar_test.world')
    
    # Start Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -v 4'],
        }.items()
    )
    
    # Bridge to connect Gazebo and ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    
    # Our Python program
    lidar_listener = Node(
        package='lidar_test',
        executable='lidar_listener',
        name='lidar_listener',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        bridge,
        lidar_listener,
    ])
