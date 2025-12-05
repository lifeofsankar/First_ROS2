from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'resolution': 0.05,
                'max_laser_range': 10.0,
                'minimum_time_interval': 0.5,
                'transform_publish_period': 0.02,
                'map_update_interval': 1.0,
                'use_scan_matching': True,
                'minimum_travel_distance': 0.2,
                'minimum_travel_heading': 0.2,
                'do_loop_closing': True,
                'solver_plugin': 'solver_plugins::CeresSolver',
            }]
        )
    ])