import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('tortoisebot_description')
    pkg_gazebo = get_package_share_directory('tortoisebot_gazebo')
   
    # File Names 
    urdf_file_name = 'turtlebot3_waffle.urdf'
    world_file_name = 'custom_obstacle_world.world'
    rviz_file_name = 'rviz2.rviz'
    
    # File Paths
    urdf_path = os.path.join(pkg_description, 'urdf', urdf_file_name)
    world_path = os.path.join(pkg_gazebo, 'worlds', world_file_name)
    rviz_config_file = os.path.join(pkg_description, 'rviz', rviz_file_name)
    
    # Read URDF
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        
        # 1. Start Gazebo (NO FLAGS - keep it simple)
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen',
        ),
        
        # 2. Start Bridge immediately (doesn't need delay)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # 3. Wait 5 seconds, then start core nodes
        TimerAction(
            period=5.0,
            actions=[
                # Robot State Publisher
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[
                        {'robot_description': robot_description},
                        {'use_sim_time': True}
                    ],
                    output='screen'
                ),
                
                # Static TF: map->odom
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_to_odom',
                    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                    parameters=[{'use_sim_time': True}],
                ),
                
                # RViz
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
            ]
        ),
        
        # 4. Wait 8 seconds, then spawn robot
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_robot',
                    output='screen',
                    arguments=[
                        '-topic', 'robot_description',
                        '-name', 'tortoisebot',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.2'
                    ],
                )
            ]
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='teleop_twist_keyboard',
                    executable='teleop_twist_keyboard',
                    name='teleop_keyboard',
                    output='screen',
                    prefix='xterm -e',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

    ])