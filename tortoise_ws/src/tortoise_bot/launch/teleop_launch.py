import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file that starts Gazebo Harmonic, spawns the robot with LIDAR, 
    connects the controls, and starts LIDAR processing.
    """
    # Paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tortoise_bot = get_package_share_directory('tortoise_bot')
    urdf_file = os.path.join(pkg_tortoise_bot, 'urdf', 'simple_bot.urdf')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'tortoise_bot',
            '-z', '0.3',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Start Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read(), 'use_sim_time': True}],
        output='screen'
    )

    # Start ROS-Gazebo Bridge for cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # Start ROS-Gazebo Bridge for LIDAR
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # Start LIDAR Processor Node
    lidar_processor = Node(
        package='tortoise_bot',
        executable='lidar_processor',
        name='lidar_processor',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # The Launch Description
    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher,
        bridge,
        lidar_bridge,
        lidar_processor
    ])