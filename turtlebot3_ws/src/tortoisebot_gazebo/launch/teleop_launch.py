import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Directories
    pkg_description = get_package_share_directory('tortoisebot_description')
    pkg_gazebo = get_package_share_directory('tortoisebot_gazebo')

    # URDF
    urdf_file_name = 'turtlebot3_waffle.urdf'
    urdf_path = os.path.join(pkg_description, 'urdf', urdf_file_name)

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # World
    world_path = os.path.join(pkg_gazebo, 'worlds', 'custom_obstacle_world.world')

    # RViz
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'rviz2.rviz')

    # Load URDF
    try:
        with open(urdf_path, 'r') as infp:
            robot_description = infp.read()
    except FileNotFoundError:
        print(f"ERROR: URDF file not found at {urdf_path}")
        raise
        
    return LaunchDescription([
        
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='World file path'
        ),
        
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial X position'
        ),
        
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial Y position'
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world'), '-r'],
            output='screen',
            shell=False,
            emulate_tty=True,
        ),

        # Spawn Robot
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_robot',
                    output='screen',
                    arguments=[
                        '-file', urdf_path,
                        '-name', 'tortoisebot',
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', '0.0'
                    ],
                )
            ]
        ),

        # ROS-Gazebo Bridge
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge',
                    arguments=[
                        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                        '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                    ],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                ),
            ]
        ),
        
        # Robot State Publisher
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[
                        {'robot_description': robot_description},
                        {'use_sim_time': use_sim_time}
                    ],
                    output='screen'
                ),

                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                ),
            ]
        ),

        # Keyboard Teleop (MAIN ADDITION)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='teleop_twist_keyboard',
                    executable='teleop_twist_keyboard',
                    name='teleop_twist_keyboard',
                    output='screen',
                    prefix='xterm -e',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
            ]
        ),

        # RViz (Optional)
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                ),
            ]
        ),
    ])