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
    # pkg_slam = get_package_share_directory('tortoisebot_slam')

    # URDF
    urdf_file_name = 'turtlebot3_waffle.urdf'
    urdf_path = os.path.join(pkg_description, 'urdf', urdf_file_name)

        
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # World
    world_path = os.path.join(pkg_gazebo, 'worlds', 'custom_obstacle_world.world')

    # RViz
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'rviz2.rviz')

    # EKF Config
    # ekf_config = os.path.join(pkg_description, 'config', 'ekf.yaml')

    # SLAM Config
    # slam_config = os.path.join(pkg_slam, 'config', 'slam.yaml')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
        
    return LaunchDescription([
        
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
        
        # DeclareLaunchArgument(
        #     'x_pose',
        #     default_value='0.0',
        #     description='Initial X position'
        # ),
        # DeclareLaunchArgument(
        #     'y_pose',
        #     default_value='0.0',
        #     description='Initial Y position'
        # ),

        # Start Gazebo Harmonic with custom world
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world'), '-r'],
            output='screen',
            shell=False,
            emulate_tty=True,
        ),

        # ROS-Gazebo Bridge for ALL sensors (CRITICAL FOR MAPPING)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                # LiDAR - MOST IMPORTANT FOR MAPPING
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # IMU
                # '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                # Cmd_vel (robot control)
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # Clock (simulation time)
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                # Odometry
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                # WOrld 
                '/world/custom_obstacle_mapping_world/model/tortoisebot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # Delay nodes so Gazebo loads first
        TimerAction(
            period=3.0,
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


                # EKF Node (Robot Localization)
                # Node(
                #     package='robot_localization',
                #     executable='ekf_node',
                #     name='ekf_filter_node',
                #     output='screen',
                #     parameters=[
                #         ekf_config,
                #         {'use_sim_time': True}
                #     ],
                #     remappings=[
                #         ('odometry/filtered', 'odom')
                #     ]
                # ),

                # SLAM Toolbox Node
                # Node(
                #     package='slam_toolbox',
                #     executable='async_slam_toolbox_node',
                #     name='slam_toolbox',
                #     output='screen',
                #     parameters=[
                #         slam_config,
                #         {'use_sim_time': True},
                #         {'odom_frame': 'odom'},
                #         {'map_frame': 'map'},
                #         {'base_frame': 'base_link'},
                #         {'scan_topic': '/scan'},
                #         {'mode': 'mapping'},
                #         {'resolution': 0.05},
                #         {'max_laser_range': 10.0}
                #     ]
                # )
            ]
        ),

        # Spawn robot in Gazebo
        TimerAction(
            period=5.0,
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
                        '-z', '0.1'
                    ],
                )
            ]
        ),
    ])