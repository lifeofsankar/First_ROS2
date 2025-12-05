import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tortoise_bot = get_package_share_directory('tortoise_bot')
    urdf_file = os.path.join(pkg_tortoise_bot, 'urdf', 'simple_bot.urdf')
    world_file = os.path.join(pkg_tortoise_bot, 'worlds', 'empty.sdf')

    # Gazebo (Harmonic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )


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

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])

# then return LaunchDescription([... , delayed_spawn, ...])

    

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': True
        }],
        output='screen'
    )

    # ROS–Gazebo Bridges
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static TF publisher (lidar → base)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher,
        cmd_vel_bridge,
        lidar_bridge,
        odom_bridge,
        static_tf_publisher
    ])
