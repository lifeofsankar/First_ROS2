from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    world = os.path.join(
        get_package_share_directory('arm_gazebo'),
        'worlds',
        'empty_world.sdf'
    )

    urdf = os.path.join(
        get_package_share_directory('arm_description'),
        'urdf',
        'arm.urdf.xacro'
    )

    return LaunchDescription([

        # 1️⃣ Gazebo (system process, publishes /clock)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen'
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),


        # 2️⃣ Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': os.popen(f'xacro {urdf}').read(),
                'use_sim_time': True
            }],
            output='screen'
        ),

        # 3️⃣ Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'six_axis_arm',
                '-z', '0.2'
            ],
            output='screen'
        ),

        # 4️⃣ Controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen'
        ),
    ])
