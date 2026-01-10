import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    # Package paths
    arm_description_pkg = get_package_share_directory('panda_description')
    arm_bringup_pkg = get_package_share_directory('panda_gazebo')
    arm_moveit_pkg = get_package_share_directory('panda_moveit_config')

    # File paths
    urdf_path = os.path.join(
        arm_description_pkg,
        'robots',
        'panda_arm.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        arm_description_pkg,
        'rviz',
        'visualize_franka.rviz'
    )

    controllers_yaml = os.path.join(
        arm_bringup_pkg,
        'config',
        'gazebo_panda_controllers.yaml'
    )

    # Robot description (xacro → urdf)
    robot_description = Command([
        'xacro ',
        urdf_path
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description,
                value_type=str
            ),
            'use_sim_time': True
        }],
        output='screen'
    )


    # ros2_control controller manager
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_yaml,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    # MoveIt move_group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                arm_moveit_pkg,
                'launch',
                'move_group.launch.py'
            )
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group,
        rviz
    ])
