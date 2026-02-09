from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command


def generate_launch_description():

    # Package name
    package_name = FindPackageShare("arm_description")

    # Default robot description if none is specified
    urdf_path = "/home/admin/First_ROS2/arm_ws/src/arm_description/urdf/panda.urdf.xacro"
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true')

    declare_urdf = DeclareLaunchArgument(
            name='sdf', default_value=urdf_path,
            description='Path to the robot description file')

    # Create a robot state publisher 
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 50.0
        }],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        declare_urdf,
        declare_use_sim_time,
        robot_state_pub
    ])