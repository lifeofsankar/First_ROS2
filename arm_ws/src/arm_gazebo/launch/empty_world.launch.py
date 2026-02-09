import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    # Package name
    package_name='arm_gazebo'
    
    # Launch configurations
    world = LaunchConfiguration('world')
    
    # Path to default world 
    world_path = "/home/admin/First_ROS2/arm_ws/src/arm_gazebo/worlds/empty_world.sdf"
    
    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')
    
    # Launch Robot State Publisher Node
    sdf_path = "/home/admin/First_ROS2/arm_ws/src/arm_description/urdf/panda.urdf.xacro"
    camera_path = "/home/admin/First_ROS2/arm_ws/src/arm_description/urdf/camera.urdf"
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'urdf': sdf_path}.items()
    )
    rsp2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true', 'urdf': camera_path}.items()
    )
    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={'gz_args': '-g '}.items()
    )
    
    # Run the spawner node from the gazebo_ros package. 
    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'arm',
            '-file', sdf_path,
            '-x', '-0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )
    spawn_camera = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'camera',
            '-file', camera_path,
            '-x', '0.0', '-y', '0.5', '-z', '0.0'
        ],
        output='screen'
    )
    # Launch the Gazebo-ROS bridge
    bridge_params = "/home/admin/First_ROS2/arm_ws/src/arm_description/config/gz_bridge.yaml"
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[{'use_sim_time': True}]  # Add this line
    )
    
    static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0','0','0','0','0','base_link','base_footprint'],
        parameters=[{'use_sim_time': True}]  # Add this line
    )
    
    static1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0','0','0','0','0','panda_link0','world'],
        parameters=[{'use_sim_time': True}]  # Add this line
    )
    
    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_rviz,
        declare_world,
        # Launch the nodes
        rsp,
        rsp2,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_entity,
        spawn_camera,
        static1,
    ])