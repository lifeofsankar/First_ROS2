from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Node to publish a static transform for the camera frame
    # This is needed for RViz to know where to display the markers
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_link']
    )

    # Node for the camera publisher
    camera_publisher_node = Node(
        package='face_detect',
        executable='camera_publisher',
        name='camera_publisher'
    )

    # Node for the DNN face detector
    face_detect_node = Node(
        package='face_detect',
        executable='face_detect_node',
        name='face_detect_node',
        output='screen'
    )

    # Node for RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    # Create a LaunchDescription object and add all nodes to it.
    return LaunchDescription([
        static_transform_publisher_node,
        camera_publisher_node,
        face_detect_node,
        rviz2_node
    ])