cmake_minimum_required(VERSION 3.8)
project(arm_gazebo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gripper_controllers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Install launch, urdf, config, worlds
install(
  DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install xacro files if needed
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()