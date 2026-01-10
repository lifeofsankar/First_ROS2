cmake_minimum_required(VERSION 3.8)
project(panda_gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gripper_controllers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(
  PROGRAMS
  
    #scripts/ik_executor.py
    #scripts/cartesian_keyboard_control.py
    scripts/initial_pose_pub.py
    scripts/plan_around_objects.cpp
  DESTINATION lib/${PROJECT_NAME}
)
