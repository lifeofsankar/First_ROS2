cmake_minimum_required(VERSION 3.8)
project(tortoisebot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# ROS 2 Messages
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Gazebo Harmonic Integration (CRITICAL)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Robot Description
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# # Install Python scripts (if you have any in src/)
# install(PROGRAMS
#   src/your_script.py  # Remove this line if you don't have Python scripts
#   DESTINATION lib/${PROJECT_NAME}
# )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()