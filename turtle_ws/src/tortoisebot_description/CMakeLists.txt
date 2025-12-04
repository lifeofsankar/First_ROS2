cmake_minimum_required(VERSION 3.8)
project(tortoisebot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Add these for Gazebo Harmonic integration (RECOMMENDED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# -----------------------------------------------------------
# Install ALL files
# -----------------------------------------------------------
install(
  DIRECTORY urdf launch rviz meshes config
  DESTINATION share/${PROJECT_NAME}
)

# Optional: Install Python scripts if you have any
# install(PROGRAMS
#   scripts/your_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------------------------------------------
# Export package
# -----------------------------------------------------------
ament_package()