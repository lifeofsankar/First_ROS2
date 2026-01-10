cmake_minimum_required(VERSION 3.5)
project(panda_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(panda_description REQUIRED)
find_package(panda_gazebo REQUIRED)

install(
  DIRECTORY config launch srdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_srdf_tests test/srdf_tests.py)
endif()

ament_package()
