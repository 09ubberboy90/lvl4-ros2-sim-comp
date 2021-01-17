cmake_minimum_required(VERSION 3.5.0)
project(simple_arm_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sim_action_server REQUIRED)
find_package(simple_interface REQUIRED)
find_package(ignition-transport9 REQUIRED)
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(gazebo_controller controllers/gazebo_controller.cpp)
ament_target_dependencies(gazebo_controller
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(webots_controller controllers/webots_controller.cpp)
ament_target_dependencies(webots_controller
  rclcpp
  rclcpp_action
  control_msgs
)
add_executable(moveit_controller controllers/moveit_controller.cpp)
ament_target_dependencies(moveit_controller
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  sim_action_server
  simple_interface
)
add_executable(vr_controller controllers/vr_controller.cpp)
ament_target_dependencies(vr_controller
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  sim_action_server
)
add_executable(ign_controller controllers/ign_controller.cpp)
ament_target_dependencies(ign_controller
  rclcpp
  rclcpp_action
  control_msgs
  ignition-transport9
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(TARGETS gazebo_controller webots_controller moveit_controller ign_controller vr_controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
