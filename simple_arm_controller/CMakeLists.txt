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
find_package(moveit_servo REQUIRED)

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
  moveit_servo
)

add_executable(control_test.cpp controllers/control_test.cpp)
ament_target_dependencies(control_test.cpp
  rclcpp
  rclcpp_action
  control_msgs
  moveit_servo
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(TARGETS gazebo_controller webots_controller moveit_controller control_test.cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
