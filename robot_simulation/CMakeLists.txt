cmake_minimum_required(VERSION 3.5)
project(robot_simulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(controller_manager REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz REQUIRED)
find_package(cmd_vel_mux REQUIRED)

install(DIRECTORY launch models param urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/robot_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
