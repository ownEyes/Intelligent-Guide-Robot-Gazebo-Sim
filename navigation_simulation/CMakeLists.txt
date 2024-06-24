cmake_minimum_required(VERSION 3.5)
project(navigation_simulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(controller_manager REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(ros_gz REQUIRED)

# Uncomment the following section if you have defined messages or services
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourMessage1.msg"
#   "msg/YourMessage2.msg"
#   "srv/YourService1.srv"
# )

# # Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/navigation_sim_demo_node.cpp)

# # Specify libraries or specific dependencies to link the executable target against
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   rclpy
#   std_msgs
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   nav2_core
#   nav2_common
#   ros_gz
# )

# Install the executable
# install(TARGETS
#   ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch files.
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Install config files.
install(DIRECTORY
  param launch rviz2 maps
  DESTINATION share/${PROJECT_NAME}
)

# If your package includes custom headers, install them as follows
# install(DIRECTORY include/
#   DESTINATION include
# )

ament_package()


