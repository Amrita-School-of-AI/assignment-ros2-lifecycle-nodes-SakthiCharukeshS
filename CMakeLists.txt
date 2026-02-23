cmake_minimum_required(VERSION 3.8)
project(ros2_lifecycle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies (Crucial: rclcpp_lifecycle)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Add Executable
add_executable(lifecycle_talker src/lifecycle_node.cpp)

# 3. Link Libraries
ament_target_dependencies(lifecycle_talker
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# 4. Install
install(TARGETS
  lifecycle_talker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()