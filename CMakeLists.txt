cmake_minimum_required(VERSION 3.8)
project(ros2_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# --- FIX IS HERE ---
# We tell CMake to compile 'src/lifecycle_sensor.cpp' 
# but name the final program 'lifecycle_node' so the test passes.
add_executable(lifecycle_node src/lifecycle_sensor.cpp)
# -------------------

ament_target_dependencies(lifecycle_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

install(TARGETS
  lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()