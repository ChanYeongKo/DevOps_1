cmake_minimum_required(VERSION 3.16)
project(lidarsim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

#build
add_executable(lidarsim_node 
  src/lidarsim.cpp
  src/main.cpp
  )
ament_target_dependencies(lidarsim_node rclcpp sensor_msgs geometry_msgs std_msgs OpenCV)

include_directories(include)

# Install
install(TARGETS
  lidarsim_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()