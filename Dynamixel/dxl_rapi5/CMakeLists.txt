cmake_minimum_required(VERSION 3.16)
project(dxl_rapi5)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Dynamixel SDK (Raspberry Pi 5에 설치되어 있어야 함)
find_package(dynamixel_sdk REQUIRED)

include_directories(
  include
)

add_executable(dxl_rapi5 src/dxl.cpp src/sub.cpp)
ament_target_dependencies(dxl_rapi5 rclcpp geometry_msgs dynamixel_sdk)

install(TARGETS
  dxl_rapi5
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
