 cmake_minimum_required(VERSION 3.16)
  project(dxl_rapi5)

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)

  include_directories(include)
  include_directories(/usr/local/include/dynamixel_sdk)

  add_executable(dxl_rapi5 src/dxl.cpp src/sub.cpp)
  ament_target_dependencies(dxl_rapi5 rclcpp geometry_msgs)
  target_link_libraries(dxl_rapi5 dxl_x64_cpp)

  install(TARGETS
    dxl_rapi5
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()
