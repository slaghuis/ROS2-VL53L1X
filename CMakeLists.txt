cmake_minimum_required(VERSION 3.5)
project(vl53l1x)

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

add_executable(vl53l1x_node src/vl53l1x_node.cpp src/vl53l1x.cpp)
ament_target_dependencies(vl53l1x_node rclcpp sensor_msgs)
target_include_directories(vl53l1x_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  vl53l1x_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
