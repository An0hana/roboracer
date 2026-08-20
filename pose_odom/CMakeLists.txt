cmake_minimum_required(VERSION 3.10)
project(pose_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vesc_msgs REQUIRED)

add_executable(pose_odom_node
  src/odom_node.cpp
)
target_include_directories(pose_odom_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(pose_odom_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  vesc_msgs
)

install(TARGETS pose_odom_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Both suites are plain executables with no ROS dependency, so they run in
  # milliseconds and need no test fixture.
  find_package(ament_cmake_gtest QUIET)

  add_executable(pose_filter_test test/pose_filter_test.cpp)
  target_include_directories(pose_filter_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  add_executable(velocity_test test/velocity_test.cpp)
  target_include_directories(velocity_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  include(CTest)
  add_test(NAME pose_filter COMMAND pose_filter_test)
  add_test(NAME velocity COMMAND velocity_test)
endif()

ament_package()
