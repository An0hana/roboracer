cmake_minimum_required(VERSION 3.8)
project(opponent_tracker)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(opponent_tracker_node src/opponent_tracker_node.cpp)
target_compile_features(opponent_tracker_node PUBLIC cxx_std_17)
ament_target_dependencies(
  opponent_tracker_node
  nav_msgs
  rclcpp
  roboracer_msgs
  sensor_msgs
)

install(TARGETS opponent_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
