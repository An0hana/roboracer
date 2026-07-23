cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)

add_executable(local_planner_node src/local_planner_node.cpp)
target_compile_features(local_planner_node PUBLIC cxx_std_17)
ament_target_dependencies(
  local_planner_node
  nav_msgs
  rclcpp
  roboracer_msgs
)

install(TARGETS local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
