cmake_minimum_required(VERSION 3.8)
project(race_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)

add_executable(race_manager_node src/race_manager_node.cpp)
target_compile_features(race_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(
  race_manager_node
  nav_msgs
  rclcpp
  roboracer_msgs
)

install(TARGETS race_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
