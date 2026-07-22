cmake_minimum_required(VERSION 3.8)
project(race_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(state_estimator_bridge src/state_estimator_bridge.cpp)
ament_target_dependencies(
  state_estimator_bridge geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

install(TARGETS state_estimator_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DEPLOYMENT_CHECKLIST.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
