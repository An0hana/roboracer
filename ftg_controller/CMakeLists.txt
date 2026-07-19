cmake_minimum_required(VERSION 3.8)
project(ftg_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ftg_controller_node src/ftg_controller.cpp)
target_compile_features(ftg_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(
  ftg_controller_node
  ackermann_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS ftg_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
