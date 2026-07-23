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

add_library(ftg_core src/ftg_core.cpp)
target_compile_features(ftg_core PUBLIC cxx_std_17)
target_include_directories(ftg_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(ftg_controller_node src/ftg_controller.cpp)
target_compile_features(ftg_controller_node PUBLIC cxx_std_17)
target_link_libraries(ftg_controller_node ftg_core)
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

install(TARGETS ftg_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
