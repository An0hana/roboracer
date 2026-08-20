cmake_minimum_required(VERSION 3.8)
project(safety_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(safety_controller_core src/safety_core.cpp)
target_compile_features(safety_controller_core PUBLIC cxx_std_17)
target_include_directories(safety_controller_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(safety_controller_node src/safety_controller_node.cpp)
target_compile_features(safety_controller_node PUBLIC cxx_std_17)
target_link_libraries(safety_controller_node safety_controller_core)
ament_target_dependencies(
  safety_controller_node
  ackermann_msgs
  builtin_interfaces
  diagnostic_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  roboracer_msgs
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_safety_core test/test_safety_core.cpp)
  target_link_libraries(test_safety_core safety_controller_core)
endif()

install(
  TARGETS safety_controller_core safety_controller_node
  EXPORT export_safety_controller_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_safety_controller_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
