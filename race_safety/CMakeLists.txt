cmake_minimum_required(VERSION 3.8)
project(race_safety)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(race_safety_core src/safety_core.cpp)
target_compile_features(race_safety_core PUBLIC cxx_std_17)
target_include_directories(race_safety_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(race_safety_node src/race_safety_node.cpp)
target_compile_features(race_safety_node PUBLIC cxx_std_17)
target_link_libraries(race_safety_node race_safety_core)
ament_target_dependencies(
  race_safety_node
  ackermann_msgs
  builtin_interfaces
  diagnostic_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
)

add_executable(vesc_command_adapter_node src/vesc_command_adapter_node.cpp)
target_compile_features(vesc_command_adapter_node PUBLIC cxx_std_17)
target_link_libraries(vesc_command_adapter_node race_safety_core)
ament_target_dependencies(
  vesc_command_adapter_node
  ackermann_msgs
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_safety_core test/test_safety_core.cpp)
  target_link_libraries(test_safety_core race_safety_core)
endif()

install(
  TARGETS race_safety_core race_safety_node vesc_command_adapter_node
  EXPORT export_race_safety_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_race_safety_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
