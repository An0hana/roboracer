cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(state_machine_core src/race_state_machine.cpp)
target_compile_features(state_machine_core PUBLIC cxx_std_17)
target_include_directories(state_machine_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(state_machine_node src/state_machine_node.cpp)
target_compile_features(state_machine_node PUBLIC cxx_std_17)
target_link_libraries(state_machine_node state_machine_core)
ament_target_dependencies(
  state_machine_node
  geometry_msgs
  nav_msgs
  rclcpp
  roboracer_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_race_state_machine test/test_race_state_machine.cpp)
  target_link_libraries(test_race_state_machine state_machine_core)
endif()

install(
  TARGETS state_machine_core state_machine_node
  EXPORT export_state_machine_core
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

ament_export_targets(export_state_machine_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
