cmake_minimum_required(VERSION 3.8)
project(race_manager)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(race_state_machine src/race_state_machine.cpp)
target_compile_features(race_state_machine PUBLIC cxx_std_17)
target_include_directories(race_state_machine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(race_manager_node src/race_manager_node.cpp)
target_compile_features(race_manager_node PUBLIC cxx_std_17)
target_link_libraries(race_manager_node race_state_machine)
ament_target_dependencies(
  race_manager_node
  geometry_msgs
  nav_msgs
  rclcpp
  roboracer_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_race_state_machine test/test_race_state_machine.cpp)
  target_link_libraries(test_race_state_machine race_state_machine)
endif()

install(
  TARGETS race_state_machine race_manager_node
  EXPORT export_race_state_machine
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

ament_export_targets(export_race_state_machine HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
