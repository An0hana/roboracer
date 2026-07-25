cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(local_planner_core src/frenet_planner.cpp)
target_compile_features(local_planner_core PUBLIC cxx_std_17)
target_include_directories(local_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(local_planner_node src/local_planner_node.cpp)
target_compile_features(local_planner_node PUBLIC cxx_std_17)
target_link_libraries(local_planner_node local_planner_core)
ament_target_dependencies(
  local_planner_node
  geometry_msgs
  nav_msgs
  rclcpp
  roboracer_msgs
  std_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frenet_planner test/test_frenet_planner.cpp)
  target_link_libraries(test_frenet_planner local_planner_core)
endif()

install(
  TARGETS local_planner_core local_planner_node
  EXPORT export_local_planner_core
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

ament_export_targets(export_local_planner_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
