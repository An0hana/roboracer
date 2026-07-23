cmake_minimum_required(VERSION 3.8)
project(opponent_tracker)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(roboracer_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(opponent_tracker_core
  src/scan_clusterer.cpp
  src/static_map_filter.cpp
)
target_compile_features(opponent_tracker_core PUBLIC cxx_std_17)
target_include_directories(opponent_tracker_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(opponent_tracker_core nav_msgs)

add_executable(opponent_tracker_node src/opponent_tracker_node.cpp)
target_compile_features(opponent_tracker_node PUBLIC cxx_std_17)
target_link_libraries(opponent_tracker_node opponent_tracker_core)
ament_target_dependencies(
  opponent_tracker_node
  geometry_msgs
  nav_msgs
  rclcpp
  roboracer_msgs
  sensor_msgs
  tf2
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scan_clusterer test/test_scan_clusterer.cpp)
  target_link_libraries(test_scan_clusterer opponent_tracker_core)

  ament_add_gtest(test_static_map_filter test/test_static_map_filter.cpp)
  target_link_libraries(test_static_map_filter opponent_tracker_core)
endif()

install(
  TARGETS opponent_tracker_core opponent_tracker_node
  EXPORT export_opponent_tracker_core
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

ament_export_targets(export_opponent_tracker_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(nav_msgs)

ament_package()
