cmake_minimum_required(VERSION 3.8)
project(local_costmap)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(local_costmap_core
  src/rolling_costmap.cpp
)
target_compile_features(local_costmap_core PUBLIC cxx_std_17)
target_include_directories(local_costmap_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(local_costmap_node src/local_costmap_node.cpp)
target_compile_features(local_costmap_node PUBLIC cxx_std_17)
target_link_libraries(local_costmap_node local_costmap_core)
ament_target_dependencies(
  local_costmap_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rolling_costmap test/test_rolling_costmap.cpp)
  target_link_libraries(test_rolling_costmap local_costmap_core)
endif()

install(
  TARGETS local_costmap_core local_costmap_node
  EXPORT export_local_costmap_core
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

ament_export_targets(export_local_costmap_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
