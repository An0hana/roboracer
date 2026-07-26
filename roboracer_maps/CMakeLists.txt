cmake_minimum_required(VERSION 3.8)
project(roboracer_maps)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/convert_optimizer_raceline.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
