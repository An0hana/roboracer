cmake_minimum_required(VERSION 3.8)
project(robo_cartographer)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/flip_monitor.py
    scripts/start_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()