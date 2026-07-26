cmake_minimum_required(VERSION 3.8)
project(mppi_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    "$<$<COMPILE_LANGUAGE:CXX>:-Wall>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Wextra>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>"
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenMP QUIET)

set(MPPI_CONTROLLER_CUDA_ARCHITECTURE "87" CACHE STRING
  "CUDA architecture for mppi_controller (87 is Jetson Orin, 89 is Ada)")

# MPPI-Generic v0.9.0 invokes legacy FindCUDA when used with Ubuntu 22.04's
# CMake 3.22.  Its architecture table predates Ada.  Mark detection complete
# and provide nvcc's flag directly, just as the vendor build does.
if(CMAKE_VERSION VERSION_LESS 3.24)
  set(MPPI_ARCH_FLAGS "sm_${MPPI_CONTROLLER_CUDA_ARCHITECTURE}")
  if(NOT CMAKE_CUDA_FLAGS MATCHES "compute_${MPPI_CONTROLLER_CUDA_ARCHITECTURE}")
    string(APPEND CMAKE_CUDA_FLAGS
      " -gencode=arch=compute_${MPPI_CONTROLLER_CUDA_ARCHITECTURE},code=sm_${MPPI_CONTROLLER_CUDA_ARCHITECTURE}")
  endif()
endif()

# The vendored dependency advertises whether the exact pinned MPPI-Generic build
# exists.  Merely finding nvcc is not enough to claim that the backend is usable.
find_package(mppi_generic_vendor QUIET)
option(MPPI_CONTROLLER_ENABLE_CUDA "Build the pinned MPPI-Generic CUDA backend"
  ON)
if(MPPI_CONTROLLER_ENABLE_CUDA AND NOT
  (mppi_generic_vendor_FOUND AND mppi_generic_vendor_CUDA_BUILT))
  message(FATAL_ERROR
    "MPPI_CONTROLLER_ENABLE_CUDA=ON, but mppi_generic_vendor was not built with CUDA")
endif()
add_library(mppi_controller_core
  src/mppi_core.cpp
  src/backend_factory.cpp
)
target_compile_features(mppi_controller_core PUBLIC cxx_std_17)
target_include_directories(mppi_controller_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(mppi_controller_core PRIVATE
  MPPI_CONTROLLER_HAS_CUDA_BACKEND=$<BOOL:${MPPI_CONTROLLER_ENABLE_CUDA}>)
if(OpenMP_CXX_FOUND)
  target_link_libraries(mppi_controller_core PUBLIC OpenMP::OpenMP_CXX)
endif()

if(MPPI_CONTROLLER_ENABLE_CUDA)
  if(POLICY CMP0104)
    if(CMAKE_VERSION VERSION_LESS 3.24)
      cmake_policy(SET CMP0104 OLD)
    else()
      cmake_policy(SET CMP0104 NEW)
    endif()
  endif()
  if(NOT CMAKE_VERSION VERSION_LESS 3.24)
    set(CMAKE_CUDA_ARCHITECTURES "${MPPI_CONTROLLER_CUDA_ARCHITECTURE}")
  endif()
  enable_language(CUDA)
  target_sources(mppi_controller_core PRIVATE src/mppi_cuda_backend.cu)
  target_link_libraries(mppi_controller_core PUBLIC MPPI::MPPI)
  set_target_properties(mppi_controller_core PROPERTIES
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CUDA_SEPARABLE_COMPILATION ON
  )
  if(CMAKE_VERSION VERSION_LESS 3.24)
    # The explicit -gencode flag above is authoritative on Ubuntu 22.04.
    # Disable CMake 3.22's pre-Ada architecture table for this target.
    set_property(TARGET mppi_controller_core PROPERTY CUDA_ARCHITECTURES OFF)
  else()
    set_property(TARGET mppi_controller_core PROPERTY
      CUDA_ARCHITECTURES "${MPPI_CONTROLLER_CUDA_ARCHITECTURE}")
  endif()
  message(STATUS
    "Building mppi_controller MPPI-Generic v0.9.0 CUDA backend for sm_${MPPI_CONTROLLER_CUDA_ARCHITECTURE}")
else()
  message(STATUS "Building mppi_controller with CPU backend only")
endif()

add_executable(mppi_controller_node src/mppi_controller_node.cpp)
target_compile_features(mppi_controller_node PUBLIC cxx_std_17)
target_link_libraries(mppi_controller_node mppi_controller_core)
ament_target_dependencies(mppi_controller_node
  ackermann_msgs
  diagnostic_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mppi_core test/test_mppi_core.cpp)
  target_link_libraries(test_mppi_core mppi_controller_core)
  ament_add_gtest(test_backend_factory test/test_backend_factory.cpp)
  target_link_libraries(test_backend_factory mppi_controller_core)
endif()

install(TARGETS mppi_controller_core
  EXPORT export_mppi_controller_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mppi_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/generate_raceline.py
  scripts/extract_raceline_from_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_mppi_controller_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(mppi_generic_vendor)
if(OpenMP_CXX_FOUND)
  ament_export_dependencies(OpenMP)
endif()
ament_package()
