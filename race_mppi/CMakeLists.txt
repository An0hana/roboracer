cmake_minimum_required(VERSION 3.8)
project(race_mppi)

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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenMP QUIET)

set(RACE_MPPI_CUDA_ARCHITECTURE "87" CACHE STRING
  "CUDA architecture for race_mppi (87 is Jetson Orin, 89 is Ada)")

# MPPI-Generic v0.9.0 invokes legacy FindCUDA when used with Ubuntu 22.04's
# CMake 3.22.  Its architecture table predates Ada.  Mark detection complete
# and provide nvcc's flag directly, just as the vendor build does.
if(CMAKE_VERSION VERSION_LESS 3.24)
  set(MPPI_ARCH_FLAGS "sm_${RACE_MPPI_CUDA_ARCHITECTURE}")
  if(NOT CMAKE_CUDA_FLAGS MATCHES "compute_${RACE_MPPI_CUDA_ARCHITECTURE}")
    string(APPEND CMAKE_CUDA_FLAGS
      " -gencode=arch=compute_${RACE_MPPI_CUDA_ARCHITECTURE},code=sm_${RACE_MPPI_CUDA_ARCHITECTURE}")
  endif()
endif()

# The vendored dependency advertises whether the exact pinned MPPI-Generic build
# exists.  Merely finding nvcc is not enough to claim that the backend is usable.
find_package(mppi_generic_vendor QUIET)
set(_race_mppi_cuda_default OFF)
if(mppi_generic_vendor_FOUND AND mppi_generic_vendor_CUDA_BUILT)
  set(_race_mppi_cuda_default ON)
endif()
option(RACE_MPPI_ENABLE_CUDA "Build the pinned MPPI-Generic CUDA backend"
  ${_race_mppi_cuda_default})
if(RACE_MPPI_ENABLE_CUDA AND NOT
  (mppi_generic_vendor_FOUND AND mppi_generic_vendor_CUDA_BUILT))
  message(FATAL_ERROR
    "RACE_MPPI_ENABLE_CUDA=ON, but mppi_generic_vendor was not built with CUDA")
endif()
add_library(race_mppi_core
  src/mppi_core.cpp
  src/backend_factory.cpp
)
target_compile_features(race_mppi_core PUBLIC cxx_std_17)
target_include_directories(race_mppi_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(race_mppi_core PRIVATE
  RACE_MPPI_HAS_CUDA_BACKEND=$<BOOL:${RACE_MPPI_ENABLE_CUDA}>)
if(OpenMP_CXX_FOUND)
  target_link_libraries(race_mppi_core PUBLIC OpenMP::OpenMP_CXX)
endif()

if(RACE_MPPI_ENABLE_CUDA)
  if(POLICY CMP0104)
    if(CMAKE_VERSION VERSION_LESS 3.24)
      cmake_policy(SET CMP0104 OLD)
    else()
      cmake_policy(SET CMP0104 NEW)
    endif()
  endif()
  if(NOT CMAKE_VERSION VERSION_LESS 3.24)
    set(CMAKE_CUDA_ARCHITECTURES "${RACE_MPPI_CUDA_ARCHITECTURE}")
  endif()
  enable_language(CUDA)
  target_sources(race_mppi_core PRIVATE src/mppi_cuda_backend.cu)
  target_link_libraries(race_mppi_core PUBLIC MPPI::MPPI)
  set_target_properties(race_mppi_core PROPERTIES
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CUDA_SEPARABLE_COMPILATION ON
  )
  if(NOT CMAKE_VERSION VERSION_LESS 3.24)
    set_property(TARGET race_mppi_core PROPERTY
      CUDA_ARCHITECTURES "${RACE_MPPI_CUDA_ARCHITECTURE}")
  endif()
  message(STATUS
    "Building race_mppi MPPI-Generic v0.9.0 CUDA backend for sm_${RACE_MPPI_CUDA_ARCHITECTURE}")
else()
  message(STATUS "Building race_mppi with CPU backend only")
endif()

add_executable(race_mppi_node src/race_mppi_node.cpp)
target_compile_features(race_mppi_node PUBLIC cxx_std_17)
target_link_libraries(race_mppi_node race_mppi_core)
ament_target_dependencies(race_mppi_node
  ackermann_msgs
  diagnostic_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mppi_core test/test_mppi_core.cpp)
  target_link_libraries(test_mppi_core race_mppi_core)
  ament_add_gtest(test_backend_factory test/test_backend_factory.cpp)
  target_link_libraries(test_backend_factory race_mppi_core)
endif()

install(TARGETS race_mppi_core
  EXPORT export_race_mppi_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS race_mppi_node
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

ament_export_targets(export_race_mppi_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(mppi_generic_vendor)
if(OpenMP_CXX_FOUND)
  ament_export_dependencies(OpenMP)
endif()
ament_package()
