cmake_minimum_required(VERSION 3.16)
project(mppi_generic_vendor VERSION 0.9.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(MPPI_GENERIC_VERSION "0.9.0")
set(MPPI_GENERIC_GIT_REPOSITORY
  "https://github.com/ACDSLab/MPPI-Generic.git")
set(MPPI_GENERIC_GIT_TAG
  "b5c8daab6a9157ff9342d5a74838535c1b2431f7")

find_program(MPPI_GENERIC_NVCC_EXECUTABLE nvcc)
set(_cuda_default OFF)
if(MPPI_GENERIC_NVCC_EXECUTABLE)
  set(_cuda_default ON)
endif()

option(
  MPPI_GENERIC_VENDOR_ENABLE_CUDA
  "Fetch and build the pinned MPPI-Generic CUDA dependency"
  ${_cuda_default})
set(_mppi_generic_cuda_arch_default "87")
if(DEFINED ENV{ROBORACER_CUDA_ARCH} AND NOT "$ENV{ROBORACER_CUDA_ARCH}" STREQUAL "")
  set(_mppi_generic_cuda_arch_default "$ENV{ROBORACER_CUDA_ARCH}")
else()
  find_program(_mppi_generic_nvidia_smi nvidia-smi)
  if(_mppi_generic_nvidia_smi)
    execute_process(
      COMMAND "${_mppi_generic_nvidia_smi}" --query-gpu=compute_cap --format=csv,noheader,nounits
      OUTPUT_VARIABLE _mppi_generic_compute_caps
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_mppi_generic_compute_caps)
      string(REGEX MATCH "[0-9]+\\.[0-9]+" _mppi_generic_first_compute_cap "${_mppi_generic_compute_caps}")
      if(_mppi_generic_first_compute_cap)
        string(REPLACE "." "" _mppi_generic_cuda_arch_default "${_mppi_generic_first_compute_cap}")
      endif()
    endif()
  endif()
endif()
set(
  MPPI_GENERIC_CUDA_ARCH "${_mppi_generic_cuda_arch_default}" CACHE STRING
  "CUDA architecture passed to MPPI-Generic (87 is Jetson Orin, 89 is RTX 40 Ada)")
set(
  MPPI_GENERIC_SOURCE_DIR "" CACHE PATH
  "Optional existing MPPI-Generic v0.9.0 source tree for offline builds")

if(MPPI_GENERIC_VENDOR_ENABLE_CUDA)
  if(NOT MPPI_GENERIC_NVCC_EXECUTABLE)
    message(FATAL_ERROR
      "MPPI_GENERIC_VENDOR_ENABLE_CUDA=ON, but nvcc was not found. "
      "Install the JetPack CUDA toolkit or build with the option OFF.")
  endif()

  include(ExternalProject)
  set(_vendor_install "${CMAKE_CURRENT_BINARY_DIR}/vendor_install")

  # MPPI-Generic v0.9.0 uses CMake's legacy FindCUDA path with CMake < 3.24.
  # Ubuntu 22.04 ships CMake 3.22, whose architecture-name table predates
  # Ada (sm_89).  Skip that table and pass nvcc's gencode flag explicitly.
  if(CMAKE_VERSION VERSION_LESS 3.24)
    set(_mppi_cuda_arch_args
      -DMPPI_ARCH_FLAGS=sm_${MPPI_GENERIC_CUDA_ARCH}
      -DCMAKE_CUDA_FLAGS=-gencode=arch=compute_${MPPI_GENERIC_CUDA_ARCH},code=sm_${MPPI_GENERIC_CUDA_ARCH})
  else()
    set(_mppi_cuda_arch_args
      -DMPPI_CUDA_ARCH_LIST=${MPPI_GENERIC_CUDA_ARCH})
  endif()

  if(MPPI_GENERIC_SOURCE_DIR)
    if(NOT EXISTS "${MPPI_GENERIC_SOURCE_DIR}/VERSION")
      message(FATAL_ERROR
        "MPPI_GENERIC_SOURCE_DIR is not an MPPI-Generic source tree: "
        "${MPPI_GENERIC_SOURCE_DIR}")
    endif()
    set(_mppi_source_args
      SOURCE_DIR "${MPPI_GENERIC_SOURCE_DIR}"
      DOWNLOAD_COMMAND ""
      UPDATE_COMMAND "")
  else()
    set(_mppi_source_args
      GIT_REPOSITORY "${MPPI_GENERIC_GIT_REPOSITORY}"
      GIT_TAG "${MPPI_GENERIC_GIT_TAG}"
      GIT_SHALLOW FALSE
      GIT_SUBMODULES_RECURSE TRUE
      UPDATE_COMMAND "")
  endif()

  ExternalProject_Add(mppi_generic_external
    ${_mppi_source_args}
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
      -DMPPI_BUILD_TESTS=OFF
      -DMPPI_BUILD_EXAMPLES=OFF
      -DMPPI_EXPORT_PACKAGE=OFF
      ${_mppi_cuda_arch_args}
    INSTALL_DIR "${_vendor_install}"
    USES_TERMINAL_DOWNLOAD TRUE
    USES_TERMINAL_BUILD TRUE
    USES_TERMINAL_INSTALL TRUE)

  # The external project is built before the ament install step, so its
  # exported MPPI-GenericConfig.cmake and headers are copied into this
  # package's install prefix for downstream find_package() calls.
  install(DIRECTORY "${_vendor_install}/" DESTINATION . USE_SOURCE_PERMISSIONS)
  set(MPPI_GENERIC_VENDOR_BUILT TRUE)
  message(STATUS
    "MPPI-Generic ${MPPI_GENERIC_VERSION} will be built for sm_${MPPI_GENERIC_CUDA_ARCH}")
else()
  set(MPPI_GENERIC_VENDOR_BUILT FALSE)
  message(STATUS
    "nvcc unavailable or CUDA vendor disabled; installing lock metadata only")
endif()

configure_file(
  cmake/mppi_generic_vendor-extras.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/mppi_generic_vendor-extras.cmake"
  @ONLY)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/mppi_generic_vendor-extras.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
install(FILES
  MPPI_GENERIC.lock
  LICENSE.MPPI-Generic
  README.md
  DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS
  "${CMAKE_CURRENT_BINARY_DIR}/mppi_generic_vendor-extras.cmake")
