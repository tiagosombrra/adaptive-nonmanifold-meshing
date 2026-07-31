cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

# Project definition
project(ap_mesh VERSION 1.0.0 LANGUAGES CXX)

include(CTest)

# -----------------------------
# Options
# -----------------------------
option(AP_MESH_USE_MPI "Build with MPI support (requires configuring with an MPI compiler wrapper)." OFF)
option(AP_MESH_USE_OPENMP "Enable OpenMP parallelism when available." ON)
option(AP_MESH_ENABLE_LTO "Enable Link Time Optimization (IPO/LTO) in Release builds when supported." ON)
option(AP_MESH_ENABLE_WARNINGS "Enable extra compiler warnings." ON)
option(AP_MESH_WARNINGS_AS_ERRORS "Treat project compiler warnings as errors." OFF)
option(AP_MESH_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer." OFF)
option(AP_MESH_DISABLE_STRICT_ALIASING "Compile with -fno-strict-aliasing (useful if code violates strict aliasing)." ON)
option(AP_MESH_STATIC_MINGW_RUNTIME "Link the MinGW runtime statically for a portable Windows executable." ON)

# Default build type for single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# -----------------------------
# Target
# -----------------------------
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB_RECURSE AP_MESH_SOURCES CONFIGURE_DEPENDS "${SRC_DIR}/*.cpp")

add_executable(ap_mesh)

# English comment: keep the standard on the target (modern CMake)
target_compile_features(ap_mesh PRIVATE cxx_std_17)

target_sources(ap_mesh PRIVATE ${AP_MESH_SOURCES})

target_include_directories(ap_mesh PRIVATE "${INC_DIR}")

target_compile_definitions(ap_mesh PRIVATE AP_MESH_VERSION="${PROJECT_VERSION}")

if(BUILD_TESTING)
  add_executable(
    runtime_config_tests
    tests/runtime_config_tests.cpp
    src/config/runtime_config.cpp
  )
  target_compile_features(runtime_config_tests PRIVATE cxx_std_17)
  target_include_directories(runtime_config_tests PRIVATE "${INC_DIR}")
  add_test(NAME runtime-config-tests COMMAND runtime_config_tests)
  set_tests_properties(
    runtime-config-tests
    PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

# Avoid dependence on whichever MinGW runtime DLLs happen to appear first in
# the reviewer's PATH. Windows system/UCRT dependencies remain dynamic.
if(MINGW AND AP_MESH_STATIC_MINGW_RUNTIME)
  target_link_options(
    ap_mesh
    PRIVATE
      -static
      -static-libgcc
      -static-libstdc++
  )
endif()

set(AP_MESH_WORKDIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_custom_target(run_book_v5_fix_balanced
  COMMAND $<TARGET_FILE:ap_mesh>
          1 1
          ./input_models/book_v5_fix.bp
          h
          output_meshes/book_v5_fix_balanceado
          y
          0.01
          0.01
          2
          0.001
          7
          0.5
          0.75
          0.10
          0.0
          0.25
          0.0005
  DEPENDS ap_mesh
  WORKING_DIRECTORY "${AP_MESH_WORKDIR}"
  USES_TERMINAL
  COMMENT "Run ap_mesh with the balanced book_v5_fix configuration"
)

add_custom_target(run_book_v5_fix_conservative
  COMMAND $<TARGET_FILE:ap_mesh>
          1 1
          ./input_models/book_v5_fix.bp
          h
          output_meshes/book_v5_fix_conservador
          y
          0.01
          0.005
          3
          0.001
          7
          0.5
          0.85
          0.12
          0.0
          0.0
          0.0
  DEPENDS ap_mesh
  WORKING_DIRECTORY "${AP_MESH_WORKDIR}"
  USES_TERMINAL
  COMMENT "Run ap_mesh with the conservative book_v5_fix configuration"
)

add_custom_target(run_book_v5_fix_strict
  COMMAND $<TARGET_FILE:ap_mesh>
          1 1
          ./input_models/book_v5_fix.bp
          h
          output_meshes/book_v5_fix_estrito
          y
          0.01
          0.015
          1
          0.001
          7
          0.5
          0.60
          0.08
          0.0
          0.50
          0.001
  DEPENDS ap_mesh
  WORKING_DIRECTORY "${AP_MESH_WORKDIR}"
  USES_TERMINAL
  COMMENT "Run ap_mesh with the strict book_v5_fix configuration"
)

# -----------------------------
# Warnings / flags
# -----------------------------
if(AP_MESH_ENABLE_WARNINGS)
  foreach(warning_target IN ITEMS ap_mesh runtime_config_tests)
    if(TARGET "${warning_target}")
      if(MSVC)
        target_compile_options("${warning_target}" PRIVATE /W4 /permissive- /FS)
      else()
        target_compile_options(
          "${warning_target}"
          PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wshadow
            -Wconversion
            # Required for legacy callback casts; all other project warnings
            # remain enabled and are promoted by the audit gate.
            -Wno-cast-function-type
        )
      endif()
    endif()
  endforeach()
endif()

if(AP_MESH_WARNINGS_AS_ERRORS)
  foreach(warning_target IN ITEMS ap_mesh runtime_config_tests)
    if(TARGET "${warning_target}")
      if(MSVC)
        target_compile_options("${warning_target}" PRIVATE /WX)
      else()
        target_compile_options("${warning_target}" PRIVATE -Werror)
      endif()
    endif()
  endforeach()
endif()

if(AP_MESH_ENABLE_SANITIZERS)
  if(MSVC)
    message(FATAL_ERROR "AP_MESH_ENABLE_SANITIZERS is supported with GCC or Clang.")
  endif()
  foreach(sanitized_target IN ITEMS ap_mesh runtime_config_tests)
    if(TARGET "${sanitized_target}")
      target_compile_options(
        "${sanitized_target}"
        PRIVATE
          -fsanitize=address,undefined
          -fno-omit-frame-pointer
      )
      target_link_options(
        "${sanitized_target}"
        PRIVATE
          -fsanitize=address,undefined
      )
    endif()
  endforeach()
endif()

# Build-type specific optimizations
if(MSVC)
  target_compile_options(ap_mesh PRIVATE $<$<CONFIG:Release>:/O2>)
else()
  target_compile_options(ap_mesh PRIVATE $<$<CONFIG:Release>:-O3>)
endif()

if(AP_MESH_DISABLE_STRICT_ALIASING AND NOT MSVC)
  target_compile_options(ap_mesh PRIVATE -fno-strict-aliasing)
endif()

# -----------------------------
# OpenMP
# -----------------------------
if(AP_MESH_USE_OPENMP)
  find_package(OpenMP QUIET)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(ap_mesh PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(ap_mesh PRIVATE AP_MESH_HAS_OPENMP=1)
  else()
    message(WARNING "OpenMP requested but not found. Building without OpenMP.")
  endif()
endif()

# -----------------------------
# MPI
# -----------------------------
if(AP_MESH_USE_MPI)
  # English comment: to build with MPI, configure with mpicxx/mpicc wrappers.
  # Example:
  #   cmake -S . -B build-mpi -DAP_MESH_USE_MPI=ON -DCMAKE_CXX_COMPILER=mpicxx -DCMAKE_C_COMPILER=mpicc
  find_package(MPI REQUIRED)
  target_link_libraries(ap_mesh PRIVATE MPI::MPI_CXX)
  target_compile_definitions(ap_mesh PRIVATE AP_MESH_HAS_MPI=1)
endif()

# -----------------------------
# LTO (IPO)
# -----------------------------
if(AP_MESH_ENABLE_LTO AND (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo"))
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
  if(ipo_supported)
    set_property(TARGET ap_mesh PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(WARNING "IPO/LTO requested but not supported: ${ipo_error}")
  endif()
endif()

# -----------------------------
# Diagnostics
# -----------------------------
message(STATUS "System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "MPI: ${AP_MESH_USE_MPI}")
message(STATUS "OpenMP: ${AP_MESH_USE_OPENMP}")
message(STATUS "LTO: ${AP_MESH_ENABLE_LTO}")
message(STATUS "Static MinGW runtime: ${AP_MESH_STATIC_MINGW_RUNTIME}")
message(STATUS "Sanitizers: ${AP_MESH_ENABLE_SANITIZERS}")
