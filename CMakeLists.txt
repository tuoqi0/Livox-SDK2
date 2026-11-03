cmake_minimum_required(VERSION 3.0)

project(livox_sdk2)

set(CMAKE_CXX_STANDARD 11)

message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
endif()

if (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif(UNIX)

add_subdirectory(sdk_core)
# Samples are standalone executables that pull in extra dependencies and are
# not used by the driver; off by default so a workspace build stays quiet.
option(LIVOX_SDK2_BUILD_SAMPLES "Build the SDK sample programs" OFF)
if(LIVOX_SDK2_BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

# Export a CMake package config so dependents (and colcon) can locate this
# build instead of a system-wide install. Added for the workspace build.
install(FILES
  cmake/Livox-SDK2Config.cmake
  cmake/Livox-SDK2ConfigVersion.cmake
  DESTINATION share/Livox-SDK2/cmake)

# Tests use the workspace's existing installed doctest entry. Standalone SDK
# builds do not acquire a robot_core production dependency.
if(BUILD_TESTING AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
  enable_testing()
  find_package(robot_core CONFIG REQUIRED)
  include(${robot_core_DIR}/robot_coreTests.cmake)
  foreach(contract IN ITEMS lifecycle resources)
    robot_core_add_test(test_sdk_${contract} test/test_sdk_${contract}.cpp)
    target_include_directories(test_sdk_${contract} PRIVATE include sdk_core 3rdparty 3rdparty/spdlog)
    target_link_libraries(test_sdk_${contract} livox_lidar_sdk_shared)
    set_property(TARGET test_sdk_${contract} APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--export-dynamic")
    set_tests_properties(test_sdk_${contract} PROPERTIES TIMEOUT 30)
  endforeach()
  add_library(test_sdk_wire_layout OBJECT test/test_sdk_wire_layout.cpp)
  target_include_directories(test_sdk_wire_layout PRIVATE include sdk_core 3rdparty 3rdparty/spdlog)
endif()
