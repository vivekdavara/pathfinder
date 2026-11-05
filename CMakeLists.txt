cmake_minimum_required(VERSION 3.25)

project(pathfinder
  VERSION 0.1.0
  DESCRIPTION "Multi-agent path finding on grid maps"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PATHFINDER_BUILD_TESTS "Build the GoogleTest suite" ${PROJECT_IS_TOP_LEVEL})
option(PATHFINDER_WERROR "Treat compiler warnings as errors" OFF)
set(PATHFINDER_SANITIZE "" CACHE STRING "Sanitizers to build with, e.g. address,undefined (empty for none)")

include(cmake/Warnings.cmake)
include(cmake/Sanitizers.cmake)

# The planning library: grids, the MovingAI formats, generators, and the plan checker.
add_library(pathfinder
  src/grid.cpp
  src/distance.cpp
  src/generate.cpp
  src/instance.cpp
  src/movingai.cpp)
add_library(pathfinder::pathfinder ALIAS pathfinder)
target_include_directories(pathfinder PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pathfinder PUBLIC cxx_std_20)
pathfinder_set_warnings(pathfinder)

if(PATHFINDER_BUILD_TESTS)
  include(cmake/Dependencies.cmake)
  enable_testing()
  add_subdirectory(tests)
endif()
