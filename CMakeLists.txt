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
option(PATHFINDER_BUILD_PYTHON "Build the Python module (pybind11) and run its tests" ${PROJECT_IS_TOP_LEVEL})
option(PATHFINDER_WERROR "Treat compiler warnings as errors" OFF)
set(PATHFINDER_SANITIZE "" CACHE STRING "Sanitizers to build with, e.g. address,undefined (empty for none)")
option(PATHFINDER_COVERAGE "Build with Clang's source-based coverage (make coverage)" OFF)

include(cmake/Warnings.cmake)
include(cmake/Sanitizers.cmake)
if(PATHFINDER_COVERAGE)
  add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
  add_link_options(-fprofile-instr-generate)
endif()

# The planning library: grids, the MovingAI formats, generators, and the plan checker.
add_library(pathfinder STATIC
  src/grid.cpp
  src/distance.cpp
  src/generate.cpp
  src/instance.cpp
  src/movingai.cpp
  src/plan.cpp)
add_library(pathfinder::pathfinder ALIAS pathfinder)
target_include_directories(pathfinder PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(pathfinder PUBLIC cxx_std_20)
# The Python module (a shared library) links it in.
set_target_properties(pathfinder PROPERTIES POSITION_INDEPENDENT_CODE ON)
pathfinder_set_warnings(pathfinder)

# The command-line tool (tools/pathfinder_cli.cpp), built as `pathfinder`.
add_executable(pathfinder_cli tools/pathfinder_cli.cpp)
set_target_properties(pathfinder_cli PROPERTIES OUTPUT_NAME pathfinder)
target_link_libraries(pathfinder_cli PRIVATE pathfinder::pathfinder)
pathfinder_set_warnings(pathfinder_cli)

if(PATHFINDER_BUILD_TESTS OR PATHFINDER_BUILD_PYTHON)
  include(cmake/Dependencies.cmake)
endif()

if(PATHFINDER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(PATHFINDER_BUILD_PYTHON)
  add_subdirectory(python)
endif()
