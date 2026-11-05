include(GoogleTest)

add_executable(pathfinder_tests
  distance_test.cpp
  generate_test.cpp
  grid_test.cpp
  instance_test.cpp
  movingai_test.cpp
  plan_test.cpp)
target_link_libraries(pathfinder_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
# Where the tests find their data files (tests/data) and the fetched benchmark maps (data/).
target_compile_definitions(pathfinder_tests PRIVATE PATHFINDER_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
pathfinder_set_warnings(pathfinder_tests)
gtest_discover_tests(pathfinder_tests DISCOVERY_MODE PRE_TEST)
