include(GoogleTest)

add_executable(pathfinder_tests
  benchmark_maps_test.cpp
  distance_test.cpp
  generate_test.cpp
  heuristic_test.cpp
  grid_test.cpp
  instance_test.cpp
  movingai_test.cpp
  parser_robustness_test.cpp
  plan_file_test.cpp
  plan_test.cpp
  prioritized_test.cpp
  reservation_test.cpp
  space_time_astar_test.cpp
  space_time_oracle_test.cpp)
target_link_libraries(pathfinder_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
# Where the tests find their data files (tests/data) and the fetched benchmark maps (data/).
target_compile_definitions(pathfinder_tests PRIVATE PATHFINDER_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
pathfinder_set_warnings(pathfinder_tests)
gtest_discover_tests(pathfinder_tests DISCOVERY_MODE PRE_TEST)

# The day-1 review's tests (tests/review/): each failed at the commit it reviewed.
add_executable(pathfinder_review_tests review/review_test.cpp)
target_link_libraries(pathfinder_review_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
target_compile_definitions(pathfinder_review_tests PRIVATE PATHFINDER_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
pathfinder_set_warnings(pathfinder_review_tests)
gtest_discover_tests(pathfinder_review_tests DISCOVERY_MODE PRE_TEST TEST_PREFIX "review.")
add_test(NAME review.cli COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/review/cli_review_test.sh $<TARGET_FILE:pathfinder_cli>)

# The day-1 second review's tests (tests/review2/): each failed at the commit it reviewed (8b29b30).
add_executable(pathfinder_review2_tests review2/review2_test.cpp)
target_link_libraries(pathfinder_review2_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
pathfinder_set_warnings(pathfinder_review2_tests)
gtest_discover_tests(pathfinder_review2_tests DISCOVERY_MODE PRE_TEST TEST_PREFIX "review2.")

# The CLI's outputs and exit codes.
add_test(NAME cli COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/cli_test.sh $<TARGET_FILE:pathfinder_cli>
  ${CMAKE_CURRENT_SOURCE_DIR}/data)

# The day-2 review's tests (tests/review_d2/): each outside ReviewD2Clean failed at the commit it reviewed (5089fd9).
add_executable(pathfinder_review_d2_tests review_d2/review_d2_test.cpp)
target_link_libraries(pathfinder_review_d2_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
pathfinder_set_warnings(pathfinder_review_d2_tests)
gtest_discover_tests(pathfinder_review_d2_tests DISCOVERY_MODE PRE_TEST TEST_PREFIX "review_d2.")
