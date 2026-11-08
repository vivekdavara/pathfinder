include(GoogleTest)

add_executable(pathfinder_tests
  benchmark_maps_test.cpp
  cbs_oracle_test.cpp
  cbs_test.cpp
  conflicts_test.cpp
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

# The day-2 second review's tests (tests/review_d2b/, at 4db3e63): all ReviewD2bClean (C++ and CLI), checks that
# passed there. Its two bugs are pytest tests in tests/python/review_d2b/, each failing at 4db3e63.
add_executable(pathfinder_review_d2b_tests review_d2b/review_d2b_test.cpp)
target_link_libraries(pathfinder_review_d2b_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
pathfinder_set_warnings(pathfinder_review_d2b_tests)
gtest_discover_tests(pathfinder_review_d2b_tests DISCOVERY_MODE PRE_TEST TEST_PREFIX "review_d2b.")
add_test(NAME review_d2b.cli COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/review_d2b/cli_review_d2b_test.sh
  $<TARGET_FILE:pathfinder_cli>)

# The day-2 third review's tests (tests/review_d2c/, at 98f60c4): all ReviewD2cClean, checks that passed there (it
# found no bug). Its Python checks are in tests/python/review_d2c/.
add_executable(pathfinder_review_d2c_tests review_d2c/review_d2c_test.cpp)
target_link_libraries(pathfinder_review_d2c_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
pathfinder_set_warnings(pathfinder_review_d2c_tests)
gtest_discover_tests(pathfinder_review_d2c_tests DISCOVERY_MODE PRE_TEST TEST_PREFIX "review_d2c.")

# The day-3 review's tests (tests/review_d3/, at ee6d2ef): each ReviewD3 test failed there; ReviewD3Clean checks
# passed there and are kept for their coverage.
add_executable(pathfinder_review_d3_tests review_d3/review_d3_test.cpp)
target_link_libraries(pathfinder_review_d3_tests PRIVATE pathfinder::pathfinder GTest::gtest_main)
pathfinder_set_warnings(pathfinder_review_d3_tests)
gtest_discover_tests(pathfinder_review_d3_tests DISCOVERY_MODE PRE_TEST TEST_PREFIX "review_d3.")
