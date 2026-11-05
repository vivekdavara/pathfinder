add_executable(pathfinder_bench bench.cpp)
target_link_libraries(pathfinder_bench PRIVATE pathfinder::pathfinder benchmark::benchmark)
pathfinder_set_warnings(pathfinder_bench)
