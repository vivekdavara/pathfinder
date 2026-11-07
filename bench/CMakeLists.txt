add_executable(pathfinder_bench bench.cpp)
target_link_libraries(pathfinder_bench PRIVATE pathfinder::pathfinder benchmark::benchmark)
pathfinder_set_warnings(pathfinder_bench)
# Where the benchmarks find the fetched MovingAI maps (data/); the ones that need them are skipped without.
target_compile_definitions(pathfinder_bench PRIVATE PATHFINDER_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
