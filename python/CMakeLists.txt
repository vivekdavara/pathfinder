# The Python package: pathfinder/__init__.py and the compiled pathfinder._core next to it, both in
# build/<preset>/python/pathfinder, so PYTHONPATH=build/<preset>/python imports it without installing anything.
set(package_dir ${PROJECT_BINARY_DIR}/python/pathfinder)

pybind11_add_module(pathfinder_python MODULE bindings.cpp)
set_target_properties(pathfinder_python PROPERTIES
  OUTPUT_NAME _core
  LIBRARY_OUTPUT_DIRECTORY ${package_dir})
target_link_libraries(pathfinder_python PRIVATE pathfinder::pathfinder)
target_compile_definitions(pathfinder_python PRIVATE PATHFINDER_VERSION="${PROJECT_VERSION}")
pathfinder_set_warnings(pathfinder_python)

configure_file(pathfinder/__init__.py ${package_dir}/__init__.py COPYONLY)
# The pure-Python parts of the package, next to it.
configure_file(pathfinder/simulator.py ${package_dir}/simulator.py COPYONLY)

if(PATHFINDER_BUILD_TESTS)
  # The pytest suite (tests/python) against the module just built, with the interpreter it was built for.
  add_test(NAME python.pytest
    COMMAND ${Python_EXECUTABLE} -m pytest -q -p no:cacheprovider ${PROJECT_SOURCE_DIR}/tests/python)
  set_tests_properties(python.pytest PROPERTIES
    ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/python;PATHFINDER_SOURCE_DIR=${PROJECT_SOURCE_DIR}")
  # The example runs end to end and checks itself (it exits 1 if anything it checks is wrong).
  add_test(NAME python.example
    COMMAND ${Python_EXECUTABLE} ${PROJECT_SOURCE_DIR}/examples/independent_paths.py --agents 40 --seed 7)
  set_tests_properties(python.example PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/python")
  add_test(NAME python.example.prioritized
    COMMAND ${Python_EXECUTABLE} ${PROJECT_SOURCE_DIR}/examples/prioritized_vs_independent.py --agents 30 --seed 1)
  set_tests_properties(python.example.prioritized PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/python")
endif()
