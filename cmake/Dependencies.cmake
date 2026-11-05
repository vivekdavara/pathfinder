# Build dependencies, fetched at configure time as release tarballs pinned by SHA-256, so every build compiles exactly
# the same sources. The planning library itself needs only the standard library. SYSTEM makes their headers system
# headers for our targets, so a warning inside them can't fail a -Werror build. The Python module needs pybind11
# and a Python 3.11 or later with its headers.
include(FetchContent)

if(PATHFINDER_BUILD_TESTS)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.tar.gz
    URL_HASH SHA256=6e3191c1455468b3fc35a417fb565c1c5071aee1b7e7f85e30cf48a98d37d8b5
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SYSTEM)
  FetchContent_MakeAvailable(googletest)
endif()

if(PATHFINDER_BUILD_PYTHON)
  # pybind11 lives at github.com/pybind/pybind11 (the "pybind" organisation; PyPI's project page links there).
  # PYBIND11_FINDPYTHON makes it use CMake's FindPython, which honours Python_EXECUTABLE (the presets point it at
  # the project's .venv).
  set(PYBIND11_FINDPYTHON ON CACHE BOOL "" FORCE)
  find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
  FetchContent_Declare(pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v3.0.4.tar.gz
    URL_HASH SHA256=74b6a2c2b4573a400cafb6ecbf60c98df300cd3d0041296b913d02b2cbbb2676
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SYSTEM)
  FetchContent_MakeAvailable(pybind11)
endif()
