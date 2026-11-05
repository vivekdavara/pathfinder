# Build dependencies, fetched at configure time as release tarballs pinned by SHA-256, so every build compiles exactly
# the same sources. The planning library itself needs only the standard library. SYSTEM makes their headers system
# headers for our targets, so a warning inside them can't fail a -Werror build.
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

