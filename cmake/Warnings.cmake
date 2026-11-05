# Warnings for the project's own targets only; the fetched dependencies keep their own flags.
function(pathfinder_set_warnings target)
  set(warnings
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wold-style-cast
    -Wcast-align
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Wdouble-promotion
    -Wimplicit-fallthrough)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    list(APPEND warnings -Wduplicated-cond -Wlogical-op -Wuseless-cast)
  else()
    # Clang only. GCC's version works from the optimiser's view of the code and gave false reports in optimised
    # builds inside libstdc++ (GCC 14) and in defaulted comparisons (GCC 15) in this author's previous C++ project.
    list(APPEND warnings -Wnull-dereference)
  endif()
  if(PATHFINDER_WERROR)
    list(APPEND warnings -Werror)
  endif()
  target_compile_options(${target} PRIVATE ${warnings})
endfunction()
