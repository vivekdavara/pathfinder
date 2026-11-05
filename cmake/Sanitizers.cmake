# PATHFINDER_SANITIZE=address,undefined builds everything, dependencies included, with those sanitizers, so a
# report can't come from a mismatch between instrumented and uninstrumented code. UBSan stops at the first report.
if(PATHFINDER_SANITIZE)
  set(sanitize_flags -fsanitize=${PATHFINDER_SANITIZE} -fno-omit-frame-pointer)
  if(PATHFINDER_SANITIZE MATCHES "undefined")
    list(APPEND sanitize_flags -fno-sanitize-recover=all)
  endif()
  add_compile_options(${sanitize_flags})
  add_link_options(-fsanitize=${PATHFINDER_SANITIZE})
endif()
