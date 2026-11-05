#!/bin/sh
# The day-1 review (commit 5ef5763): the CLI reported success when its output couldn't be written.
#   tests/review/cli_review_test.sh <pathfinder binary>
# tools/pathfinder_cli.cpp never checks std::cout after writing, so `gen-map` / `gen-scen` with a closed (or full)
# stdout exit 0 ("all is well") though nothing was written; a script redirecting into a file on a full disk gets a
# truncated .map or .scen and a success status.
set -u
cli=$1
failures=0
"$cli" gen-map empty 3 3 >&- 2>/dev/null
code=$?
if [ "$code" -eq 0 ]; then
  echo "FAIL: gen-map with a closed stdout exited 0"
  failures=$((failures + 1))
fi
[ "$failures" -eq 0 ]
