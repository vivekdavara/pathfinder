#!/bin/sh
# The day-2 second review (at 4db3e63): ReviewD2bClean checks of `pathfinder plan`, which passed there. The CLI test
# (tests/cli_test.sh) runs every --order on an instance where they all agree, so swapping "longest" and "shortest"
# survives it; here they disagree. The dead-end corridor off a small room: agent 0 (distance 2) parked in the
# corridor first walls in agent 1 (distance 5); agent 1 first, both get in.
#   tests/review_d2b/cli_review_d2b_test.sh <pathfinder binary>
set -u
cli=$1
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
failures=0
fail() {
  echo "FAIL: $*"
  failures=$((failures + 1))
}
printf 'type octile\nheight 2\nwidth 5\nmap\n.....\n..@@@\n' > "$tmp/room.map"
printf 'version 1\n0\troom.map\t5\t2\t1\t0\t3\t0\t2\n0\troom.map\t5\t2\t0\t1\t4\t0\t4.41421356\n' > "$tmp/room.scen"
# order exit-code first-line-prefix
check() {
  "$cli" plan "$tmp/room.map" "$tmp/room.scen" 2 --order "$1" --out "$tmp/$1.plan" > "$tmp/out" 2> "$tmp/err"
  got=$?
  [ "$got" -eq "$2" ] || { fail "--order $1: exit $got, expected $2"; sed 's/^/  | /' "$tmp/out" "$tmp/err"; }
  grep -q "^$3" "$tmp/out" || { fail "--order $1: output doesn't start with '$3'"; sed 's/^/  | /' "$tmp/out"; }
}
check scenario 1 "not solved: agent 1 found no path around the 1 agent planned before it"
check shortest 1 "not solved: agent 1 found no path around the 1 agent planned before it"
check longest 0 "solved 2 agents: sum of costs"
check reverse 0 "solved 2 agents: sum of costs"
[ -e "$tmp/longest.plan" ] || fail "--order longest wrote no plan"
[ ! -e "$tmp/shortest.plan" ] || fail "--order shortest wrote a plan for an unsolved instance"
"$cli" validate "$tmp/room.map" "$tmp/room.scen" "$tmp/longest.plan" > "$tmp/out" 2>&1 || fail "the longest-first plan isn't valid"
if [ "$failures" -ne 0 ]; then
  echo "$failures failure(s)"
  exit 1
fi
echo "all review_d2b CLI checks passed"
