#!/bin/sh
# The CLI's outputs and exit codes (0 fine, 1 a problem with the instance, 2 usage or unreadable input).
#   tests/cli_test.sh <pathfinder binary> <tests/data directory>
set -u
cli=$1
data=$2
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
failures=0

fail() {
  echo "FAIL: $*"
  failures=$((failures + 1))
}

# expect <exit code> <command...>: runs it with stdout to $tmp/out and stderr to $tmp/err.
expect() {
  want=$1
  shift
  "$@" > "$tmp/out" 2> "$tmp/err"
  got=$?
  if [ "$got" -ne "$want" ]; then
    fail "exit $got, expected $want: $*"
    sed 's/^/  | /' "$tmp/out" "$tmp/err"
  fi
}

# has <file> <text>: the file contains the text as a fixed string.
has() {
  grep -qF -- "$2" "$1" || { fail "'$2' not in $1 output:"; sed 's/^/  | /' "$1"; }
}

# is_line <file> <text>: the file has the text as a whole line.
is_line() {
  grep -qxF -- "$2" "$1" || { fail "no line '$2' in $1 output:"; sed 's/^/  | /' "$1"; }
}

# Generated maps, exactly.
expect 0 "$cli" gen-map empty 3 2
printf 'type octile\nheight 2\nwidth 3\nmap\n...\n...\n' | cmp -s - "$tmp/out" || fail "gen-map empty 3 2"
expect 0 "$cli" gen-map warehouse 2 3 3 1 1 1
printf 'type octile\nheight 5\nwidth 13\nmap\n.............\n.@@@.@@@.@@@.\n.............\n.@@@.@@@.@@@.\n.............\n' |
  cmp -s - "$tmp/out" || fail "gen-map warehouse 2 3 3 1 1 1"

# The same seed, the same file; info on it.
"$cli" gen-map random 32 32 20 1 > "$tmp/a.map" && "$cli" gen-map random 32 32 20 1 > "$tmp/b.map" ||
  fail "gen-map random"
cmp -s "$tmp/a.map" "$tmp/b.map" || fail "gen-map random isn't deterministic"
expect 0 "$cli" info "$tmp/a.map"
has "$tmp/out" "size 32x32"
has "$tmp/out" "passable 819 of 1024"

# info on a hand-written map.
expect 0 "$cli" info "$data/small.map"
has "$tmp/out" "size 5x4"
has "$tmp/out" "passable 17 of 20"
has "$tmp/out" "pieces 1"
has "$tmp/out" "largest piece 17"

# A generated scenario checks out on its map, with octile lengths throughout.
expect 0 "$cli" gen-scen "$tmp/a.map" 25 7
has "$tmp/out" "version 1"
has "$tmp/out" "a.map"
cp "$tmp/out" "$tmp/a.scen"
expect 0 "$cli" check "$tmp/a.map" "$tmp/a.scen"
has "$tmp/out" "instance of 25 agents on a 32x32 map: valid"
has "$tmp/out" "stated optimal lengths equal to the octile distance: 25 of 25"
expect 0 "$cli" gen-scen "$tmp/a.map" 3 7 other-name.map
has "$tmp/out" "other-name.map"
expect 0 "$cli" check "$data/small.map" "$data/small.scen" 2
has "$tmp/out" "instance of 2 agents"
expect 0 "$cli" check "$data/small.map" "$data/small.scen"
has "$tmp/out" "stated optimal lengths equal to the octile distance: 3 of 3"

# Problems with the instance: exit 1.
printf 'version 1\n0\tm.map\t5\t4\t0\t0\t4\t3\t7\n0\tm.map\t5\t4\t4\t0\t4\t3\t3\n' > "$tmp/shared_goal.scen"
expect 1 "$cli" check "$data/small.map" "$tmp/shared_goal.scen"
has "$tmp/out" "invalid instance: agent 1's goal (4, 3) is also agent 0's goal"
printf 'type octile\nheight 1\nwidth 3\nmap\n.@.\n' > "$tmp/split.map"
printf 'version 1\n0\tsplit.map\t3\t1\t0\t0\t2\t0\t2\n' > "$tmp/split.scen"
expect 1 "$cli" check "$tmp/split.map" "$tmp/split.scen"
has "$tmp/out" "unreachable goals: 1 agents (first: agent 0)"
printf 'version 1\n0\tsmall.map\t5\t4\t0\t0\t4\t3\t7\n' > "$tmp/manhattan.scen"
expect 0 "$cli" check "$data/small.map" "$tmp/manhattan.scen"
has "$tmp/out" "stated optimal lengths equal to the octile distance: 0 of 1"
has "$tmp/out" "agent 0: stated 7, octile 6.41421"

# validate: a valid plan (exit 0), an invalid one (exit 1), and plans that can't be read (exit 2).
expect 0 "$cli" validate "$data/small.map" "$data/small.scen" "$data/small.plan"
has "$tmp/out" "valid plan for 2 agents: sum of costs 13, makespan 7"
# Head on along the top row: they meet in (2, 0) at t=2 (one vertex conflict; passing through each other from the
# same cell isn't a swap) ...
printf 'version 1\nagents 2\n0 (0,0) (1,0) (2,0) (3,0) (4,0) (4,1) (4,2) (4,3)\n1 (4,0) (3,0) (2,0) (1,0) (0,0) (0,1) (0,2)\n' > "$tmp/meet.plan"
expect 1 "$cli" validate "$data/small.map" "$data/small.scen" "$tmp/meet.plan"
has "$tmp/out" "invalid plan: 1 problem"
has "$tmp/out" "agents 0 and 1 are both at (2, 0) at t=2"
# ... and with agent 1 waiting once, they swap (2, 0) and (3, 0) instead.
printf 'version 1\nagents 2\n0 (0,0) (1,0) (2,0) (3,0) (4,0) (4,1) (4,2) (4,3)\n1 (4,0) (3,0) (3,0) (2,0) (1,0) (0,0) (0,1) (0,2)\n' > "$tmp/swap.plan"
expect 1 "$cli" validate "$data/small.map" "$data/small.scen" "$tmp/swap.plan"
has "$tmp/out" "invalid plan: 1 problem"
has "$tmp/out" "agents 0 and 1 swap (2, 0) and (3, 0) between t=2 and t=3"
printf 'version 1\nagents 4\n0 (0,0)\n1 (1,0)\n2 (2,0)\n3 (3,0)\n' > "$tmp/four.plan"
expect 2 "$cli" validate "$data/small.map" "$data/small.scen" "$tmp/four.plan"
has "$tmp/err" "the plan has 4 agents, the scenario only 3 entries"
printf 'version 1\nagents 1\n0 (5,0)\n' > "$tmp/outside.plan"
expect 2 "$cli" validate "$data/small.map" "$data/small.scen" "$tmp/outside.plan"
is_line "$tmp/err" "pathfinder: $tmp/outside.plan: line 3: agent 0's cell (5,0) at t=0 is outside the 5x4 map"
expect 1 "$cli" validate "$data/small.map" "$tmp/shared_goal.scen" "$data/small.plan"
has "$tmp/out" "invalid instance"

# Usage errors and unreadable files: exit 2.
expect 2 "$cli"
has "$tmp/out" "usage:"
expect 0 "$cli" --help
has "$tmp/out" "pathfinder gen-scen MAP AGENTS SEED"
expect 2 "$cli" frobnicate
has "$tmp/err" "unknown command 'frobnicate'"
expect 2 "$cli" gen-map
expect 2 "$cli" gen-map hexagonal 3 3
expect 2 "$cli" gen-map empty 3
expect 2 "$cli" gen-map empty 0 3
has "$tmp/err" "outside 1..10000"
expect 2 "$cli" gen-map empty -3 3
expect 2 "$cli" gen-map empty 3x 3
expect 2 "$cli" gen-map random 8 8 101 1
has "$tmp/err" "0..100"
expect 2 "$cli" gen-map random 8 8 nan 1
expect 2 "$cli" gen-map random 8 8 1e1 1
expect 2 "$cli" gen-map random 8 8 0x10 1
expect 2 "$cli" gen-map random 8 8 " 5" 1
expect 0 "$cli" gen-map random 8 8 12.5 1
expect 2 "$cli" gen-map warehouse 1 1 1 1 0 1
expect 2 "$cli" info "$data/short_row.map"
is_line "$tmp/err" "pathfinder: $data/short_row.map: line 6: row 1 has 2 characters, expected 3"
expect 2 "$cli" info "$data/no_such_file.map"
has "$tmp/err" "can't open the file"
expect 2 "$cli" info "$data"
is_line "$tmp/err" "pathfinder: $data: can't read the file"
expect 2 "$cli" gen-scen "$data/small.map" 17 1
has "$tmp/err" "17 agents need at least 18"
expect 2 "$cli" gen-scen "$data/small.map" 3 1 "bad name"
expect 2 "$cli" check "$data/small.map" "$data/small.scen" 4
has "$tmp/err" "has 3 entries, not 4"
expect 2 "$cli" check "$data/small.map"

# plan: solved (exit 0), with the plan written and valid; every order gives the same here.
expect 0 "$cli" plan "$data/small.map" "$data/small.scen" 3 --out "$tmp/planned.plan"
has "$tmp/out" "solved 3 agents: sum of costs 20 (lower bound 20), makespan 7, 20 expansions"
expect 0 "$cli" validate "$data/small.map" "$data/small.scen" "$tmp/planned.plan"
is_line "$tmp/out" "valid plan for 3 agents: sum of costs 20, makespan 7"
for order in scenario reverse longest shortest; do
  expect 0 "$cli" plan "$data/small.map" "$data/small.scen" 3 --order "$order" --time-limit 0
  has "$tmp/out" "solved 3 agents: sum of costs 20"
done
expect 0 "$cli" plan "$data/small.map" "$data/small.scen" 0
has "$tmp/out" "solved 0 agents: sum of costs 0 (lower bound 0), makespan 0"

# plan: not solved (exit 1). A dead-end corridor off a small room: agent 0 parks half-way in before agent 1 can get
# past it; planned the other way round, both get in.
printf 'type octile\nheight 2\nwidth 5\nmap\n.....\n..@@@\n' > "$tmp/room.map"
printf 'version 1\n0\troom.map\t5\t2\t1\t0\t3\t0\t2\n0\troom.map\t5\t2\t0\t1\t4\t0\t4.41421356\n' > "$tmp/room.scen"
expect 1 "$cli" plan "$tmp/room.map" "$tmp/room.scen" 2 --out "$tmp/room.plan"
has "$tmp/out" "not solved: agent 1 found no path around the 1 agent planned before it, "
[ ! -e "$tmp/room.plan" ] || fail "plan wrote a plan file for an unsolved instance"
expect 0 "$cli" plan "$tmp/room.map" "$tmp/room.scen" 2 --order reverse --out "$tmp/room.plan"
expect 0 "$cli" validate "$tmp/room.map" "$tmp/room.scen" "$tmp/room.plan"
printf 'type octile\nheight 1\nwidth 3\nmap\n.@.\n' > "$tmp/split.map"
printf 'version 1\n0\tsplit.map\t3\t1\t0\t0\t2\t0\t2\n' > "$tmp/split.scen"
expect 1 "$cli" plan "$tmp/split.map" "$tmp/split.scen" 1
is_line "$tmp/out" "not solved: agent 0 can't reach its goal even alone"
expect 1 "$cli" plan "$data/small.map" "$data/small.scen" 3 --max-expansions 5
has "$tmp/out" "not solved: stopped by a limit at agent 0, 5 expansions"
expect 1 "$cli" plan "$data/small.map" "$tmp/shared_goal.scen" 2
has "$tmp/out" "invalid instance: "

# plan --planner cbs: the room prioritized planning can't solve in scenario order, at its optimum (10, against 7 for
# the two alone); the plan written and valid.
expect 0 "$cli" plan "$tmp/room.map" "$tmp/room.scen" 2 --planner cbs --out "$tmp/room_cbs.plan"
has "$tmp/out" "solved 2 agents: sum of costs 10 (lower bound 7), makespan 5, "
has "$tmp/out" " nodes ("
expect 0 "$cli" validate "$tmp/room.map" "$tmp/room.scen" "$tmp/room_cbs.plan"
is_line "$tmp/out" "valid plan for 2 agents: sum of costs 10, makespan 5"
expect 0 "$cli" plan "$data/small.map" "$data/small.scen" 3 --planner cbs --max-nodes 1
has "$tmp/out" "solved 3 agents: sum of costs 20 (lower bound 20), makespan 7, 20 expansions, 1 nodes (0 expanded), "
expect 0 "$cli" plan "$data/small.map" "$data/small.scen" 3 --planner pp
has "$tmp/out" "solved 3 agents: sum of costs 20"
# Two agents that must swap in a two-cell corridor: no plan, and CBS can't prove it, so a limit stops it.
printf 'type octile\nheight 1\nwidth 2\nmap\n..\n' > "$tmp/pair.map"
printf 'version 1\n0\tpair.map\t2\t1\t0\t0\t1\t0\t1\n0\tpair.map\t2\t1\t1\t0\t0\t0\t1\n' > "$tmp/pair.scen"
expect 1 "$cli" plan "$tmp/pair.map" "$tmp/pair.scen" 2 --planner cbs --max-nodes 50 --out "$tmp/pair.plan"
has "$tmp/out" "not solved: stopped by a limit, "
has "$tmp/out" " 50 nodes ("
[ ! -e "$tmp/pair.plan" ] || fail "plan --planner cbs wrote a plan file for an unsolved instance"
expect 1 "$cli" plan "$tmp/pair.map" "$tmp/pair.scen" 2 --planner cbs --time-limit 0.05
has "$tmp/out" "not solved: stopped by a limit, "
expect 1 "$cli" plan "$tmp/split.map" "$tmp/split.scen" 1 --planner cbs
is_line "$tmp/out" "not solved: agent 0 can't reach its goal even alone"

# plan: usage errors and unwritable output (exit 2).
expect 2 "$cli" plan "$data/small.map" "$data/small.scen"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 4
has "$tmp/err" "has 3 entries, not 4"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --order random
has "$tmp/err" "unknown order 'random'"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --order
has "$tmp/err" "option --order needs a value"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --max-expansions -1
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --time-limit -1
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --verbose yes
has "$tmp/err" "unknown option '--verbose'"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --out "$tmp"
has "$tmp/err" "can't write $tmp"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --planner astar
has "$tmp/err" "unknown planner 'astar' (pp or cbs)"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --planner cbs --order reverse
has "$tmp/err" "--order is for prioritized planning (--planner pp)"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --max-nodes 10
has "$tmp/err" "--max-nodes is for conflict-based search (--planner cbs)"
expect 2 "$cli" plan "$data/small.map" "$data/small.scen" 3 --planner cbs --max-nodes x

# Output that can't be written (here a closed stdout) is an error, not success.
"$cli" gen-map empty 3 3 >&- 2> "$tmp/err"
[ $? -eq 2 ] || fail "gen-map with a closed stdout didn't exit 2"
is_line "$tmp/err" "pathfinder: can't write the output"

if [ "$failures" -ne 0 ]; then
  echo "$failures CLI check(s) failed"
  exit 1
fi
echo "every CLI check passed"
