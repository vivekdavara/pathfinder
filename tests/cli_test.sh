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
expect 2 "$cli" gen-map warehouse 1 1 1 1 0 1
expect 2 "$cli" info "$data/short_row.map"
has "$tmp/err" "short_row.map: line 6: row 1 has 2 characters, expected 3"
expect 2 "$cli" info "$data/no_such_file.map"
has "$tmp/err" "can't open the file"
expect 2 "$cli" gen-scen "$data/small.map" 17 1
has "$tmp/err" "17 agents need at least 18"
expect 2 "$cli" gen-scen "$data/small.map" 3 1 "bad name"
expect 2 "$cli" check "$data/small.map" "$data/small.scen" 4
has "$tmp/err" "has 3 entries, not 4"
expect 2 "$cli" check "$data/small.map"

if [ "$failures" -ne 0 ]; then
  echo "$failures CLI check(s) failed"
  exit 1
fi
echo "every CLI check passed"
