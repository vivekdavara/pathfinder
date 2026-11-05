#!/bin/sh
# Runs `pathfinder check` on every fetched MovingAI scenario (data/scen-random) with its map (data/mapf-map) and sums
# up: files whose instance is valid with every goal reachable, and stated optimal lengths equal to the octile
# distance. Exits 1 if any file fails either.
#   scripts/check_maps.sh <pathfinder binary> <data directory>     (or: make check-maps)
set -u
cli=$1
data=$2
if [ ! -d "$data/scen-random" ] || [ ! -d "$data/mapf-map" ]; then
  echo "check_maps: no benchmark data in $data (run: make maps)" >&2
  exit 2
fi
files=0
failed=0
matching=0
entries=0
start=$(date +%s)
for scen in "$data"/scen-random/*.scen; do
  map="$data/mapf-map/$(sed -n 2p "$scen" | cut -f2)"
  out=$("$cli" check "$map" "$scen" 2>&1)
  code=$?
  files=$((files + 1))
  line=$(printf '%s\n' "$out" | grep '^stated optimal lengths')
  m=$(printf '%s\n' "$line" | sed -E 's/.*: ([0-9]+) of ([0-9]+)$/\1/')
  n=$(printf '%s\n' "$line" | sed -E 's/.*: ([0-9]+) of ([0-9]+)$/\2/')
  if [ "$code" -ne 0 ] || [ -z "$m" ] || [ "$m" != "$n" ]; then
    failed=$((failed + 1))
    echo "$scen: exit $code"
    printf '%s\n' "$out" | sed 's/^/  | /' | head -8
  fi
  matching=$((matching + ${m:-0}))
  entries=$((entries + ${n:-0}))
done
echo "scenario files: $files, failing: $failed"
echo "entries: $entries, stated optimal length equal to the octile distance: $matching"
echo "time: $(( $(date +%s) - start )) s"
[ "$failed" -eq 0 ]
