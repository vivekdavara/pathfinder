# Changelog

## v1.0.0 (2026-11-10)

The first release: multi-agent path finding on grid maps, from the file formats to a replay you can watch. Built in
five days; [README.md](README.md) has the measured results ("Results at v1.0.0"), [DESIGN.md](DESIGN.md) the rules
the code implements, [WALKTHROUGH.md](WALKTHROUGH.md) the questions it should be able to answer.

### The library (C++20)

- The grid (4-connected, cells as `y * width + x`), strict parsers and canonical writers for the MovingAI `.map` and
  `.scen` formats with line-numbered errors, and a `.plan` format (day 1).
- Seeded generators (empty, random and warehouse maps, random scenarios) whose output is the same on Linux and macOS,
  pinned by hashes (day 1).
- `validate`, the oracle every plan from every planner goes through: vertex and swap conflicts, illegal moves,
  costs, with standard MAPF semantics (an agent that has arrived stays on its goal) (day 1).
- Space-time A\* with a reservation table and heuristic tables by passable cell; "no path" is a proof, not a timeout
  (day 2).
- Prioritized planning, the baseline: fast, valid by construction, neither complete nor optimal (day 2).
- Conflict-based search, returning plans of minimum sum of costs, with a conflict scanner 23x to 34x faster than
  `validate` in its inner loop; node, expansion and time limits (day 3).

### Tools

- The `pathfinder` CLI: `info`, `gen-map`, `gen-scen`, `check`, `validate`, `plan` (`--planner pp|cbs`) (days 1-3).
- `scripts/fetch_maps.py`: the MovingAI MAPF benchmark set, checked by SHA-256, not redistributed (day 1).

### Python (pybind11)

- The module `pathfinder._core` behind the `pathfinder` package: everything above, with C++ exceptions as Python ones
  and cell ids range-checked (day 1 onwards).
- `pathfinder.simulator`: plans executed step by step with robots delayed, as a timetable (delays become recorded
  collisions) or in the plan's order of visits to each cell (the action dependency graph: no collision, no
  deadlock, no cost for an undelayed plan) (day 4).
- `pathfinder.replay`: a run recorded as JSON and checked by running the simulator again on it; `html()` writes a
  self-contained page that plays runs side by side on one timeline (`#t=` links to a moment) (day 5).
- `pathfinder.render`: the same drawing as an animated GIF (Pillow) (day 5).

### Experiments

- `scripts/sweep.py` and `scripts/sweep_summary.py`: both planners across agent counts on MovingAI maps, every plan
  validated, paired SOC comparisons (day 4; records in `results/day4/`).
- `scripts/delays.py`: plans executed under random and single delays with both policies (day 4).
- `scripts/replay.py`: one instance planned, executed with each policy under the same delays, written as a page,
  JSON and a GIF (day 5; `results/day5/`).

### Quality

- 204 CTest tests: the GoogleTest suites (brute-force oracles for space-time A\* and for CBS's optimum, randomly
  damaged files for the parsers), the CLI script, 166 pytest tests, 16 Node tests of the viewer's script, the page in
  headless Chrome, the examples and scripts end to end.
- CI on every push: GCC 14 and Clang 18 on Ubuntu, Apple Clang on macOS (Debug with warnings as errors, Release, a
  benchmark smoke test), and the C++ tests under ASan and UBSan.
- A review each day: an agent in its own worktree on days 1 to 4 (its findings kept as tests in `tests/review*/` and
  `tests/python/review*/`), mutation checks, coverage; day 5's review by hand (README "Review of day 5").

### Known limitations

README "Limitations": plain CBS (no cardinal conflicts, bypassing or symmetry reasoning) stops finishing at about 30
to 80 agents on the 32 x 32 benchmark maps within 5 s; delays are whole steps; the ordered policy never reorders
robots at a cell; the viewer has no zoom.
