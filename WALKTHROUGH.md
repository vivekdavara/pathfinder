# Pathfinder: walkthrough

The questions an interviewer is likely to ask about this project, with short answers that point at the code. Day 1
covers the model, the formats, the generators and the validator; later days add the planners.

## In two minutes

Pathfinder plans paths for many robots on one grid so that none collide: multi-agent path finding. Day 1 built
everything the planners will stand on: a strict reader for the MovingAI benchmark files every MAPF paper uses,
seeded generators for random and warehouse floors, and `validate`, an independent checker that every plan from every
planner will go through. Running the readers on the real benchmark set (690,275 start-goal pairs) turned up one wrong
assumption of mine about the files, which is now documented, tested and handled.

## The model

**What is MAPF, and why is it hard?**
k agents on a shared graph, each with a start and a goal, all moving at once in discrete steps; find paths with no
collisions, minimising the sum of the agents' arrival times. Planning each agent alone is easy (BFS); the joint
problem's state space is the product of every agent's position, and finding an optimal plan is NP-hard (Yu and
LaValle, 2013). The planners (days 2 and 3) are two ways of not searching that product space directly.

**What counts as a collision?**
Two kinds (DESIGN.md "The model"): a *vertex conflict*, two agents in one cell at one time, and an *edge conflict*,
two agents swapping cells in one step. Following (moving into a cell its occupant is leaving) and rotations of three
or more agents are allowed. That is the definition the MAPF benchmark suite uses (Stern et al. 2019), so results are
comparable with published ones. `ProblemKind` in `include/pathfinder/plan.hpp` lists every problem `validate` reports.

**What happens when an agent reaches its goal?**
It stays there for ever; it doesn't disappear. So a parked agent still blocks its cell (`validate` looks up position
`path[min(t, size - 1)]`, `src/plan.cpp`), and a planner may not end an agent's path on a cell another agent must
still cross. The test is `Validate.AnAgentThatHasFinishedStillOccupiesItsGoal`.

**How is cost defined?**
An agent's cost is the time of its *final* arrival at its goal (`path_cost`): waits at the goal afterwards are free,
waits elsewhere and leaving the goal and coming back aren't. Sum of costs (SOC) adds them; makespan is the largest.
CBS minimises SOC; prioritized planning doesn't promise to.

## The validator

**Why build a validator before any planner?**
Because it's the oracle: every plan from space-time A\*, prioritized planning and CBS will go through it in the tests,
so a planner bug shows up as a failing test instead of a plausible-looking number. It shares nothing with the
planners but the grid, so a bug in their conflict detection can't hide in it too.

**How does it work, and what does it cost?**
Each path on its own first (start, goal, every cell passable and on the grid, every step a wait or a 4-neighbour
move), then, for each time step, an occupancy map from cell to the agents in it: any cell with two or more is a
vertex conflict for each pair, and an agent moving u → v while some agent at v moves to u is a swap. That is O(k · T)
for k agents and makespan T, against O(k² · T) for checking every pair. Problems come out in a documented order
(`PlanCheck::problems`), so a test can compare the whole list.

**How do you know the validator itself is right?**
Three ways (`tests/plan_test.cpp`): a test per rule with a small plan breaking only that rule; a differential test
against the most direct check possible (every pair of agents at every time) on 300 random-walk plans, which must give
the same list in the same order; and six mutations (no parking at the goal, duplicate swaps, diagonal steps allowed,
a step across a row's end allowed, the cost loop's bound, the last step skipped), each of which fails a test.

**What's the row-wrap trap?**
Cells are numbered `y * width + x`, so the last cell of a row and the first of the next have consecutive ids but
aren't neighbours. `Grid::adjacent_or_same` compares (x, y), not ids, and
`Validate.AStepAcrossARowsEndIsAJumpThoughTheIdsAreConsecutive` pins it.

## The files

**Why so strict about parsing?**
A benchmark result on a silently padded or truncated map is a wrong number nobody can trace. So `parse_map` and
`parse_scenario` (`src/movingai.cpp`) accept exactly what DESIGN.md lists and throw a `ParseError` naming the line
(and column, for a bad character) for anything else; the writers produce a canonical form and refuse anything the
parsers would reject, so write-then-parse round-trips exactly.

**What did the real benchmark data teach you?**
That I had the scenario files' "optimal length" column wrong. I assumed the MAPF scenarios would state 4-connected
distances, since MAPF agents move 4-connected; a check of all 825 files against BFS found only 46,078 of 690,275
entries matching. In `empty-8-8-random-1.scen`, (1, 4) → (4, 7) is stated as 4.24264069, which is 3√2: the octile
(8-connected) distance. A quick probe settled the corner rule (272 of 272 sampled entries match without corner
cutting, 64 with it), and `octile_distances` (`src/distance.cpp`) now checks the column; `make check-maps` compares
every entry. The generator writes octile lengths too, so a generated file means what a benchmark file means. The
planners never use that column; they compute 4-connected distances.

**How are the benchmark files obtained safely?**
`scripts/fetch_maps.py` downloads two archives over HTTPS, refuses them unless their SHA-256 and size match the
pinned values, and unpacks only `.map`/`.scen` members to plain paths under `data/`, refusing absolute paths, `..`,
symbolic links and oversized members. The files aren't committed (they're not ours to redistribute).

## The generators

**Why not `std::mt19937` and `std::uniform_int_distribution`?**
The engine is fully specified by the standard, but the distributions aren't: libstdc++ and libc++ turn the same
engine output into different numbers. Seeds must give the same map on Linux and macOS, so `rng.hpp` has SplitMix64
and its own bounded draw, and four generated files are pinned by hash in the C++ and Python tests.

**What does `below(n)` do, and why not `next() % n`?**
`next() % n` favours small values when n doesn't divide 2^64. `below` rejects draws under 2^64 mod n, leaving a whole
number of copies of [0, n). The test uses n = 3 · 2^62, where a plain modulo would put half the draws in the bottom
third; replacing `below` with a plain modulo makes `SplitMix64.BelowRejectsTheBiasedLowRange` fail (tried).

**How do you know the warehouse floor is connected?**
Every shelf block is surrounded by free cells: aisles of at least one cell between blocks and a margin of at least
one around them, so the free cells form one connected grid of corridors. `WarehouseMap.HasTheStatedSizeAndOnePiece`
checks size, blocked-cell count and one component on several layouts.

**How are start-goal pairs drawn?**
From the largest connected piece only (so every goal is reachable), by partial Fisher-Yates shuffles: one for the
starts, one for the goals, redrawing a goal that equals the agent's own start. With fewer agents than cells, at least
two cells remain at every step, so the redraw always ends; `RandomScenario.CanFillAllButOneCell` tests the edge.

## Engineering

**How do you catch platform differences?**
CI (`.github/workflows/ci.yml`) builds and tests with GCC 14 and Clang 18 on Ubuntu and Apple Clang on macOS, plus
ASan and UBSan; the pinned hashes make generator drift fail everywhere at once. GCC flags things Clang doesn't, so
`make gcc-check` compiles every project file with Homebrew's g++-15 before a push. (CI hasn't run yet: the GitHub
repository doesn't exist; see the README.)

**How are dependencies pinned?**
GoogleTest and pybind11 are fetched by CMake as release tarballs pinned by SHA-256. One near miss: I first looked up
pybind11 at `github.com/pybind11/pybind11`, which redirects to an unrelated repository; the real project is
`github.com/pybind/pybind11` (PyPI's page links there), and that's what the build fetches.

**Why Python bindings at all?**
The experiments (day 4) are scripts: sweeps over agent counts, plots, a step-by-step simulator. pybind11 exposes the
library as `pathfinder._core` with C++ exceptions mapped to Python ones (`ParseError` is a `ValueError` carrying
`.line`) and cell ids range-checked at the boundary, so a bad index is an `IndexError`, not undefined behaviour.

## What's next

**How will space-time A\* know when to give up?**
Past the last timed reservation the problem is static, and a shortest path in a static grid has at most as many
steps as there are passable cells, so a bound of (last reservation time + passable cells) on t is enough: a search
that passes it reports no path (DESIGN.md "The planners").

**What's subtle about CBS and goals?**
A vertex constraint on an agent's goal at time t means its path can't end there before t + 1, because it would then
sit on the goal at t. The low-level goal test is "at the goal, at a time after which no constraint covers the goal".
