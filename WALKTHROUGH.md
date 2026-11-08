# Pathfinder: walkthrough

The questions an interviewer is likely to ask about this project, with short answers that point at the code. Day 1
covers the model, the formats, the generators and the validator; day 2 the single-agent search and the baseline
planner; day 3 conflict-based search; later days add the benchmarks and the visualizer.

## In two minutes

Pathfinder plans paths for many robots on one grid so that none collide: multi-agent path finding. Day 1 built
everything the planners will stand on: a strict reader for the MovingAI benchmark files every MAPF paper uses,
seeded generators for random and warehouse floors, and `validate`, an independent checker that every plan from every
planner will go through. Running the readers on the real benchmark set (690,275 start-goal pairs) turned up one wrong
assumption of mine about the files, which is now documented, tested and handled. Day 2 added space-time A\* (one
agent around reservations), checked against a brute-force oracle, and prioritized planning, the baseline that CBS
will be measured against: fast, valid by construction, and visibly incomplete once the floor gets crowded. Day 3
added conflict-based search: it returns plans of minimum sum of costs, which a brute-force search over joint states
confirms on every tiny instance it finishes, and it shows the classic trade: plans 2% to 6% cheaper on average than
prioritized planning's on the benchmark maps (up to 23%), but an effort that ranges from 9 to 375,673 tree nodes for
ten agents on the same floor.

## The model

**What is MAPF, and why is it hard?**
k agents on a shared graph, each with a start and a goal, all moving at once in discrete steps; find paths with no
collisions, minimising the sum of the agents' arrival times. Planning each agent alone is easy (BFS); the joint
problem's state space is the product of every agent's position, and finding an optimal plan is NP-hard (Yu and
LaValle, 2013). The planners (days 2 and 3) are two ways of not searching that product space directly.

**Why can't each robot just take its own shortest path?**
`examples/independent_paths.py` tries exactly that: each of 40 agents on the default warehouse floor (204 free
cells) walks down its own BFS distances. Every path is legal and as short as possible, so the sum of costs, 591, is
a lower bound no planner can beat, but the validator finds 83 vertex and 23 swap conflicts; with 150 agents, 1,145
and 318. Removing those at the least extra cost is the whole problem.

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
starts, one for the goals, and the whole goal draw repeated whenever any agent's goal is its own start. That is plain
rejection sampling, so every valid configuration is equally likely, and a draw is accepted with probability at least
about 1/e, so it takes under 3 draws on average.

**Why not just redraw the one goal that clashed?**
That was my first version, and the review showed it's biased: the number of choices left for agent i depends on
whether an earlier goal took agent i's start. On a 3-cell row with 2 agents, the 18 valid configurations came up
about 750 or 1,500 times each in 18,000 seeds instead of 1,000 (`ReviewGenerate.RandomScenarioConfigurationsAreUniform`
in `tests/review/`). Each goal's own distribution looked fine, which is why my per-cell test didn't see it: the bias
was in the joint distribution.

## Plan files

**Why a plan format of your own?**
MovingAI defines maps and scenarios but no format for solutions, and the planners (day 2) need to write plans that
the CLI can check and the visualizer (day 5) can replay: `version 1`, `agents k`, then one line per agent with its
cells at t = 0, 1, ... as `(x,y)` (DESIGN.md "Plan files", `src/plan_file.cpp`). `pathfinder validate MAP SCEN PLAN`
prints a valid plan's SOC and makespan, or every problem.

**Why does the parser need the grid?**
To turn (x, y) into cell ids, and to refuse a cell outside the grid: as an id, (4, 0) on a 4-wide grid would silently
become (0, 1). The parser checks only the format, though; a file may hold a conflicting plan, because describing
what's wrong with it is the validator's job.

## The planners (day 2)

**How does one agent avoid the others?**
Space-time A\* (`src/space_time_astar.cpp`): A\* over (cell, time) instead of cells, where the five actions (wait,
four moves) all cost 1, so g is just the time, and h is the agent's BFS distance to its goal ignoring everyone. What
to avoid comes from a reservation table (`src/reservation.cpp`): cells forbidden at a time, moves forbidden at a
time, and cells forbidden from a time on (someone parked on their goal). Successors failing either check are never
generated.

**Why is the reservation table written from the searching agent's side?**
So both planners feed it the same way. Prioritized planning reserves each planned path as "you may not be here
then" for every cell, "you may not make the reverse move then" for every move (that is what forbids swaps, while
following stays legal), and "you may not be here from my arrival on" for its goal. CBS's constraints (day 3) are
"agent a may not be in v at t" and "may not move u → v at t": the same entries, added directly.

**When can a path end?**
Only at the goal, at a time after the last vertex entry on the goal: an agent that arrives early but would be
sitting there when someone else must pass has to step aside and come back
(`AnAgentOnItsGoalStepsAsideForSomeonePassing`). A goal someone rests on for ever has no valid ending at all, so the search refuses it before starting.

**How does space-time A\* know there's no path?**
Day 1's design bounded the time at (horizon + passable cells), the horizon being the latest time any entry names.
The search does something simpler: after the horizon nothing changes with time, so (c, 50) and (c, 51) have the same
future, and it keys states by (cell, min(t, horizon + 1)). The state space is then finite, (horizon + 2) x passable
at most, and an empty open list is a proof that no path exists. The one subtlety: a late state can be reached
again with a *lower* time, and must then be pushed again, as in any A\* with decrease-key.

**How do you know the search is right?**
`tests/space_time_oracle_test.cpp` compares it with brute force on 6,000 random problems: the set of cells the agent
can be in at each time, layer by layer, from the table's answers alone, up to day 1's explicit bound (and twice as
far when it finds nothing). Same earliest arrival, same "no path". The first generator (other agents' random walks
plus loose entries) passed a mutant that drops the lower-time re-push, even over 300,000 rounds; a second
generator, walls that go up early on an open grid so the heuristic is badly wrong after the horizon, kills it in 60.
Three other survivors are equivalent mutants, and I can say why for each (the commit message of `b735e68`).

**How does prioritized planning work, and how does it fail?**
`src/prioritized.cpp`: agents in a fixed order, each by space-time A\* against everything planned before, then
reserved. It's fast but incomplete. On the default 27 x 12 warehouse floor it solves every seed I tried at 30 agents
and fails some from 40: at 50 (seed 1), agent 27 starts in a corner on agent 17's goal and is walled in by agents
parked before it. On the MovingAI random-32-32-20 map, scenario 1 fails at 50 agents: agent 42's goal is sealed off
by parked agents; a brute-force search of its reservation table agrees there's no way. Another order can solve the
same instance (`TheOtherOrderSolvesTheSameInstance`), which is why the CLI has `--order`.

**Does the order matter?**
More than I expected, in two opposite ways (README "Results (day 2)"). On den520d with 400 agents, shortest-first
needs 49x fewer expansions than the scenario order (0.4 M against 20.8 M; 195 ms against 8.1 s) and lands 0.6%
above the lower bound, because short-path agents park early and keep the later agents' reservation horizons short.
On crowded maps longest-first solves the most: room-64-64-8 at 200 agents, 7 of 10 scenarios against 0 for the
other two orders, because long-path agents planned first get through before others park in their way. No order wins
everywhere, which is part of the case for CBS.

**Why check for unreachable goals separately?**
An agent whose goal is in another connected piece makes the instance unsolvable for every planner. One
`components()` pass names it (`kUnreachableGoal`) before any search, instead of letting a benchmark burn its time
limit on it.

**Why heuristic tables by passable cell?**
DESIGN.md's budget from day 1: 4 bytes a cell is 3.9 GB for 1,000 agents on orz900d, 90% of whose cells are walls.
`DistanceTable` stores only passable cells through a shared `PassableIndex`: a tenth of the memory there. My first
version was 12% to 31% slower to build than a table of every cell, because its BFS looked up every neighbour's
number in the index; precomputing each passable cell's neighbours by number made it 1.2x to 2.3x faster than the
all-cells BFS instead (0.36 against 0.65 ms on orz900d). Prioritized planning holds one table at a time anyway;
it's for CBS, which keeps one per agent.

## Conflict-based search (day 3)

**How does CBS work, in one breath?**
Plan every agent alone (the root). Take the node with the lowest sum of costs, find the first collision in its plan
(agents a and b in cell v at time t, or swapping), and make two children: one where a may not be in v at t, one where
b may not. Each child replans only its constrained agent, by the same space-time A\* as day 2, with that agent's
constraints as its reservation table. The first node popped with no collision is the answer (`src/cbs.cpp`).

**Why is the answer optimal?**
Two facts. Every valid plan obeys at least one child's constraints at every split (a and b can't both be in v at t
in a valid plan), so splitting never throws a valid plan away. And a node's SOC never decreases from parent to child
(the child replans one agent under more constraints, and the low level returns that agent's cheapest path). So the
open list, ordered by SOC, reaches the cheapest valid plan before any more expensive node. What makes the second fact
true is the low level being exactly optimal, which day 2's oracle test pins.

**How do you know it's right?**
`tests/cbs_oracle_test.cpp`: a brute-force optimum that knows nothing about trees or A\*. It is Dijkstra over joint
states, every agent's cell plus whether it's done, where an agent costs 1 per step until it declares itself done on
its goal. That models "cost is the final arrival" without tracking time, so the state space is finite and the search
proves unsolvability too. On 1,500 random tiny instances, CBS's SOC equals it on all 1,116 that CBS finishes in
3,000 nodes, and CBS never returns a plan where the brute force proves none exists. Six of seven mutants (a swap
constraint pointing the wrong way, ignoring SOC in the open list, forgetting ancestor constraints, ...) fail it or
the hand-derived cases in `tests/cbs_test.cpp`; the seventh, preferring more conflicts among equal-SOC nodes, only
changes how fast CBS gets there.

**What's subtle about goals?**
A finished agent sits on its goal for ever, so another agent walking through it is a conflict at a time after the
first agent's path ended. CBS splits it like any vertex conflict, and the child that forbids the parked agent from
its goal at t forces its path to end after t. That needed no new code: the low-level goal test from day 2 already
says "a path ends only after the last constraint on the goal" (`AnAgentOnItsGoalStepsAsideAndComesBack`: SOC 4,
worked out by hand in the test's comment).

**What does a node store?**
Only what it changed: its parent, its one new constraint, the agent it replanned and that agent's new path. A plan
is assembled by walking up to the root, taking each agent's path from the nearest node that replanned it; an agent's
constraints come from the same walk. The nodes live in a `std::deque`, so a plan can be a vector of pointers into
the tree while it grows. The alternative, a full copy of k paths per node, costs memory the tree needs for nodes.

**Why a separate conflict scanner instead of calling validate()?**
CBS looks at every node's plan, so this is its inner loop. `validate()` rebuilds a hash map of occupants at every
time step and reports everything in a fixed order; the scanner keeps one list per cell, stamped with an epoch so
nothing is cleared between time steps or plans, and keeps only the count and the earliest conflict. It finds
exactly validate()'s conflicts (3,000 random-walk plans, one scanner reused across all of them) and is 23x to 34x
faster (`make bench`). The count is the open list's tie-break: among nodes of equal SOC, the one closest to valid
first. It can't change the answer (a mutation test confirmed that: it survives every correctness test), but it
changes the effort a lot: without it the same benchmark instances took a median 1.66x as many nodes, up to 186x,
and one 30-agent instance stopped finishing within 10 s (README "Results (day 3)").

**When does CBS give up, and can it prove there's no plan?**
Two agents that must swap in a dead end: no plan exists, but each constraint only pushes a move one step later, so
the tree never runs out. That's why the node, expansion and time limits are part of the contract and every CBS call
in the tests has a node limit (a broken CBS then fails a test instead of hanging CI). If the open list does empty,
that is a proof (every child was infeasible), reported as `kNoPath`; on 226 unsolvable tiny instances it never
happened, so in practice "unsolvable" shows up as a limit.

**How does it compare with prioritized planning?**
On random-32-32-20 and warehouse-10-20-10-2-1 (MovingAI scenarios 1-5): both solve every 10- and 20-agent case,
CBS in at most 176 ms; prioritized planning's SOC is 2% to 6% above CBS's on average and up to 23%. At 30 agents
prioritized planning still solves all of them in milliseconds, CBS only 2 and 3 of 5 within 10 s. That's the trade
the project exists to measure (day 4 does it properly, across agent counts).

**Why is CBS's effort so uneven?**
Ten agents on the same warehouse floor took 9, 77, 8,309, 14,041 or 375,673 nodes depending on the seed. When two
agents meet in a corridor, both children usually cost the same, and the conflict reappears a step later, so the tree
fills with ties. The known fix is to split on cardinal conflicts first (ones where both children must cost more,
found from each agent's graph of optimal paths); it's on the list as a measured refinement, not an assumption.

## The review

**How was day 1 reviewed?**
By an agent in its own git worktree, asked for correctness bugs and to prove each with a failing test before
reporting it, then a second round on the fixes. The first round found 11 (README "Review of day 1"); none was in
`validate`, which it also compared with its own Python model of DESIGN.md's rules on 20,000 random instances with
no mismatch. Its tests live in `tests/review/` and `tests/python/review/`, so each fix is pinned.

**Which finding taught you the most?**
`make_instance` trusting coordinates: the parser checked them, so I'd assumed every scenario was checked, but a
scenario built in code (or from Python) never meets the parser, and `y * width + x` wraps a point off the right edge
onto the next row without any error. Checks belong at the boundary where the data enters, and an instance has more
than one way in.

**And the second round?**
It was asked what the first round's fixes broke, and found two: the plan parser I'd just written refused more than
a million agents while `write_plan` happily wrote them, so the format's own round-trip promise failed (the bound is
now the grid's cell count, since no instance has more agents than cells); and my new duplicate check in
`fetch_maps.py` compared names exactly, on a Mac file system that treats `x.map` and `X.map` as one file. Both
fixes came from new code, which is why a second round is worth its cost. A third round, on the changes after the
second, found three more in code written after that round (a case-folding key that still let some Greek spellings
collide, and two regressions in the Python coordinate handling), and nothing in the parts it had already cleared.

**What did the day-2 review find?**
Three bugs, all at the edges of the contract rather than in the search: an expansion budget used up exactly
stopped an agent that needed no expansion, because the search read "0 left" as "no limit" (now an `optional`); the
time limit was never checked for agents that need no expansion, because only the search looked at the clock (the
planner now checks before each agent); and `DistanceTable`'s "is this the grid's index?" check compared only counts,
so another grid's index crashed the BFS (`PassableIndex::fits` now checks the set exactly, in O(passable cells), by
noting that as many cells, all passable in the grid, is the same set). Its strongest clean result: the reservation
table forbids exactly what `validate` calls a conflict, on 40,000 random pairs of paths. That ties the planner's
model of the rules to the oracle's.

**And the second round?**
Two bugs, both in the Python bindings I wrote after the first round. The worse one: I released the GIL during
`space_time_astar` for speed, but the grid and reservation table it reads are mutable Python objects, so a second
thread calling `forbid_vertex` rehashed the hash set under the running search and crashed the interpreter from pure
Python. The search now keeps the GIL; `prioritized_planning` can still release it because nothing in Python can
change an `Instance`. Releasing the GIL is a promise that nobody else can touch what you're reading.

**Tell me about a bug in your own test tooling.**
Two. My first test of the bounded draw's bias used n = 2^63 + 1 and expected about half the draws above 2^63 - 1;
but only two values lie there, so it failed on correct code. The right example is n = 3 · 2^62, where a plain modulo
puts half the draws in the bottom third instead of a third, and the test now fails on a mutant with a plain modulo.
And the parser robustness test chose between duplicating and dropping a line with a fresh random number instead of
the case it was in, so it duplicated 1 time in 6 instead of half the time.

## Engineering

**How did you make the planners faster, and how do you know it helped?**
Two changes, each measured before and after. The passable-only heuristic table was first 12% to 31% slower to build
than a plain all-cells table, because its BFS looked up every neighbour's number; precomputing neighbours by number
made it 1.2x to 2.3x faster than all-cells. And a byte per cell in the reservation table, saying whether anything
names the cell, lets the search skip the hash lookups for most cells: prioritized planning of 50 agents went from
18.0-19.0 to 14.4-14.9 ms in an interleaved A/B of the two binaries (interleaved because a review agent was
building at the same time, which would have skewed back-to-back runs). Both kept the oracle tests green, and a
mutation check showed that forgetting any of the three marks fails 11 tests.

**How do you catch platform differences?**
CI (`.github/workflows/ci.yml`) builds and tests with GCC 14 and Clang 18 on Ubuntu and Apple Clang on macOS, plus
ASan and UBSan; the pinned hashes make generator drift fail everywhere at once. GCC flags things Clang doesn't, so
`make gcc-check` compiles every project file with Homebrew's g++-15 before a push. Even so, CI's first run was red,
on macOS: the runner's Xcode 16 libc++ has no floating-point `std::from_chars`, which this Mac's newer Clang has, and
no feature macro tells the two apart. Decimals now go through one strict parser, with a `strtod_l` fallback on
libc++ before 20, tested here by forcing it.

**How are dependencies pinned?**
GoogleTest and pybind11 are fetched by CMake as release tarballs pinned by SHA-256. One near miss: I first looked up
pybind11 at `github.com/pybind11/pybind11`, which redirects to an unrelated repository; the real project is
`github.com/pybind/pybind11` (PyPI's page links there), and that's what the build fetches.

**Why Python bindings at all?**
The experiments (day 4) are scripts: sweeps over agent counts, plots, a step-by-step simulator. pybind11 exposes the
library as `pathfinder._core` with C++ exceptions mapped to Python ones (`ParseError` is a `ValueError` carrying
`.line`) and cell ids range-checked at the boundary, so a bad index is an `IndexError`, not undefined behaviour.

## What's next

Day 4: success rate, runtime and SOC against the number of agents for both planners on the benchmark maps, and a
step-by-step simulator in Python. Day 5: a replay visualizer, the final results and v1.0.0.
