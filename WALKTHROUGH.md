# Pathfinder: walkthrough

The questions an interviewer is likely to ask about this project, with short answers that point at the code. Day 1
covers the model, the formats, the generators and the validator; day 2 the single-agent search and the baseline
planner; day 3 conflict-based search; day 4 the simulator and the benchmarks; day 5 the replay viewer and the
release.

## In two minutes

Pathfinder plans paths for many robots on one grid so that none collide: multi-agent path finding. Day 1 built
everything the planners will stand on: a strict reader for the MovingAI benchmark files every MAPF paper uses, seeded
generators for random and warehouse floors, and `validate`, an independent checker that every plan from every planner
will go through. Running the readers on the real benchmark set (690,275 start-goal pairs) turned up one wrong
assumption of mine about the files, which is now documented, tested and handled. Day 2 added space-time A\* (one agent
around reservations), checked against a brute-force oracle, and prioritized planning, the baseline that CBS will be
measured against: fast, valid by construction, and visibly incomplete once the floor gets crowded. Day 3 added
conflict-based search: it returns plans of minimum sum of costs, which a brute-force search over joint states confirms
on every tiny instance it finishes, and it shows the classic trade: plans 3% to 6% cheaper on average than prioritized
planning's on the benchmark maps (up to 23%), but an effort that ranges from 9 to 14,331 tree nodes for ten agents on
the same floor, and stops finishing at about 30 agents on a 32 x 32 map. It also exposed a performance bug in day 2's
search, now fixed. Day 4 measured the trade properly (1,550 tries on four benchmark maps: prioritized planning solves
nearly everything to 100 agents in milliseconds, CBS falls from 24 or 25 of 25 at 10 agents to none by 30 to 80, and
where both solve prioritized planning costs 0.5% to 12% more on average) and built a simulator that runs plans with
robots delayed. Run as timetables, plans collide after a handful of delays; run in each plan's order of visits to
every cell, they can't collide or deadlock, and each step of delay costs the fleet 1.01 to 1.21 steps. Day 5 made
that visible: any run can be recorded as a replay (JSON, checked by running the simulator again on it), played in a
self-contained page with several runs side by side on one timeline, or rendered as a GIF; the README opens with one
CBS plan run both ways under the same delays (6 collisions as a timetable, none ordered, for 5 more steps of cost).

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
Because it's the oracle: every plan from space-time A\*, prioritized planning and CBS goes through it in the tests,
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
the CLI can check and the replay viewer can play (`scripts/replay.py --plan FILE`, day 5): `version 1`, `agents k`, then one line per agent with its
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
changes the effort a lot: without it the same benchmark instances took a median 1.65x as many nodes, up to 186x,
and one 30-agent instance stopped finishing within 10 s (README "Results (day 3)").

**When does CBS give up, and can it prove there's no plan?**
Two agents that must swap in a dead end: no plan exists, but each constraint only pushes a move one step later, so
the tree never runs out. That's why the node, expansion and time limits are part of the contract and every CBS call
in the tests has a node limit (a broken CBS then fails a test instead of hanging CI). If the open list does empty,
that is a proof (every child was infeasible), reported as `kNoPath`; on 226 unsolvable tiny instances it never
happened, so in practice "unsolvable" shows up as a limit.

**How does it compare with prioritized planning?**
On random-32-32-20 and warehouse-10-20-10-2-1 (MovingAI scenarios 1-5): both solve every 10- and 20-agent case,
CBS in at most 33 ms; prioritized planning's SOC is 3% to 6% above CBS's on average and up to 23%. At 30 agents
prioritized planning still solves all of them in milliseconds, CBS only 2 and 3 of 5 within 10 s. That's the trade
the project exists to measure (day 4 does it properly, across agent counts).

**Tell me about a performance bug CBS exposed.**
On orz900d, CBS with 10 agents built only 2 tree nodes but spent 36 million low-level expansions and blew through
a 20 s limit. The cause was in day 2's search, not in CBS: when another agent must pass through a parked agent's goal
at, say, t = 500, the goal test correctly refuses to end before 501, but the heuristic (distance to the goal) didn't
know that, so A\* expanded every detour that would end too early, one f-layer at a time. The fix is one line in
`src/space_time_astar.cpp`: f = max(t + h, last goal entry + 1), still admissible and consistent. A test pins it (a
goal ten cells away, taken at t = 500: 1,408,200 expansions before, 501 after), the brute-force oracle confirms the
arrivals didn't change, and that orz900d case now takes 12 ms. The same search serves prioritized planning, where an
earlier agent crossing a later agent's goal late sets the same trap, and it had been paying for it since day 2: its
50-agent benchmark went from 14.9 ms to 3.6 ms. A day-2 measurement had been hiding a bug the whole time; CBS just
made it impossible to miss.

**Why is CBS's effort so uneven?**
Ten agents on the same warehouse floor take 9, 77, 2,675, 8,069 or 14,331 nodes depending on the seed, and the
2,675 was 375,673 before the goal bound, for the same SOC: only which of several equally short paths the low level
returned had changed. When two agents meet in a corridor, both children usually cost the same, and the conflict
reappears a step later, so the tree fills with ties. The known fix is to split on cardinal conflicts first (ones where both children must cost more,
found from each agent's graph of optimal paths); it's on the list as a measured refinement, not an assumption.

## The benchmarks (day 4)

**What did you measure, and how?**
`scripts/sweep.py`: both planners on the first k agents of each of the 25 MovingAI random scenarios of four maps
(empty-32-32, random-32-32-20, room-32-32-4, warehouse-10-20-10-2-1), k = 10 to 100, 5 s per try. Solved means a
valid plan within the limit (prioritized planning's "no path" counts as a failure: it's incomplete). Every plan goes
through `validate()` and is checked against the lower bound; SOC comparisons use only the instances both planners
solve; and once a planner solves none at some k it isn't tried beyond. Every try is a JSON record in
`results/day4/`, and `scripts/sweep_summary.py` rebuilds the README's table from them.

**What did it show?**
The textbook trade, with numbers. Prioritized planning answers in milliseconds at every size and solves all 25
scenarios up to 100 agents on the empty map, 21 of 25 on random-32-32-20 at 100; it suffers only where the map has
choke points (room-32-32-4: 19 of 25 at 30 agents, 1 of 25 at 90). CBS solves everything small and then falls off a
cliff: on empty-32-32 25, 23, 17, 14, 9, 5, 1, 0 of 25 from 10 to 80 agents; on random-32-32-20 25, 25, 9, 1, 0 from 10
to 50; on room-32-32-4 24, 12, 0 from 10 to 30. Where both solve, prioritized planning's plans cost more, and more as
the map fills: on empty-32-32 0.5% more on average at 10 agents, 9.3% at 50 (up to 17.6% on one instance).

**Why does CBS fail on an empty map, where the answer is nearly the lower bound?**
On empty-32-32 its solved plans are at the lower bound on 24 of 25 instances at 10 agents and on 81 of the 94 it
solved at any size, never more than 0.6% above it, so the optimum is easy to describe; CBS just can't prove it. Two
robots whose shortest paths cross on an open grid have many equally short ways to dodge, and each split's two children
usually cost the same, so the tree fills with ties (the known "rectangle" symmetry). It isn't slow per node: a median
of about 71,000 nodes a second per try, and its failures had a median of 354,684 nodes when the limit stopped them.

**Would a longer time limit help?**
Not much, by this data: of the 269 tries CBS solved, 228 took under 10 ms and only 11 took between 1 and 5 s. It's bimodal: an instance is easy or it's hopeless for plain CBS. That's a measurement of this
sweep, not a proof; the fix that matters is algorithmic (cardinal conflicts first, symmetry reasoning), not seconds.

**So which planner would you deploy?**
For a live warehouse, prioritized planning (or a bounded-suboptimal search) plus ordered execution: it answers in
milliseconds, its plans cost a few percent more, and the ordered execution absorbs the delays any plan will meet.
CBS is the right tool for small groups, for offline planning, and as the yardstick that tells you what those few
percent are.

## The simulator (day 4)

**What is the simulator for?**
A plan is a timetable: robot i is in cell c at time t. Real robots slip, and a plan has no slack (an optimal plan is
as tight as its cost), so the question a warehouse asks is what a delay does to it. `python/pathfinder/simulator.py`
executes a valid plan one step at a time, lets any robot be delayed at any step (it holds its cell for the step),
checks every step on its own, and reports the costs the robots actually drove and every collision.

**What are the two ways it executes a plan?**
*Timetable*: every robot keeps to its own path, and a delayed robot falls one step behind; nobody else adapts. That's
the naive execution, and a delay turns straight into collisions (a follower rear-ends its delayed leader the next
step). *Ordered*: the plan's action dependency graph (Hönig et al., RA-L 2019). Each robot keeps its path's cells but
not its clock: the plan's waits are dropped, and a robot enters a cell only when it's the next robot the plan sends
through that cell and the one before it has left. So the plan's order of visits to every cell is kept, not its times.

**Why can't the ordered policy collide or deadlock?**
No two robots share a cell, because each cell is entered in the plan's order, one visit at a time, after the visit
before has left. No head-on swap: if i (in u) and j (in v) could both move, j's visit to v comes just before i's and
i's to u just before j's, which in the plan forces them to leave at the same step, a swap in the plan itself. No
deadlock: in a step with nobody delayed, the earliest-planned move still pending can always be made, since everything
it waits for was planned earlier and so has happened. If any of these ever failed it would be a bug, so the simulator
raises instead of recording (the swap and deadlock guards tested by breaking a cell's queue by hand, since no valid
plan can trigger them; the collision and illegal-move checks by forcing a bad step, in the review's tests).

**Why decide everyone's move together instead of one robot at a time?**
Rotations. Four robots on a 2 x 2 grid each moving to the next cell clockwise is valid, but each needs the robot ahead
to leave first; one robot at a time, nobody ever goes. So the step starts from every robot that's next in its target
cell's queue and drops each one whose target is held by a robot that isn't a candidate; a dropped robot stays put, so
the drop passes back to the one robot that wanted its cell, and on down the chain. What survives is chains of
followers and closed cycles (`test_ordered_moves_a_rotation_in_one_step`). The first version re-scanned every robot
until nothing changed: the same answer, but quadratic per step on a long blocked chain (2.2 s for 400 followers in a
corridor, 0.34 s now; the warehouse delay experiment prints identical tables with either).

**What does an undelayed ordered run of a CBS plan cost?**
Exactly the plan, robot by robot. Two facts squeeze it: with no delays every move happens no later than planned (by
induction on planned time), so no robot's cost goes up; and the trajectories driven are themselves a valid plan, which
can't cost less than CBS's optimum. The test checks it on every CBS plan it makes. For prioritized planning the
guarantee is only "no later".

**Tell me about a claim of yours that turned out false.**
Prioritized planning's plans under the ordered policy. I expected dropped waits to make some robots early, saw none
on 14 plans, and wrote down why: each path is the earliest arrival around the robots planned before it, so its waits
are all needed. The review broke it with a 9-agent instance where one robot arrives a step early. My argument was
true only against the earlier robots' *planned* timing, and the ordered policy changes that timing: robot 3's wait is
dropped, so a cell frees a step early, and robot 6, which was waiting for that cell, gets in sooner. It's rare (15
robots among 556,778 in the reviewer's random search), but "rare" isn't "never", and the docs said never. Fourteen
examples were never a proof; the argument was the thing to check.

**How do you know the simulator's own checks are right?**
They're compared with two independent checks in the library. On 40 delayed timetable runs, the collisions the
simulator records are exactly the conflicts `validate()` and the conflict scanner find on the trajectories it drove,
pair by pair. That comparison caught a bug on its first run: the swap check looked robots up by their move, one robot
per move, but under the timetable two robots already stacked in one cell can make the same move, and the second
robot's swap went unseen (47 found against validate's 48). My mutation check of the simulator kills 13 of 13 mutants;
the review then found 8 of its own that my tests missed, and its tests kill them.

**What does a delay actually do to a plan?**
Run as a timetable, it breaks it, fast. On warehouse-10-20-10-2-1 with 20 robots, delaying each robot at each step
with probability 1% (about 16 robot-steps of delay in a run) makes 53.5% of prioritized planning's runs collide and
23.5% of CBS's; on the denser random-32-32-20, 57.3% and 42.3% (`scripts/delays.py`, 460 and 480 runs a row).
Executed in the plan's order instead, nothing collides and the cost is mostly just the delay itself: each robot-step
of delay costs the fleet 1.01 to 1.05 robot-steps on the warehouse map and 1.09 to 1.21 on the denser one, where a
delayed robot is more often in someone's way (a five-step delay of one robot there costs the others 2.4 to 2.7
steps). That's the argument for executing a plan's order rather than its clock.

**Why do prioritized planning's plans collide more often under the same delays?**
Because they hand off cells more tightly. On the same 23 warehouse instances, its plans have 24.4 following moves
each (a robot moving into a cell in the step another moves out of it) against CBS's 7.3; on random-32-32-20, 18.8
against 14.8. A one-step delay of the robot ahead turns exactly those into collisions. Prioritized planning routes
each robot by the earliest arrival around the robots before it, which hugs their paths; CBS starts from each robot's
own shortest path and changes it only where a conflict forces it, so its robots meet less. I didn't expect it; the
count was a hypothesis checked after the collision rates came out.

**Why is the simulator in Python, not C++?**
It isn't a hot path (a few thousand robot-steps per run), it sits next to the experiments that use it, and the checks
that matter (validate, the scanner) are the library's, called through the bindings. Keeping it small and readable
matters more than speed here.

## The replay viewer (day 5)

**What does the viewer show, and why build it?**
One plan executed with robots delayed, step by step: each robot moving between cell centres, its goal as a square in
its colour, a dashed ring where the plan has it at that moment (so the gap is how far behind it is), an amber ring
when it is delayed in this step, a red ring when two robots share a cell and a red bar when they swap. The numbers of
day 4 say that a timetable collides and the ordered policy waits; the page shows *where* and *why*: in the README's
example the timetable's first collision is at t = 3: robot 9 was held in the first step, so it reaches the crossing
cell (13, 4) a step late, just as robot 6 enters it on schedule; in the ordered run robot 6 waits one more step at
(13, 5) and goes in after robot 9 has left. Several runs share one timeline, so
the same plan under the same delays can be watched both ways at once. Clicking robot 6 (or opening
`warehouse-16.html#t=5&robot=6`) draws its plan and the path it has driven in both runs, with a line per run: in the
timetable, "collided with 9 at t = 3"; ordered, "no collisions".

**What is a replay, and how do you know one is right?**
A JSON document (`python/pathfinder/replay.py`): the map, the robots, the plan, every robot's cell at every step,
who was delayed in each step, the collisions and the run's summary. `record()` runs the simulator with an `on_step`
callback and writes down what it did. The check is the part I'd point at: the simulator is deterministic given the
plan and each step's delays, so `check()` doesn't re-implement the rules; it rebuilds the instance, validates the
plan, re-runs the simulator with the recorded delays and requires every position, collision and summary field to come
out identical, with the same JSON types. A hand-edited file where a robot jumps, a delayed robot moves, a collision
is dropped or a cost is changed fails; `test_replay.py` has one test per kind of edit.

**Why a static HTML file instead of a web app?**
Nothing needs a server: a replay is tens to hundreds of KiB and everything on screen is arithmetic on it. So
`html()` inlines `player.js` and the replays' JSON into one file that opens from a download, fetches nothing and has
no build step; the GIF exists because GitHub's README shows images but runs no scripts.

**The page is JavaScript and the GIF is Python. How do they stay consistent?**
Both compute the same six things at a moment of the run (positions, the plan's ghosts, who is delayed, which
collisions show, the counts, the colours), and a test runs `player.js` under Node on a real run with delays and a
collision and compares every value with the Python functions at whole and fractional times, around each collision's
window, before the start and after the end (`test_the_page_and_python_agree_on_every_moment`). Two copies of a rule
are a bug waiting to happen; a test that compares them is how I'd live with needing both.

**How is the JSON put into the page safely?**
In a `<script type="application/json">` element, with `<`, `>` and `&` written as `\u003c`-style escapes. Those
characters can only occur inside JSON strings, where the escapes mean the same thing, so a title containing
`</script>` or `<!--` can't end or change the element as far as the HTML parser is concerned; a test puts exactly
that in a title and a map name. The template is split at its placeholders before anything is inserted, so a title
containing a placeholder's text isn't replaced either.

**Why does the timetable run have the lower sum of costs in the README's example?**
260 against the ordered run's 265, because nothing stops a timetable robot that drives into another: it keeps its
schedule through the collision. That cost isn't achievable by real robots; the ordered run's 265 is what the same
delays cost when nobody collides.

**How did you test something visual?**
In layers. The pure functions are tested in Node (15 tests) and against Python. The GIF's pixels are checked where
the replay says things are: every robot's centre pixel in its palette colour at several moments of both runs, an
amber ring around each delayed robot, red only in the run that has collisions, and a real timetable swap (found by
searching seeds) drawn as a red bar along the whole edge. The page's own drawing code runs in headless Chrome in
the test suite: a probe script added to the test page reads the canvas at every robot's position and inside a
collision's ring after the page has drawn itself, and three hand-made bugs in the drawing (x and y swapped, the ring
too small, the wrong colours) each fail it. The controls I checked by hand in a browser (Space pausing once with the
Play button focused was a bug I found that way).

## The review

**How was day 5 reviewed?**
By me, not an agent: the run started five hours late, at the same time as the other project's session, and two
sessions plus agents had hit the account's usage limit before. Four passes, each finding something: reading the
code over (four edge cases in the script and the page), a mutation check of 38 mutants (two test gaps, and a special
case in both languages that did nothing: a collision at the run's last moment already falls in its window) and 10
more for the robot following (two more gaps: a robot second in a collision named the wrong partner in a mutant no
test saw), coverage (no test had a valid instance with an invalid plan), and property tests (the page's shape check
let `"planner": 7` through, and the page then failed with a JavaScript error instead of saying what was wrong with
the file). And a false claim of mine in a test comment, that
simulator runs never contain a swap: timetable runs do, and the test now draws a real one.

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

**What did the day-3 review find?**
One bug, again at the contract's edge: CBS asked "may I generate another node?" before each child's search instead
of once the child existed, so a child that turned out to have no path still used up the node limit, and a run given
exactly the nodes it needed gave up one split early (about 1% of tiny instances). Now the limit is checked when a
child is about to be added, and a stop at the limit has generated exactly the limit. It also noted that CBS built all
its distance tables before looking at the clock; with 300 agents on a big grid that turned a 20 ms limit into 3.2 s.
Its best clean check compared CBS with its own brute force on instances my generator never makes (agents starting
on their goals, goals that are a permutation of the starts): no disagreement in about 5,300 instances.

**What did the day-4 review find?**
No bug, and one false claim of mine (the prioritized-planning robot that arrives early, above). Its most useful work
was making the tests stronger: eight mutants of the simulator survived my tests (a swap recorded backwards, the
ordered policy's collision raise switched off, `makespan` computed as a minimum, both `finished()` rules, ...), and its
tests kill all eight. Its best clean check ran the ordered policy step by step against its own executor of the
dependency graph, written differently (an explicit list of each visit's earlier visitors), under adversarial delay
schedules such as "delay whoever could move": 260,064 runs, the same moves at every step. It also noted three script
edges I then fixed: the sweep's give-up rule assumed ascending agent counts, a time limit of 0 meant none, and a
ratio divided by zero when every agent starts on its goal. A second round, on everything after, found two bugs in
those scripts, one in my fix: I'd guarded one division by zero and left the next one, by the lower bound, which is 0
on exactly the same instances; and the summary script counted a repeated try once as solved but twice as tried. It
also caught docs describing the step's old algorithm and a "twice as often" that held at one delay rate only. A
third round found the same pattern once more: my time-limit check went into `sweep.py` but not into `delays.py`,
which passes the same flag to the same planner, and my deduplication covered agent counts but not map names. A fix
is a claim about every place the problem lives, so I now grep for the siblings before committing one.

**Tell me about a bug in your own test tooling.**
Two. My first test of the bounded draw's bias used n = 2^63 + 1 and expected about half the draws above 2^63 - 1;
but only two values lie there, so it failed on correct code. The right example is n = 3 · 2^62, where a plain modulo
puts half the draws in the bottom third instead of a third, and the test now fails on a mutant with a plain modulo.
And the parser robustness test chose between duplicating and dropping a line with a fresh random number instead of
the case it was in, so it duplicated 1 time in 6 instead of half the time. Day 4 added a third: my mutation script
backed up the simulator from the working file before each run, and when one run was killed mid-mutant (a deadlock
mutant hung its tests), the next run backed up the mutant and "restored" it. A `cmp` against the earlier backup and
the next run's own check (the line it meant to mutate was gone) caught it before any commit; the script now restores
from a known-good copy and bounds each test run at 60 s.

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

**What would you do next?**
In order, each measured against what's here rather than assumed to help:
1. Cardinal conflicts first in CBS (split on a conflict where both children cost more), then bypassing: the sweep's
   CBS failures are symmetric conflicts on maps where the optimum is the lower bound, which is exactly what those
   address.
2. A switchable dependency graph for the ordered execution: let two robots trade places in a cell's queue when the one
   ahead is delayed, if that costs less; today one long delay holds up everyone queued behind it.
3. Replanning in the loop for long delays, compared with the ordered policy on the same delay draws (the replay
   viewer can already show both runs side by side).
4. Zoom and pan in the viewer, for maps where a cell is a few pixels.

**What did v1.0.0 leave out on purpose?**
The CBS refinements above: the point of the benchmarks was to measure plain CBS against prioritized planning, and an
improved CBS needs its own sweep and review, not a change on release day. The viewer stayed a static page for the
reasons above.
