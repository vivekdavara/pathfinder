"""The day-4 review (commit ed1cf44): the simulator (python/pathfinder/simulator.py), its tests, scripts/sweep.py and
scripts/delays.py.

No test here fails at ed1cf44: the review found no correctness bug in the simulator or the scripts. What is here:

- test_clean_*: checks that pass, each against an independent re-implementation or hand-worked answer, several of
  them written to kill mutants of simulator.py that survived tests/python/test_simulator.py (named in each);
- test_claim_*: a counterexample to a claim in DESIGN.md (it passes: it shows the claim is false).
"""

import functools
import importlib.util
import io
import os
import random
import re
import types
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import pytest

import pathfinder as pf
from pathfinder.simulator import SimulationError, Simulator, delay_robot, random_delays

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[3]))


# --- independent references ---


def dedupe(path):
    cells, times = [], []
    for t, c in enumerate(path):
        if not cells or cells[-1] != c:
            cells.append(c)
            times.append(t)
    return cells, times


class RefOrdered:
    """The ordered policy written another way: events with explicit predecessor lists. Robot i may enter its k-th
    distinct cell c when it isn't delayed and every earlier visit (j, m) the plan makes to c has entered
    (progress[j] >= m) and left (progress[j] > m) or leaves in this step; the step's movers are the greatest set
    closed under that rule."""

    def __init__(self, plan):
        self.cells = []
        visits = []
        for i, p in enumerate(plan):
            cells, times = dedupe(p)
            self.cells.append(cells)
            visits += [(t, c, i, k) for k, (c, t) in enumerate(zip(cells, times))]
        self.before = {}
        per_cell = {}
        for _, c, i, k in sorted(visits):
            seen = per_cell.setdefault(c, [])
            self.before[(i, k)] = list(seen)
            seen.append((i, k))
        self.progress = [0] * len(plan)

    def finished(self, i):
        return self.progress[i] == len(self.cells[i]) - 1

    def step(self, delayed):
        movers = {i for i in range(len(self.cells))
                  if i not in delayed and not self.finished(i)
                  and all(self.progress[j] >= m for j, m in self.before[(i, self.progress[i] + 1)])}
        changed = True
        while changed:
            changed = False
            for i in list(movers):
                if any(self.progress[j] == m and j not in movers for j, m in self.before[(i, self.progress[i] + 1)]):
                    movers.discard(i)
                    changed = True
        for i in movers:
            self.progress[i] += 1
        return movers

    def positions(self):
        return [cells[k] for cells, k in zip(self.cells, self.progress)]


def ref_timetable(plan, delay_sets):
    index = [0] * len(plan)
    trajectories = [[p[0]] for p in plan]
    for delayed in delay_sets:
        for i, p in enumerate(plan):
            if i not in delayed and index[i] < len(p) - 1:
                index[i] += 1
            trajectories[i].append(p[index[i]])
    return trajectories


def visit_order(trajectories):
    events = sorted((t, c, i) for i, cells in enumerate(trajectories) for t, c in enumerate(cells)
                    if t == 0 or cells[t - 1] != c)
    order = {}
    for _, c, i in events:
        order.setdefault(c, []).append(i)
    return order


def padded(plan):
    n = max(len(p) for p in plan)
    return [list(p) + [p[-1]] * (n - len(p)) for p in plan]


# --- delay schedules (adversarial ones look at the simulator) ---


def movers_schedule(p, seed):
    """Delays, with probability p, robots that could move now (ordered: its next visit heads its cell's queue)."""
    rng = random.Random(seed)

    def schedule(t, sim):
        out = []
        for i in range(sim.robots):
            if sim.policy == "ordered":
                k = sim._visit[i]
                if k + 1 < len(sim._cells[i]):
                    v = sim._cells[i][k + 1]
                    if sim._queue[v][sim._next[v]] == (i, k + 1) and rng.random() < p:
                        out.append(i)
            elif rng.random() < p:
                out.append(i)
        return out

    return schedule


def all_but_one_schedule(seed):
    """Everyone but one robot (a new one every 1-5 steps) delayed, with an undelayed step one time in five."""
    rng = random.Random(seed)
    state = {"free": 0, "left": 0}

    def schedule(t, sim):
        if state["left"] == 0:
            state["free"], state["left"] = rng.randrange(sim.robots), rng.randrange(1, 6)
        state["left"] -= 1
        return [] if rng.random() < 0.2 else [i for i in range(sim.robots) if i != state["free"]]

    return schedule


def burst_schedule(seed):
    """One robot at a time delayed for 5-15 consecutive steps, finished robots included."""
    rng = random.Random(seed)
    state = {"robot": 0, "left": 0}

    def schedule(t, sim):
        if state["left"] == 0 and rng.random() < 0.3:
            state["robot"], state["left"] = rng.randrange(sim.robots), rng.randrange(5, 16)
        if state["left"]:
            state["left"] -= 1
            return [state["robot"], state["robot"]]  # a duplicate too
        return []

    return schedule


def schedules(seed):
    return [None, random_delays(0.1, seed), random_delays(0.4, seed + 1), movers_schedule(0.5, seed + 2),
            all_but_one_schedule(seed + 3), burst_schedule(seed + 4)]


# --- instances and plans ---


def add_slack(instance, plan, rng, tries=6):
    """The plan with up to `tries` random waits put in, each kept only if the plan stays valid."""
    plan = [list(p) for p in plan]
    for _ in range(tries):
        i = rng.randrange(len(plan))
        t = rng.randrange(len(plan[i]))
        candidate = [list(p) for p in plan]
        candidate[i] = candidate[i][: t + 1] + [candidate[i][t]] * rng.randrange(1, 3) + candidate[i][t + 1:]
        if pf.validate(instance, candidate).valid:
            plan = candidate
    return plan


def small_instances(seed, count):
    rng = random.Random(seed)
    made = 0
    for _ in range(count * 5):
        if made == count:
            return
        grid = pf.random_map(rng.randrange(3, 9), rng.randrange(2, 8), rng.choice([0, 0, 10, 20, 30]),
                             rng.randrange(1 << 30))
        if grid.passable_count < 3:
            continue
        k = rng.randrange(2, min(10, grid.passable_count // 2 + 2))
        try:
            scenario = pf.random_scenario(grid, k, rng.randrange(1 << 30), "r.map")
        except ValueError:  # the largest connected piece is too small for k robots
            continue
        if len(scenario) < k:
            continue
        made += 1
        yield pf.make_instance(grid, scenario, k)


@functools.cache
def small_plans():
    """(instance, plan, kind) for small dense instances: CBS, PP in a random order, each with waits put in."""
    rng = random.Random(4)
    found = []
    for instance in small_instances(4, 120):
        plans = []
        r = pf.cbs(instance, max_nodes=3000)
        if r.solved:
            plans.append(("cbs", [list(p) for p in r.plan]))
        order = list(range(len(instance.agents)))
        rng.shuffle(order)
        r = pf.prioritized_planning(instance, order=order)
        if r.solved:
            plans.append(("pp", [list(p) for p in r.plan]))
        for kind, plan in list(plans):
            plans.append((kind + "+slack", add_slack(instance, plan, rng)))
        found += [(instance, plan, kind) for kind, plan in plans]
    return tuple(found)


def run_ordered_against_reference(instance, plan, schedule):
    sim = Simulator(instance, plan, policy="ordered")
    ref = RefOrdered(plan)
    steps = []
    while not sim.done:
        assert sim.time < 10 * max(len(p) for p in plan) + 1000, "the run didn't finish"
        delayed = list(schedule(sim.time, sim)) if schedule else []
        unfinished = set(sim.unfinished())
        step = sim.step(delayed)
        steps.append(step)
        assert set(step.moved) == ref.step(set(delayed)), (plan, sim.time)
        assert sim.positions == ref.positions()
        assert set(step.delayed) == set(delayed) & unfinished
        if not set(delayed) & unfinished:
            assert step.moved, f"nobody moved at {sim.time - 1} and nobody was delayed"
    return sim, steps


# --- the ordered policy ---


def test_clean_ordered_matches_an_independent_adg_under_adversarial_delays():
    """Covers: no collision, no swap, no deadlock, the plan's visit order kept, maximal moves (an independent
    executor moves exactly the same robots at every step), Step.delayed, delay_steps, costs from the trajectories.
    120 small dense random instances (3-8 x 2-7, 0-30% obstacles, 2-9 robots; 102 with a plan), 362 plans (99 CBS,
    82 PP in a random order, each also with random waits put in; 122 paths revisit a cell), 6 schedules each (none,
    random 0.1 and 0.4, delay-the-movers, everyone-but-one, long bursts): 2,172 runs. The scratch version of this
    (more plan kinds, a different seed) ran 260,064 plan x schedule runs on 5,646 instances, all clean."""
    runs = 0
    for n, (instance, plan, kind) in enumerate(small_plans()):
        planned = pf.validate(instance, plan).costs
        for schedule in schedules(n * 10):
            sim, steps = run_ordered_against_reference(instance, plan, schedule)
            run = sim.execution()
            check = pf.validate(instance, sim.trajectories)
            assert run.completed and check.valid and list(check.costs) == run.costs
            assert visit_order(sim.trajectories) == visit_order(padded(plan))
            assert run.delay_steps == sum(len(s.delayed) for s in steps)
            assert run.makespan == max(run.costs) and run.sum_of_costs == sum(run.costs)
            if schedule is None:
                assert all(c <= p for c, p in zip(run.costs, planned))
                if kind == "cbs":
                    assert run.costs == list(planned)
            runs += 1
    assert runs >= 1500


def test_clean_ordered_without_delays_makes_every_move_no_later_than_planned():
    """DESIGN's induction, checked move by move (not just on the costs): with no delays the k-th distinct cell of
    each robot is entered no later than the plan enters it, on the small plans above (waits put in included)."""
    checked = 0
    for instance, plan, _ in small_plans():
        sim = Simulator(instance, plan)
        sim.run()
        for path, driven in zip(plan, sim.trajectories):
            _, planned_times = dedupe(path)
            _, driven_times = dedupe(driven)
            assert len(planned_times) == len(driven_times)
            assert all(d <= p for d, p in zip(driven_times, planned_times))
            checked += 1
    assert checked >= 1000


def pocket():
    # (0,0) (1,0) (2,0) (3,0) on top, (1,1) a pocket below (1,0). Robot 0 starts on its goal (1,0) and has to step
    # into the pocket for robot 1 to pass from (3,0) to (0,0), then comes back.
    g = pf.parse_map("type octile\nheight 2\nwidth 4\nmap\n....\n@.@@\n")
    c = g.id
    instance = pf.Instance(g, [pf.Agent(c(1, 0), c(1, 0)), pf.Agent(c(3, 0), c(0, 0))])
    plan = [[c(1, 0), c(1, 1), c(1, 1), c(1, 0)], [c(3, 0), c(2, 0), c(1, 0), c(0, 0)]]
    assert pf.validate(instance, plan).valid
    return g, instance, plan


def test_clean_a_robot_that_starts_on_its_goal_and_steps_aside():
    """Kills the surviving mutants that make finished() mean "at its goal" (either policy): robot 0 starts on its
    goal and must step aside, so it isn't finished at t = 0 and a delay to it applies. Expected (worked by hand):
    timetable, robot 0 delayed in steps 0 and 1: it still holds (1,0) at t = 2 when robot 1 gets there (a vertex
    conflict at t = 2), costs [5, 3]; ordered, delayed in step 0: absorbed (the plan's pocket waits are dropped), costs [3, 3]."""
    g, instance, plan = pocket()
    sim = Simulator(instance, plan, policy="timetable")
    assert not sim.finished(0) and sim.unfinished() == [0, 1]
    first = sim.step([0])
    assert first.delayed == (0,) and first.moved == (1,)
    run = sim.run(delay_robot(0, at=1))
    assert [(c.kind, c.a, c.b, c.time, c.cell) for c in run.collisions] == [("vertex", 0, 1, 2, g.id(1, 0))]
    assert run.costs == [5, 3] and run.delay_steps == 2
    sim = Simulator(instance, plan)
    assert sim.step([0]).delayed == (0,)
    run = sim.run()
    assert run.costs == [3, 3] and not run.collisions and run.delay_steps == 1  # the delay is absorbed


def test_clean_a_run_doesnt_stop_while_everyone_is_on_their_goals():
    """Both robots start on their goals; robot 1's plan steps out and back ([g, x, g]: cost 2). At t = 0 everyone
    is at a goal, but robot 1 has its plan still to do: run() must execute it (costs [0, 2], 2 steps). A finished()
    that meant "at its goal" stopped at 0 steps with costs [0, 0] (mutant survived test_simulator.py)."""
    g = pf.Grid(3, 1)
    instance = pf.Instance(g, [pf.Agent(0, 0), pf.Agent(1, 1)])
    plan = [[0], [1, 2, 1]]
    for policy in ("timetable", "ordered"):
        run = Simulator(instance, plan, policy=policy).run()
        assert run.completed and run.steps == 2 and run.costs == [0, 2] and run.makespan == 2


def test_clean_corner_cases_against_the_reference():
    """Hand-built plans, every schedule: a start that is another's goal (corridor, 0 -> 3 behind 1 -> 4), a robot
    passing through another's goal before its owner arrives, the pocket (start on goal, step aside, revisit), a
    5-robot following chain, a 2x2 rotation, single-cell paths."""
    g5 = pf.Grid(5, 1)
    corridor = (pf.Instance(g5, [pf.Agent(0, 3), pf.Agent(1, 4)]), [[0, 1, 2, 3], [1, 2, 3, 4]])
    # Robot 1 (0 -> 2) passes robot 0's goal 1 at t = 1; robot 0 (from the pocket below 1) enters it at t = 2.
    gp = pf.parse_map("type octile\nheight 2\nwidth 3\nmap\n...\n@.@\n")
    through = (pf.Instance(gp, [pf.Agent(gp.id(1, 1), gp.id(1, 0)), pf.Agent(gp.id(0, 0), gp.id(2, 0))]),
               [[gp.id(1, 1), gp.id(1, 1), gp.id(1, 0)], [gp.id(0, 0), gp.id(1, 0), gp.id(2, 0)]])
    g10 = pf.Grid(10, 1)
    chain = (pf.Instance(g10, [pf.Agent(i, i + 5) for i in range(5)]), [list(range(i, i + 6)) for i in range(5)])
    g4 = pf.Grid(2, 2)
    cycle = [0, 1, 3, 2]
    rotation = (pf.Instance(g4, [pf.Agent(cycle[i], cycle[(i + 2) % 4]) for i in range(4)]),
                [[cycle[i], cycle[(i + 1) % 4], cycle[(i + 2) % 4]] for i in range(4)])
    g3 = pf.Grid(3, 1)
    still = (pf.Instance(g3, [pf.Agent(0, 0), pf.Agent(1, 2)]), [[0], [1, 2]])
    _, pocket_instance, pocket_plan = pocket()
    cases = [corridor, through, (pocket_instance, pocket_plan), chain, rotation, still]
    for n, (instance, plan) in enumerate(cases):
        assert pf.validate(instance, plan).valid, n
        for schedule in schedules(100 + n):
            sim, _ = run_ordered_against_reference(instance, plan, schedule)
            assert pf.validate(instance, sim.trajectories).valid
            assert visit_order(sim.trajectories) == visit_order(padded(plan))
    # The following chain with its head delayed 3 steps: nobody can move; then all move together.
    sim = Simulator(*chain)
    run = sim.run(delay_robot(4, at=0, steps=3))
    assert run.costs == [8] * 5 and run.delay_steps == 3


def test_clean_the_ordered_step_check_raises_on_a_collision():
    """Kills the surviving mutant `if found and self.policy == "ordered":` -> `if False:` (the ordered policy's
    collision raise was never exercised): forget who is in cell 1 of the corridor, delay robot 1 there, and robot 0
    walks into it. Right: SimulationError "let robots collide"."""
    g = pf.Grid(5, 1)
    sim = Simulator(pf.Instance(g, [pf.Agent(0, 3), pf.Agent(1, 4)]), [[0, 1, 2, 3], [1, 2, 3, 4]])
    sim._queue[1] = [(0, 1), (1, 0)]  # robot 0's visit to 1 now heads the queue...
    sim._next[1] = 0
    del sim._occupant[1]  # ...and nobody seems to be there
    with pytest.raises(SimulationError, match="collide"):
        sim.step([1])


def test_clean_the_jump_check_fires():
    """Kills the surviving mutant that turns the jump check off: a timetable whose path is edited to teleport robot
    0 from cell 0 to cell 2 must raise "jumped"."""
    g = pf.Grid(5, 1)
    sim = Simulator(pf.Instance(g, [pf.Agent(0, 3), pf.Agent(1, 4)]), [[0, 1, 2, 3], [1, 2, 3, 4]],
                    policy="timetable")
    sim._paths[0] = [0, 2, 2, 3]
    with pytest.raises(SimulationError, match="jumped"):
        sim.step()


# --- the timetable policy ---


def test_clean_timetable_matches_a_reference_and_validates_cells_too():
    """Covers: trajectories (an independent timetable replay of the recorded delays), every collision field against
    validate() (kind, a, b, time, and cell/to, which test_simulator.py doesn't compare: the mutant swapping a swap's
    cell and to survived it), the conflict scanner's count, collisions in step order, costs. Same plans and
    schedules as the ordered check (2,172 runs; the scratch version 260,064, 113,003 of them with a collision and
    6,165 swaps, all matching)."""
    runs = collided = swaps = 0
    for n, (instance, plan, _) in enumerate(small_plans()):
        for schedule in schedules(n * 10):
            log = []

            def recording(t, sim, schedule=schedule):
                d = list(schedule(t, sim)) if schedule else []
                log.append(set(d))
                return d

            sim = Simulator(instance, plan, policy="timetable")
            run = sim.run(recording)
            assert sim.trajectories == ref_timetable(plan, log)
            check = pf.validate(instance, sim.trajectories)
            count, _ = pf.scan_conflicts(instance.grid, sim.trajectories)
            want = sorted((p.kind.name, p.agent, p.other, p.time, p.cell,
                           p.to if p.kind.name == "EDGE_CONFLICT" else None) for p in check.problems)
            got = sorted(("VERTEX_CONFLICT" if c.kind == "vertex" else "EDGE_CONFLICT", c.a, c.b, c.time, c.cell,
                          c.to) for c in run.collisions)
            assert got == want and count == len(want)
            ends = [c.time + (c.kind == "swap") for c in run.collisions]
            assert ends == sorted(ends)
            assert run.completed and run.costs == list(check.costs)
            runs += 1
            collided += bool(run.collisions)
            swaps += sum(c.kind == "swap" for c in run.collisions)
    assert runs >= 1500 and collided >= 300 and swaps >= 20


def test_clean_timetable_moved_leaves_out_the_plans_waits():
    """Kills the surviving mutant that puts every advancing robot in Step.moved: robot 0's plan waits at 0 for a
    step ([0, 0, 1, 2, 3]); in step 0 it advances along its path but doesn't move. Right: moved == (1,)."""
    g = pf.Grid(5, 1)
    instance = pf.Instance(g, [pf.Agent(0, 3), pf.Agent(1, 4)])
    sim = Simulator(instance, [[0, 0, 1, 2, 3], [1, 2, 3, 4]], policy="timetable")
    assert sim.step().moved == (1,)
    assert sim.step().moved == (0, 1)


def test_clean_execution_makespan():
    """Kills the surviving mutant max -> min in Execution.makespan (no test read it): the delayed corridor's
    timetable run costs [3, 4], makespan 4; the ordered run [4, 4], makespan 4."""
    g = pf.Grid(5, 1)
    instance = pf.Instance(g, [pf.Agent(0, 3), pf.Agent(1, 4)])
    plan = [[0, 1, 2, 3], [1, 2, 3, 4]]
    run = Simulator(instance, plan, policy="timetable").run(delay_robot(1, at=0))
    assert run.costs == [3, 4] and run.makespan == 4 and run.sum_of_costs == 7
    run = Simulator(instance, [[0, 1, 2, 3], [1, 2, 3, 4]]).run()
    assert run.makespan == 3


# --- random_delays ---


def test_clean_random_delays_gives_both_policies_the_same_draws_in_real_runs():
    """test_the_same_seed_delays_both_policies_alike calls the schedules on two fresh simulators only, so a
    random_delays drawing only for unfinished robots (which breaks the docstring's promise once the policies'
    finished robots differ) survived it. Here both policies run the same PP plans with the same seed and the draws
    each run saw are compared over the steps both ran: equal (5 warehouse instances x 4 seeds)."""
    grid = pf.warehouse_map(pf.WarehouseLayout())
    compared = 0
    for seed in range(1, 6):
        instance = pf.make_instance(grid, pf.random_scenario(grid, 30, seed, "warehouse.map"), 30)
        plan = pf.prioritized_planning(instance).plan
        for delay_seed in range(4):
            seen = {}
            for policy in ("timetable", "ordered"):
                schedule = random_delays(0.1, delay_seed)
                log = seen[policy] = []
                Simulator(instance, plan, policy=policy).run(lambda t, sim: log.append(schedule(t, sim)) or log[-1])
            n = min(len(seen["timetable"]), len(seen["ordered"]))
            assert seen["timetable"][:n] == seen["ordered"][:n]
            compared += n
    assert compared > 500


# --- DESIGN's claims ---


def test_claim_pp_plans_can_arrive_earlier_under_the_ordered_policy():
    """DESIGN.md says "On prioritized planning's plans none arrived earlier either: each path is the earliest arrival
    around those planned before it, so its waits are all needed". False: pf.random_map(8, 8, 20, 429054828), the
    first 9 agents of pf.random_scenario(grid, 9, 592240811), default order. Undelayed ordered run: robot 6 costs 9,
    planned 10 (SOC 69 against 70). Robot 3 (planned before 6) waits at (2,5) at t = 2-3 and leaves at 4; the
    ordered policy moves it on to (2,6) at t = 3 (it waits there instead, arriving on time), so (2,5) frees a step
    early and robot 6, whose wait at (1,5) was for that cell, gets in at t = 3, not 4. A random search found 15 such
    robots in ~51,000 PP plans with random orders (556,778 robots), and this one in 1,268 default-order plans."""
    grid = pf.random_map(8, 8, 20, 429054828)
    instance = pf.make_instance(grid, pf.random_scenario(grid, 9, 592240811, "r.map"), 9)
    result = pf.prioritized_planning(instance)
    assert result.solved
    planned = list(pf.validate(instance, result.plan).costs)
    run = Simulator(instance, result.plan).run()
    assert planned[6] == 10 and run.costs[6] == 9
    assert run.sum_of_costs == 69 and result.sum_of_costs == 70
    assert [c for i, c in enumerate(run.costs) if i != 6] == [c for i, c in enumerate(planned) if i != 6]


# --- scripts/sweep.py ---


def load_script(name):
    spec = importlib.util.spec_from_file_location(f"{name}_under_review_d4", ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Fake:
    """A real PlanResult with some fields replaced."""

    def __init__(self, real, **fields):
        self._real = real
        self.__dict__.update(fields)

    def __getattr__(self, name):
        return getattr(self._real, name)


def test_clean_sweep_statistics_give_up_and_exit_code():
    """sweep.py on generated-warehouse, 10 and 20 agents, scenarios 1-4, with the planners' clocks replaced by
    known numbers and CBS made to fail every try at 10 agents (status LIMIT). Worked by hand: PP's times at 10
    agents are 0.1, 0.4, 0.2, 0.3 s: median 250 ms (the mean of the middle two), max 400 ms; CBS 0 / 4 at 10, so not
    tried at 20 ("-") unless --keep-going; both solved 0 at 10 ("-" ratios); exit 0. Then CBS's SOC is made one
    above PP's on scenario 2: the script must exit 1."""
    sweep = load_script("sweep")
    real_run = sweep.run
    times = iter([0.1, 0.4, 0.2, 0.3] + [0.5] * 100)
    limit = types.SimpleNamespace(name="LIMIT")

    def fake_run(planner, instance, time_limit):
        r = real_run(planner, instance, time_limit)
        if planner == "cbs" and len(instance.agents) == 10:
            return Fake(r, status=limit, solved=False, seconds=1.0)
        return Fake(r, seconds=next(times)) if planner == "pp" else r

    sweep.run = fake_run
    args = ["--maps", "generated-warehouse", "--agents", "10", "20", "--scenarios", "1-4", "--time-limit", "2"]
    out = io.StringIO()
    with redirect_stdout(out), redirect_stderr(io.StringIO()):
        assert sweep.main(args) == 0
    rows = {int(m.group(1)): [c.strip() for c in m.group(2).split("|")]
            for m in re.finditer(r"^\| (\d+) \|(.*)\|$", out.getvalue(), re.M)}
    assert rows[10][:5] == ["4 / 4", "0 / 4", "250.0 ms / 400.0 ms", "1.00 s / 1.00 s", "0"]
    assert rows[10][5:] == ["-", "-", "-"]
    assert rows[20][1] == "-" and rows[20][3] == "-"

    # The planners' answers swapped (both plans valid, both at or above the lower bound): wherever PP's SOC was above
    # CBS's, "CBS" is now above "PP", and the script must exit 1 (scenario 1 at 10 agents has PP 1.0676 x CBS).
    def swapped(planner, instance, time_limit):
        return real_run("cbs" if planner == "pp" else "pp", instance, time_limit)

    sweep.run = swapped
    err = io.StringIO()
    with redirect_stdout(io.StringIO()), redirect_stderr(err):
        assert sweep.main(["--maps", "generated-warehouse", "--agents", "10", "--scenarios", "1-3"]) == 1
    assert "CBS's SOC is above prioritized planning's" in err.getvalue()
    assert "doesn't check out" not in err.getvalue()


# --- scripts/delays.py ---


def test_clean_delays_numbers_against_a_recomputation():
    """delays.py on generated-warehouse, 10 agents, scenarios 1-3, p 0.1, 3 seeds, recomputed here from the
    simulator directly: runs, the share of timetable runs with a collision, collisions per run, delay robot-steps
    per run, extra SOC per delay and extra makespan per run all match the printed row (both planners). (Written at
    ed1cf44, where each planner used its own solved instances; delays.py now uses only the instances both solve.
    Both planners solve all three here, so the two are the same set.)"""
    delays = load_script("delays")
    out = io.StringIO()
    args = ["--map", "generated-warehouse", "--agents", "10", "--scenarios", "1-3", "--probabilities", "0.1",
            "--seeds", "3", "--time-limit", "5"]
    with redirect_stdout(out):
        assert delays.main(args) == 0
    grid = pf.warehouse_map(pf.WarehouseLayout())
    for name, planner in (("pp", pf.prioritized_planning), ("cbs", pf.cbs)):
        runs = collided = collisions = delay_steps = extra = makespan = 0
        for n in range(1, 4):
            instance = pf.make_instance(grid, pf.random_scenario(grid, 10, n, "warehouse.map"), 10)
            r = planner(instance, time_limit=5)
            if not r.solved:
                continue
            planned = pf.validate(instance, r.plan)
            for seed in range(3):
                tt = Simulator(instance, r.plan, policy="timetable").run(random_delays(0.1, seed))
                od = Simulator(instance, r.plan).run(random_delays(0.1, seed))
                runs += 1
                collided += bool(tt.collisions)
                collisions += len(tt.collisions)
                delay_steps += od.delay_steps
                extra += od.sum_of_costs - planned.sum_of_costs
                makespan += od.makespan - planned.makespan
        want = (f"| {name} | 0.1 | {runs} | {collided / runs:.1%} | {collisions / runs:.2f} | "
                f"{delay_steps / runs:.2f} | {extra / delay_steps:.3f} | {makespan / runs:.2f} |")
        assert want in out.getvalue().splitlines(), (want, out.getvalue())
