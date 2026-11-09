"""The step-by-step simulator (pathfinder.simulator): it replays plans exactly, sees every collision a delay causes,
and its own step checks agree with validate() on what it drove."""

import functools

import pytest

import pathfinder as pf
from pathfinder.simulator import Collision, SimulationError, Simulator, delay_robot, random_delays


def corridor():
    # A 5 x 1 corridor: robot 0 from (0,0) to (3,0) right behind robot 1 from (1,0) to (4,0). The plan has robot 0
    # follow robot 1 one cell behind, so it has no slack at all.
    g = pf.Grid(5, 1)
    instance = pf.Instance(g, [pf.Agent(0, 3), pf.Agent(1, 4)])
    plan = [[0, 1, 2, 3], [1, 2, 3, 4]]
    return instance, plan


def warehouse_instances(agents, seeds):
    grid = pf.warehouse_map(pf.WarehouseLayout())
    for seed in seeds:
        yield pf.make_instance(grid, pf.random_scenario(grid, agents, seed, "warehouse.map"), agents)


def padded(plan, length):
    return [list(p) + [p[-1]] * (length - len(p)) for p in plan]


def test_the_timetable_policy_replays_a_plan_exactly():
    for instance in warehouse_instances(30, range(1, 6)):
        result = pf.prioritized_planning(instance)
        assert result.solved
        sim = Simulator(instance, result.plan, policy="timetable")
        run = sim.run()
        assert run.completed and not run.collisions and run.delay_steps == 0
        assert run.costs == pf.validate(instance, result.plan).costs
        assert run.sum_of_costs == result.sum_of_costs == run.planned_sum_of_costs and run.extra_cost == 0
        assert run.steps == max(len(p) for p in result.plan) - 1
        assert sim.trajectories == padded(result.plan, run.steps + 1)


def test_a_delayed_leader_is_rear_ended():
    instance, plan = corridor()
    sim = Simulator(instance, plan, policy="timetable")
    first = sim.step(delayed=[1])
    assert first.time == 1 and first.moved == (0,) and first.delayed == (1,)
    assert first.collisions == (Collision("vertex", 0, 1, 1, 1),)
    run = sim.run()
    # Robot 1 is a step behind for the rest of the run, so robot 0 sits in its cell at every step until it stops.
    assert [c.time for c in run.collisions] == [1, 2, 3]
    assert all(c.kind == "vertex" for c in run.collisions)
    assert run.costs == [3, 4] and run.extra_cost == 1 and run.delay_steps == 1


def test_a_delay_can_make_two_robots_swap():
    # Robot 0 goes (0,0) -> (2,0) through the bottom row's middle; robot 1 waits in the pocket (1,1) for it to pass,
    # then goes up to (1,0) and on to (0,0). Delay robot 0 by three steps and they meet head on between (0,0) and
    # (1,0).
    g = pf.parse_map("type octile\nheight 2\nwidth 3\nmap\n...\n@.@\n")
    instance = pf.Instance(g, [pf.Agent(g.id(0, 0), g.id(2, 0)), pf.Agent(g.id(1, 1), g.id(0, 0))])
    a = [g.id(0, 0), g.id(1, 0), g.id(2, 0)]
    b = [g.id(1, 1), g.id(1, 1), g.id(1, 1), g.id(1, 0), g.id(0, 0)]
    assert pf.validate(instance, [a, b]).valid
    sim = Simulator(instance, [a, b], policy="timetable")
    run = sim.run(delay_robot(0, at=0, steps=3))
    kinds = [(c.kind, c.time) for c in run.collisions]
    assert ("swap", 3) in kinds, run.collisions


def test_timetable_collisions_are_exactly_the_conflicts_validate_finds():
    # The simulator's own checks against the library's two independent ones (validate and the conflict scanner) on
    # the trajectories the robots actually drove, over many delayed runs. Scenario 5 with delay seed 3 has two robots
    # stacked in one cell making the same move while a third comes the other way: two swaps, of which the first
    # version of the check (one robot per move) saw one.
    runs = collided = 0
    for instance in warehouse_instances(30, range(1, 6)):
        result = pf.prioritized_planning(instance)
        assert result.solved
        plan = result.plan
        for seed in range(8):
            sim = Simulator(instance, plan, policy="timetable")
            run = sim.run(random_delays(0.1, seed))
            assert run.completed
            check = pf.validate(instance, sim.trajectories)
            count, _ = pf.scan_conflicts(instance.grid, sim.trajectories)
            assert len(run.collisions) == len(check.problems) == count
            want = sorted((p.kind.name, p.agent, p.other, p.time) for p in check.problems)
            got = sorted(("VERTEX_CONFLICT" if c.kind == "vertex" else "EDGE_CONFLICT", c.a, c.b, c.time)
                         for c in run.collisions)
            assert got == want
            assert run.costs == list(check.costs)
            runs += 1
            collided += bool(run.collisions)
    assert runs == 40 and collided > 20  # delays at 10% a step nearly always break a plan with no slack


def test_a_delayed_robot_falls_behind_by_its_delay_and_nobody_else_changes():
    instance = next(warehouse_instances(10, [2]))  # CBS solves it in 9 nodes (README "Results (day 3)")
    result = pf.cbs(instance, max_nodes=20_000)
    assert result.solved
    plan = result.plan
    planned = pf.validate(instance, plan).costs
    robot = max(range(10), key=lambda i: planned[i])
    sim = Simulator(instance, plan, policy="timetable")
    run = sim.run(delay_robot(robot, at=2, steps=3))
    assert run.delay_steps == 3
    assert run.costs == [c + 3 if i == robot else c for i, c in enumerate(planned)]
    # Its trajectory is its path with the three waits put in at time 2.
    path = plan[robot]
    assert sim.trajectories[robot][: len(path) + 3] == path[:3] + [path[2]] * 3 + path[3:]


def test_delays_of_finished_robots_dont_count():
    instance, plan = corridor()
    sim = Simulator(instance, plan, policy="timetable")
    sim.run()
    assert sim.done
    step = sim.step(delayed=[0, 1])
    assert step.delayed == () and step.moved == () and sim.execution().delay_steps == 0


def test_errors():
    instance, plan = corridor()
    with pytest.raises(ValueError, match="policy"):
        Simulator(instance, plan, policy="fast")
    with pytest.raises(ValueError, match="isn't valid"):
        Simulator(instance, [[0, 1, 2, 3], [1, 1, 2, 3, 4]])  # robot 0 runs into robot 1 at t = 1
    with pytest.raises(ValueError, match="isn't valid"):
        Simulator(instance, [[0, 1, 2, 3]])
    sim = Simulator(instance, plan, policy="timetable")
    with pytest.raises(IndexError):
        sim.step(delayed=[2])
    with pytest.raises(ValueError, match="complete"):
        _ = sim.execution().extra_cost
    with pytest.raises(ValueError):
        random_delays(1.5, 0)
    with pytest.raises(ValueError):
        delay_robot(0, at=-1)


def test_a_run_stops_at_max_steps():
    instance, plan = corridor()
    sim = Simulator(instance, plan, policy="timetable")
    run = sim.run(random_delays(1.0, 0), max_steps=5)  # everyone delayed at every step: nobody ever moves
    assert not run.completed and run.steps == 5 and run.costs == [] and run.delay_steps == 10
    assert issubclass(SimulationError, RuntimeError)


# --- the ordered policy ---


def random_instances(agents, seeds, size=16, obstacles=20):
    for seed in seeds:
        grid = pf.random_map(size, size, obstacles, seed)
        yield pf.make_instance(grid, pf.random_scenario(grid, agents, seed, "random.map"), agents)


@functools.cache
def cbs_plans():
    # Solved CBS plans (optimal), from instances small enough to solve fast in a Debug build; computed once (the
    # simulator changes neither instances nor plans).
    found = []
    for instance in list(warehouse_instances(10, range(1, 6))) + list(random_instances(12, range(1, 11))):
        result = pf.cbs(instance, max_nodes=20_000)
        if result.solved:
            found.append((instance, result))
    assert len(found) >= 10
    return tuple(found)


def visit_order(trajectories):
    # Each cell's visitors in the order they entered it (a robot entering again counts again).
    order = {}
    events = []
    for i, cells in enumerate(trajectories):
        for t, c in enumerate(cells):
            if t == 0 or cells[t - 1] != c:
                events.append((t, c, i))
    for t, c, i in sorted(events):
        order.setdefault(c, []).append(i)
    return order


def test_ordered_follower_waits_for_a_delayed_leader():
    instance, plan = corridor()
    sim = Simulator(instance, plan)  # the default policy
    assert sim.policy == "ordered"
    first = sim.step(delayed=[1])
    assert first.moved == () and first.collisions == ()  # robot 0 holds: robot 1 is still in the cell it wants
    run = sim.run()
    assert not run.collisions and run.costs == [4, 4] and run.extra_cost == 2


def test_ordered_drops_the_plans_waits():
    instance, _ = corridor()
    plan = [[0, 0, 0, 1, 2, 3], [1, 2, 3, 4]]  # robot 0 waits for nothing
    run = Simulator(instance, plan).run()
    assert run.costs == [3, 3] and run.extra_cost == -2
    timetable = Simulator(instance, plan, policy="timetable").run()
    assert timetable.costs == [5, 3]


def test_ordered_moves_a_rotation_in_one_step():
    # Four robots on a 2 x 2 grid each move to the next cell clockwise at once: a rotation, which the model allows.
    # Each waits for the robot ahead to leave, so the moves must be decided together, not one robot at a time.
    g = pf.Grid(2, 2)
    cycle = [0, 1, 3, 2]
    instance = pf.Instance(g, [pf.Agent(cycle[i], cycle[(i + 1) % 4]) for i in range(4)])
    plan = [[cycle[i], cycle[(i + 1) % 4]] for i in range(4)]
    sim = Simulator(instance, plan)
    step = sim.step()
    assert step.moved == (0, 1, 2, 3) and sim.done and sim.execution().costs == [1, 1, 1, 1]
    # Hold one of them and nobody can move.
    sim = Simulator(instance, plan)
    assert sim.step(delayed=[2]).moved == ()


def test_ordered_execution_of_an_optimal_plan_costs_exactly_the_plan():
    # With no delays every move happens no later than planned, so no robot's cost can go up; CBS's SOC is the
    # least any valid plan has, and the executed trajectories are one, so none can go down either.
    for instance, result in cbs_plans():
        sim = Simulator(instance, result.plan)
        run = sim.run()
        assert run.completed and not run.collisions
        assert run.costs == pf.validate(instance, result.plan).costs
        assert pf.validate(instance, sim.trajectories).valid


def test_ordered_execution_never_makes_a_robot_later_than_planned():
    # The guarantee is "no later". On prioritized planning's plans it comes out "exactly on time" for every robot
    # here: each path is the earliest arrival around the robots planned before it, so its waits are all needed in
    # the order the plan fixes (a first version of this test expected some to be dropped, and none ever was).
    solved = 0
    for instance in list(warehouse_instances(30, range(1, 6))) + list(random_instances(25, range(1, 11))):
        result = pf.prioritized_planning(instance)
        if not result.solved:
            continue
        sim = Simulator(instance, result.plan)
        run = sim.run()
        planned = pf.validate(instance, result.plan).costs
        assert all(c <= p for c, p in zip(run.costs, planned))
        assert pf.validate(instance, sim.trajectories).valid
        solved += 1
    assert solved >= 10


@pytest.mark.parametrize("probability", [0.05, 0.2, 0.5])
def test_ordered_execution_under_random_delays_never_collides_and_keeps_the_plans_order(probability):
    runs = 0
    for instance, result in cbs_plans():
        planned_order = visit_order(padded(result.plan, max(len(p) for p in result.plan)))
        for seed in range(5):
            sim = Simulator(instance, result.plan)
            run = sim.run(random_delays(probability, seed))
            assert run.completed and not run.collisions  # a collision or a deadlock would have raised
            check = pf.validate(instance, sim.trajectories)
            assert check.valid and list(check.costs) == run.costs
            assert run.sum_of_costs >= result.sum_of_costs  # nothing valid beats CBS
            assert visit_order(sim.trajectories) == planned_order
            runs += 1
    assert runs >= 50


def test_the_same_seed_delays_both_policies_alike():
    instance = next(warehouse_instances(30, [1]))
    plan = pf.prioritized_planning(instance).plan
    a = Simulator(instance, plan, policy="timetable")
    b = Simulator(instance, plan, policy="ordered")
    da, db = random_delays(0.1, 7), random_delays(0.1, 7)
    for t in range(10):
        assert list(da(t, a)) == list(db(t, b))


def test_the_ordered_policys_guards_fire_on_a_broken_order():
    # No valid plan can make the ordered policy deadlock or swap (Simulator's docstring), so these break its state
    # by hand: the guards are what stops a bug there from hanging run() or driving robots through each other.
    instance, plan = corridor()
    sim = Simulator(instance, plan)
    sim._queue[2].reverse()  # robot 0 must now enter cell 2 before robot 1, who is in the cell robot 0 needs first
    with pytest.raises(SimulationError, match="deadlock"):
        sim.run(max_steps=100)  # bounded: without the guard this would loop for ever
    sim = Simulator(instance, plan)
    sim._cells = [[0, 1], [1, 0]]
    sim._queue = {0: [(0, 0), (1, 1)], 1: [(1, 0), (0, 1)]}
    with pytest.raises(SimulationError, match="would swap"):  # before moving; the step check would also catch it
        sim.step()
