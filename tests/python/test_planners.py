"""The planners through the Python module: the same facts the C++ tests pin, so the bindings can't drift."""

import math

import pytest

import pathfinder as pf


def corridor_room():
    # A dead-end corridor (2,0)-(3,0)-(4,0) off a 2 x 2 room (DESIGN.md "The planners": prioritized planning is
    # incomplete).
    return pf.parse_map("type octile\nheight 2\nwidth 5\nmap\n.....\n..@@@\n")


def test_reservation_table_entries():
    g = pf.Grid(4, 3)
    r = pf.ReservationTable(g)
    assert r.empty and r.horizon == 0
    r.forbid_vertex(5, 3)
    r.forbid_move(5, 6, 1)
    r.forbid_from(11, 7)
    assert not r.vertex_free(5, 3) and r.vertex_free(5, 4)
    assert not r.move_free(5, 6, 1) and r.move_free(6, 5, 1)
    assert not r.vertex_free(11, 7) and not r.vertex_free(11, 10**6) and r.vertex_free(11, 6)
    assert r.last_vertex_time(5) == 3 and r.last_vertex_time(0) is None
    assert r.forbidden_from(11) == 7 and r.forbidden_from(5) is None
    assert r.horizon == 7
    assert repr(r) == "ReservationTable(1 vertex, 1 move, 1 from-on entries; horizon 7)"


def test_reservation_table_errors():
    g = pf.Grid(4, 3)
    r = pf.ReservationTable(g)
    with pytest.raises(ValueError, match="aren't 4-neighbours"):
        r.forbid_move(3, 4, 0)  # consecutive ids across a row's end
    with pytest.raises(IndexError):
        r.forbid_vertex(12, 0)
    with pytest.raises(IndexError):
        r.vertex_free(12, 0)
    with pytest.raises(ValueError, match="latest allowed"):
        r.forbid_vertex(0, pf.MAX_RESERVATION_TIME + 1)
    with pytest.raises(TypeError):
        r.forbid_vertex(0, -1)
    # A bad path changes nothing.
    with pytest.raises(ValueError):
        r.reserve_path([0, 1, 2, 3, 4])
    with pytest.raises(ValueError):
        r.reserve_path([])
    assert r.empty


def test_space_time_astar_waits_for_a_reserved_cell():
    g = pf.Grid(5, 1)
    r = pf.ReservationTable(g)
    r.forbid_vertex(2, 2)
    s = pf.space_time_astar(g, 0, 4, r)
    assert s.status == pf.SearchStatus.FOUND
    assert s.path == [0, 1, 1, 2, 3, 4]
    assert s.expansions > 0 and s.generated >= s.expansions


def test_space_time_astar_statuses_and_limits():
    g = pf.Grid(30, 30)
    r = pf.ReservationTable(g)
    goal = g.id(29, 29)
    r.forbid_from(g.id(28, 29), 0)
    r.forbid_from(g.id(29, 28), 0)  # the goal's two neighbours, closed for ever
    assert pf.space_time_astar(g, 0, goal, r).status == pf.SearchStatus.NO_PATH
    limited = pf.space_time_astar(g, 0, goal, r, max_expansions=10)
    assert limited.status == pf.SearchStatus.LIMIT and limited.expansions == 10 and limited.path == []
    with pytest.raises(ValueError, match="another size"):
        pf.space_time_astar(pf.Grid(5, 5), 0, 4, r)
    with pytest.raises(IndexError):
        pf.space_time_astar(g, 0, 900, r)
    with pytest.raises(ValueError, match="time limit"):
        pf.space_time_astar(g, 0, 5, r, time_limit=-1.0)
    with pytest.raises(ValueError, match="time limit"):
        pf.space_time_astar(g, 0, 5, r, time_limit=math.nan)
    assert pf.space_time_astar(g, 0, 5, r, time_limit=math.inf).status == pf.SearchStatus.FOUND


def test_prioritized_planning_on_a_warehouse_floor():
    g = pf.warehouse_map()
    inst = pf.make_instance(g, pf.random_scenario(g, 30, 1, "w.map"), 30)
    r = pf.prioritized_planning(inst)
    assert r.solved and r.status == pf.PlanStatus.SOLVED
    check = pf.validate(inst, r.plan)
    assert check.valid
    assert (r.sum_of_costs, r.makespan) == (check.sum_of_costs, check.makespan)
    lower_bound = sum(pf.distances(g, a.goal)[a.start] for a in inst.agents)
    assert r.sum_of_costs >= lower_bound
    assert r.expansions > 0 and r.seconds >= 0


def test_prioritized_planning_order_decides_the_corridor():
    g = corridor_room()
    inst = pf.Instance(g, [pf.Agent(g.id(1, 0), g.id(3, 0)), pf.Agent(g.id(0, 1), g.id(4, 0))])
    first = pf.prioritized_planning(inst)
    assert first.status == pf.PlanStatus.NO_PATH and first.failed_agent == 1
    assert first.plan[1] == [] and first.sum_of_costs == 0
    other = pf.prioritized_planning(inst, order=[1, 0])
    assert other.solved and pf.validate(inst, other.plan).valid


def test_prioritized_planning_errors_and_other_statuses():
    g = pf.parse_map("type octile\nheight 1\nwidth 3\nmap\n.@.\n")
    inst = pf.Instance(g, [pf.Agent(0, 2)])
    r = pf.prioritized_planning(inst)
    assert r.status == pf.PlanStatus.UNREACHABLE_GOAL and r.failed_agent == 0
    open_floor = pf.empty_map(20, 20)
    many = pf.make_instance(open_floor, pf.random_scenario(open_floor, 10, 4, "e.map"), 10)
    assert pf.prioritized_planning(many, max_expansions=3).status == pf.PlanStatus.LIMIT
    with pytest.raises(ValueError, match="permutation"):
        pf.prioritized_planning(many, order=[0] * 10)
    with pytest.raises(ValueError, match="empty order"):
        pf.prioritized_planning(many, order=[])
    with pytest.raises(ValueError, match="time limit"):
        pf.prioritized_planning(many, time_limit=-0.5)
    assert pf.prioritized_planning(pf.Instance(open_floor, []), order=[]).solved


def test_cbs_solves_the_corridor_at_its_optimum():
    g = corridor_room()
    inst = pf.Instance(g, [pf.Agent(g.id(1, 0), g.id(3, 0)), pf.Agent(g.id(0, 1), g.id(4, 0))])
    r = pf.cbs(inst)
    assert r.solved and pf.validate(inst, r.plan).valid
    assert (r.sum_of_costs, r.makespan) == (10, 5)  # tests/cbs_test.cpp derives it by hand
    assert r.nodes_generated > r.nodes_expanded > 0
    pp = pf.prioritized_planning(inst)
    assert pp.nodes_generated == 0 and pp.nodes_expanded == 0  # prioritized planning has no tree


def test_cbs_limits_and_statuses():
    pair = pf.Instance(pf.Grid(2, 1), [pf.Agent(0, 1), pf.Agent(1, 0)])  # must swap: no plan
    r = pf.cbs(pair, max_nodes=40)
    assert r.status == pf.PlanStatus.LIMIT and r.nodes_generated <= 40
    assert r.plan == [[], []] and r.sum_of_costs == 0
    assert pf.cbs(pair, max_expansions=100).status == pf.PlanStatus.LIMIT
    assert pf.cbs(pair, time_limit=0.02).status == pf.PlanStatus.LIMIT
    split = pf.parse_map("type octile\nheight 1\nwidth 3\nmap\n.@.\n")
    unreachable = pf.cbs(pf.Instance(split, [pf.Agent(0, 2)]))
    assert unreachable.status == pf.PlanStatus.UNREACHABLE_GOAL and unreachable.failed_agent == 0
    with pytest.raises(ValueError, match="time limit"):
        pf.cbs(pair, time_limit=math.nan)
    with pytest.raises(TypeError):
        pf.cbs(pair, max_nodes=-1)
    assert pf.cbs(pf.Instance(split, [])).solved


def test_cbs_never_costs_more_than_prioritized_planning():
    # Ten agents on the warehouse floor. CBS's effort varies wildly: seeds 1-5 took 8,309, 9, 77, 375,673 and 14,041
    # nodes when this was written, so seed 4 stops at the limit here.
    g = pf.warehouse_map()
    solved = 0
    for seed in range(1, 6):
        inst = pf.make_instance(g, pf.random_scenario(g, 10, seed, "w.map"), 10)
        pp = pf.prioritized_planning(inst)
        r = pf.cbs(inst, max_nodes=20000)
        if r.status == pf.PlanStatus.LIMIT:
            continue
        assert r.solved and pf.validate(inst, r.plan).valid
        assert pp.solved and r.sum_of_costs <= pp.sum_of_costs
        solved += 1
    assert solved == 4


def test_scan_conflicts_matches_validate():
    g = pf.Grid(6, 1)
    plan = [[0, 1], [2, 1, 0], [4, 5], [5, 4]]  # 0 and 1 meet in cell 1 at t = 1; 2 and 3 swap before that
    count, first = pf.scan_conflicts(g, plan)
    assert count == 2
    assert (first.edge, first.a, first.b, first.time, first.cell, first.to) == (True, 2, 3, 0, 4, 5)
    assert repr(first) == "Conflict(swap, a=2, b=3, time=0, cell=4, to=5)"
    assert first == pf.scan_conflicts(g, plan)[1] and first != "a conflict"
    inst = pf.Instance(g, [pf.Agent(p[0], p[-1]) for p in plan])
    assert count == sum(1 for p in pf.validate(inst, plan).problems if "conflict" in str(p.kind).lower())
    assert pf.scan_conflicts(g, [[0, 1, 2], [5, 4, 3]]) == (0, None)
    with pytest.raises(ValueError, match="empty"):
        pf.scan_conflicts(g, [[0], []])
    with pytest.raises(IndexError):
        pf.scan_conflicts(g, [[6]])


def test_cbs_runs_in_threads_without_the_gil():
    # cbs() releases the GIL: four threads planning one instance at once each get the plan it gets alone.
    import threading

    g = pf.warehouse_map()
    inst = pf.make_instance(g, pf.random_scenario(g, 10, 1, "w.map"), 10)  # 8,309 nodes
    alone = pf.cbs(inst, max_nodes=20000)
    assert alone.solved
    results = [None] * 4

    def run(i):
        results[i] = pf.cbs(inst, max_nodes=20000)

    threads = [threading.Thread(target=run, args=(i,)) for i in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    for r in results:
        assert r.solved and r.plan == alone.plan and r.nodes_generated == alone.nodes_generated
