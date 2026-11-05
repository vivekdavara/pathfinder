"""The Python module against the same facts the C++ tests pin, so the bindings can't drift from the library."""

import os
from pathlib import Path

import pytest

import pathfinder as pf

DATA = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[2])) / "tests" / "data"


def fnv1a(text: str) -> int:
    h = 0xCBF29CE484222325
    for b in text.encode():
        h ^= b
        h = (h * 0x100000001B3) % (1 << 64)
    return h


def test_version_matches_the_cmake_project():
    assert pf.__version__ == "0.1.0"


def test_grid_basics():
    g = pf.Grid(4, 3)
    assert (g.width, g.height, g.cell_count, g.passable_count) == (4, 3, 12, 12)
    g.set_passable(1, 1, False)
    assert not g.passable(1, 1)
    assert not g.passable(-1, 0) and not g.passable(4, 0)
    assert g.passable_count == 11
    assert g.id(2, 2) == 10
    assert tuple(g.point(10)) == (2, 2)
    assert g.point(10) == pf.Point(2, 2)
    assert g.neighbors(g.id(1, 0)) == [g.id(2, 0), g.id(0, 0)]  # east, west; south is blocked
    assert repr(g) == "Grid(4x3, 11 passable)"


def test_grid_errors_become_python_exceptions():
    with pytest.raises(ValueError, match="outside 1..10000"):
        pf.Grid(0, 3)
    g = pf.Grid(2, 2)
    with pytest.raises(IndexError):
        g.set_passable(2, 0, False)
    with pytest.raises(IndexError):
        g.id(0, 2)
    with pytest.raises(IndexError):
        g.point(4)
    with pytest.raises(IndexError):
        g.neighbors(4)
    with pytest.raises(IndexError):
        pf.distances(g, 4)


def test_maps_parse_write_and_report_the_line():
    g = pf.parse_map("type octile\nheight 2\nwidth 3\nmap\n.@.\n...\n")
    assert pf.write_map(g) == "type octile\nheight 2\nwidth 3\nmap\n.@.\n...\n"
    assert pf.parse_map(pf.write_map(g)) == g
    with pytest.raises(pf.ParseError) as err:
        pf.parse_map("type octile\nheight 2\nwidth 3\nmap\n...\n.x.\n")
    assert err.value.line == 6
    assert "'x' at column 2" in str(err.value)
    assert isinstance(err.value, ValueError)
    loaded = pf.load_map(DATA / "terrain.map")
    assert (loaded.width, loaded.height, loaded.passable_count) == (7, 3, 17)
    with pytest.raises(pf.ParseError) as err:
        pf.load_map(DATA / "short_row.map")
    assert err.value.line == 6 and "short_row.map" in str(err.value)
    with pytest.raises(RuntimeError):
        pf.load_map(DATA / "no_such_file.map")


def test_scenarios_and_instances():
    s = pf.load_scenario(DATA / "small.scen")
    assert len(s) == 3
    e = s.entries[0]
    assert (e.map, e.width, e.height, e.start, e.goal, e.optimal_length) == (
        "small.map", 5, 4, pf.Point(0, 0), pf.Point(4, 3), 6.41421356)
    assert pf.parse_scenario(pf.write_scenario(s)) == s
    g = pf.load_map(DATA / "small.map")
    inst = pf.make_instance(g, s, 2)
    assert [(a.start, a.goal) for a in inst.agents] == [(g.id(0, 0), g.id(4, 3)), (g.id(4, 0), g.id(0, 2))]
    for entry in s.entries:
        assert pf.octile_distances(g, g.id(*entry.start))[g.id(*entry.goal)] == pytest.approx(entry.optimal_length,
                                                                                              abs=1e-8)
    with pytest.raises(ValueError, match="has 3 entries, not 4"):
        pf.make_instance(g, s, 4)
    with pytest.raises(ValueError, match="also agent 0's goal"):
        pf.Instance(g, [pf.Agent(0, 5), pf.Agent(1, 5)])
    split = pf.parse_map("type octile\nheight 1\nwidth 3\nmap\n.@.\n")
    assert pf.unreachable_agents(pf.Instance(split, [pf.Agent(0, 2)])) == [0]


def test_distances_and_components():
    g = pf.parse_map("type octile\nheight 1\nwidth 5\nmap\n..@..\n")
    assert pf.distances(g, 0) == [0, 1, pf.UNREACHABLE, pf.UNREACHABLE, pf.UNREACHABLE]
    c = pf.components(g)
    assert c.label == [0, 0, pf.NO_COMPONENT, 1, 1]
    assert c.sizes == [2, 2] and c.largest == 0


def test_generators_match_the_cpp_pins():
    # The same FNV-1a values as tests/generate_test.cpp: the bindings run the same code.
    assert fnv1a(pf.write_map(pf.random_map(32, 32, 20.0, 1))) == 0xCCBA3A993502F7FA
    assert fnv1a(pf.write_map(pf.random_map(64, 48, 35.5, 42))) == 0x751A555B243E850A
    warehouse = pf.warehouse_map()
    assert fnv1a(pf.write_map(warehouse)) == 0x808A76F5F88E0D90
    assert fnv1a(pf.write_scenario(pf.random_scenario(warehouse, 50, 2026, "warehouse.map"))) == 0x3E3C169C6DC3B76C
    small = pf.warehouse_map(pf.WarehouseLayout(shelf_rows=2, shelf_columns=3, shelf_width=3, shelf_height=1,
                                                aisle=1, margin=1))
    assert pf.write_map(small).splitlines()[4:] == [".............", ".@@@.@@@.@@@.", ".............",
                                                    ".@@@.@@@.@@@.", "............."]
    with pytest.raises(ValueError):
        pf.warehouse_map(pf.WarehouseLayout(aisle=0))
    with pytest.raises(ValueError):
        pf.random_scenario(pf.Grid(2, 1), 2, 1, "m.map")


def test_validate_accepts_a_valid_plan_and_reports_each_problem():
    g = pf.Grid(4, 1)
    inst = pf.Instance(g, [pf.Agent(0, 2), pf.Agent(3, 1)])
    met = pf.validate(inst, [[0, 1, 2], [3, 3, 3, 2, 1]])  # agent 1 walks into agent 0, parked on its goal
    assert not met.valid
    assert [p.kind for p in met.problems] == [pf.ProblemKind.VERTEX_CONFLICT]
    assert pf.describe(met.problems[0], g) == "agents 0 and 1 are both at (2, 0) at t=3"
    apart = pf.Instance(g, [pf.Agent(0, 1), pf.Agent(3, 2)])
    good = pf.validate(apart, [[0, 0, 1], [3, 2, 2]])
    assert good.valid
    assert good.costs == [2, 1] and good.sum_of_costs == 3 and good.makespan == 2
    swap = pf.validate(pf.Instance(g, [pf.Agent(1, 2), pf.Agent(2, 1)]), [[1, 2], [2, 1]])
    assert [p.kind for p in swap.problems] == [pf.ProblemKind.EDGE_CONFLICT]
    wrong = pf.validate(inst, [[0, 2]])
    assert [p.kind for p in wrong.problems] == [pf.ProblemKind.WRONG_PATH_COUNT]
    jump = pf.validate(inst, [[0, 2], [3, 2, 1]])
    assert [p.kind for p in jump.problems] == [pf.ProblemKind.ILLEGAL_MOVE, pf.ProblemKind.VERTEX_CONFLICT]
    assert jump.costs == []


def test_path_cost():
    assert pf.path_cost([5, 6, 7, 7], 7) == 2
    assert pf.path_cost([7, 6, 7], 7) == 2
    with pytest.raises(ValueError):
        pf.path_cost([], 7)
    with pytest.raises(ValueError):
        pf.path_cost([5, 6], 7)


def test_plan_files_round_trip_and_validate():
    g = pf.load_map(DATA / "small.map")
    plan = pf.load_plan(DATA / "small.plan", g)
    assert [len(p) for p in plan] == [8, 7]
    assert pf.parse_plan(pf.write_plan(plan, g), g) == plan
    check = pf.validate(pf.make_instance(g, pf.load_scenario(DATA / "small.scen"), 2), plan)
    assert check.valid and check.sum_of_costs == 13 and check.makespan == 7
    with pytest.raises(pf.ParseError) as err:
        pf.parse_plan("version 1\nagents 1\n0 (5,0)\n", g)
    assert err.value.line == 3 and "outside the 5x4 map" in str(err.value)
    with pytest.raises(ValueError):
        pf.write_plan([[]], g)
