"""The day-2 third-round review (commit 98f60c4, reviewing 4db3e63..98f60c4). It found no bug: every test here is a
test_review_d2c_clean_* check that passed on that commit, kept for its coverage."""

import os
import subprocess
import sys
import textwrap

import pytest

import pathfinder as pf

# prioritized_planning releases the GIL on the claim that Python can't change an Instance. Three threads plan the same
# instance at once while a fourth does everything Python can do to it: call Instance.__init__ on it again (pybind11
# ignores a second __init__), read .grid and .agents (copies), and pass it to the other functions taking one
# (unreachable_agents, validate). Every plan must equal the one planned alone, and the instance must be unchanged.
# In a child process (a crash fails this test only), bounded by its own clock and the subprocess timeout.
RACE = textwrap.dedent(
    """
    import sys, threading, time
    import pathfinder as pf

    limit = float(sys.argv[1])
    g = pf.warehouse_map()
    inst = pf.make_instance(g, pf.random_scenario(g, 40, 11, "w"), 40)
    alone = pf.prioritized_planning(inst)
    assert alone.solved, alone.status
    grid0, agents0 = inst.grid, inst.agents
    want = [list(p) for p in alone.plan]
    deadline = time.monotonic() + limit
    bad = []

    def plan():
        n = 0
        while time.monotonic() < deadline and n < 100:
            r = pf.prioritized_planning(inst, order=list(range(40)))
            if r.status != alone.status or [list(p) for p in r.plan] != want or r.sum_of_costs != alone.sum_of_costs:
                bad.append(n)
            n += 1

    def meddle():
        small = pf.Grid(3, 3)
        n = 0
        while time.monotonic() < deadline and n < 100_000:
            pf.Instance.__init__(inst, small, [pf.Agent(0, 1)])
            pf.Instance.__init__(inst, small, [])
            _ = inst.grid, inst.agents
            pf.unreachable_agents(inst)
            pf.validate(inst, alone.plan)
            n += 1

    threads = [threading.Thread(target=plan) for _ in range(3)] + [threading.Thread(target=meddle)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not bad, bad
    assert inst.grid == grid0 and inst.agents == agents0
    print("survived")
    """
)


def test_review_d2c_clean_prioritized_planning_is_unchanged_by_other_threads_using_its_instance():
    done = subprocess.run(
        [sys.executable, "-c", RACE, "2"],
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert done.returncode == 0, f"the child died with status {done.returncode} (negative: a signal): {done.stderr}"
    assert "survived" in done.stdout


def test_review_d2c_clean_a_second_instance_init_changes_nothing():
    g = pf.Grid(4, 4)
    inst = pf.Instance(g, [pf.Agent(0, 15), pf.Agent(3, 12)])
    pf.Instance.__init__(inst, pf.Grid(2, 2), [pf.Agent(0, 1)])
    assert inst.grid == g and inst.agents == [pf.Agent(0, 15), pf.Agent(3, 12)]
    got = inst.grid
    got.set_passable(1, 1, False)
    assert inst.grid == g  # a copy


# move_free: IndexError for a cell off the grid (either one, before the neighbour test), ValueError for any two cells
# that aren't 4-neighbours (a wait included), and an answer for every neighbour pair; on every grid up to 6 x 6.
# forbid_move agrees, and a refused forbid_move changes nothing.
@pytest.mark.parametrize("w", range(1, 7))
def test_review_d2c_clean_move_free_refuses_exactly_the_non_neighbours(w):
    for h in range(1, 7):
        g = pf.Grid(w, h)
        r = pf.ReservationTable(g)
        n = g.cell_count
        for a in range(n):
            near = set(g.neighbors(a))
            for b in range(n):
                if b in near:
                    assert r.move_free(a, b, 0) is True
                    continue
                with pytest.raises(ValueError, match="aren't 4-neighbours"):
                    r.move_free(a, b, 0)
                with pytest.raises(ValueError):
                    r.forbid_move(a, b, 0)
            for off in (n, n + w, 2**32 - 1):
                with pytest.raises(IndexError):
                    r.move_free(a, off, 0)
                with pytest.raises(IndexError):
                    r.move_free(off, a, 0)
                with pytest.raises(IndexError):
                    r.forbid_move(off, a, 0)
        assert r.empty and r.horizon == 0


def test_review_d2c_clean_move_free_answers_from_the_table_on_a_one_wide_grid():
    g = pf.Grid(1, 4)
    r = pf.ReservationTable(g)
    r.reserve_path([0, 1, 2])
    assert not r.move_free(1, 0, 0) and not r.move_free(2, 1, 1)
    assert r.move_free(0, 1, 0) and r.move_free(1, 2, 1) and r.move_free(1, 0, 1)
    with pytest.raises(ValueError):
        r.move_free(0, 2, 0)
