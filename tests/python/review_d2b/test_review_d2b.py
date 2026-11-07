"""The day-2 second-round review (commit 4db3e63, reviewing 5089fd9..4db3e63): each test outside the
ReviewD2bClean section failed on that commit because of the bug its comment names."""

import os
import subprocess
import sys
import textwrap

import pytest

import pathfinder as pf

# A search reading a reservation table with the GIL released while a second Python thread adds entries to the same
# table. Run in a child process, so a crash fails this test only; bounded by its own clock (`limit` seconds) and by
# the subprocess timeout.
RACE = textwrap.dedent(
    """
    import sys, threading, time
    import pathfinder as pf

    limit = float(sys.argv[1])
    g = pf.Grid(40, 40)
    r = pf.ReservationTable(g)
    goal = g.cell_count - 1
    for c in range(g.cell_count):
        r.forbid_vertex(c, 1_000_000 + c)  # every cell named, so every lookup goes to the hash tables
    deadline = time.monotonic() + limit
    done = False

    def search():
        global done
        while time.monotonic() < deadline and not done:
            pf.space_time_astar(g, 0, goal, r, max_expansions=300_000)
        done = True

    def mutate():
        n = 0
        while not done and n < 4_000_000:
            r.forbid_vertex(n % (g.cell_count - 1), 2_000_000 + n)  # new keys: the hash set grows and rehashes
            n += 1

    a = threading.Thread(target=search)
    b = threading.Thread(target=mutate)
    a.start(); b.start(); b.join(); a.join()
    print("survived")
    """
)


# python/bindings.cpp, space_time_astar's binding: `py::gil_scoped_release release; return space_time_astar(g, start,
# goal, table, limits);` releases the GIL while the search reads `table` (and `g`), which stay ordinary mutable Python
# objects. Another Python thread can then call table.forbid_vertex() (or reserve_path, forbid_from, or
# grid.set_passable) during the search: the std::unordered_set the search is probing rehashes under it, freeing the
# bucket array it is reading. Pure Python code crashes the interpreter: here SIGSEGV or SIGBUS (exit 139/138) within
# a second, every one of 9 runs on this Mac; the same steps run one after the other finish fine. (A fix either keeps
# the GIL for the search, or searches a copy of the table and grid.)
def test_a_search_survives_another_thread_adding_to_its_table():
    done = subprocess.run(
        [sys.executable, "-c", RACE, "2"],
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert done.returncode == 0, f"the child died with status {done.returncode} (negative: a signal): {done.stderr}"
    assert "survived" in done.stdout


# python/bindings.cpp, ReservationTable.move_free: it checks that both cells are inside the grid but not that they
# are 4-neighbours, which ReservationTable::move_free states as a precondition (reservation.hpp) and forbid_move
# enforces (ValueError, test_planners.py). For cells 3 and 4 of a 4 x 3 grid (consecutive ids across a row's end,
# not neighbours) the answer then depends on unrelated entries: True ("the move is free") on an empty table or with a
# move entry from cell 3 at another time, ValueError once any move entry from cell 3 exists at that time (the new
# byte-per-cell mark in reservation.hpp skips the direction check unless one does). Expected: ValueError, always, as
# forbid_move(3, 4, 0) gives.
def test_move_free_refuses_cells_that_arent_neighbours_whatever_the_table_holds():
    g = pf.Grid(4, 3)
    r = pf.ReservationTable(g)
    with pytest.raises(ValueError, match="aren't 4-neighbours"):
        r.forbid_move(3, 4, 0)
    with pytest.raises(ValueError, match="aren't 4-neighbours"):
        r.move_free(3, 4, 0)  # empty table: True at 4db3e63
    r.forbid_move(3, 2, 1)
    with pytest.raises(ValueError, match="aren't 4-neighbours"):
        r.move_free(3, 4, 0)  # an entry from 3 at another time: still True
    with pytest.raises(ValueError, match="aren't 4-neighbours"):
        r.move_free(0, 11, 0)  # opposite corners


# ---------------------------------------------------------------------------------------------------------------------
# ReviewD2bClean: checked and found right at 4db3e63
# ---------------------------------------------------------------------------------------------------------------------


# The same race without the second thread: insert first, then search. Shows the crash above comes from the
# concurrency, not from the table's size.
def test_review_d2b_clean_the_same_steps_one_after_the_other_finish():
    g = pf.Grid(40, 40)
    r = pf.ReservationTable(g)
    for c in range(g.cell_count):
        r.forbid_vertex(c, 1_000_000 + c)
    for n in range(20_000):
        r.forbid_vertex(n % (g.cell_count - 1), 2_000_000 + n)
    s = pf.space_time_astar(g, 0, g.cell_count - 1, r, max_expansions=1_000)
    assert s.status == pf.SearchStatus.FOUND or s.status == pf.SearchStatus.LIMIT


# Python's 0 means "no limit" for both searches and planners (a C++ SearchLimits of 0 would allow none), and tiny,
# huge, infinite and negative-zero time limits behave as PlannerLimits says.
def test_review_d2b_clean_python_limits():
    g = pf.Grid(6, 1)
    r = pf.ReservationTable(g)
    assert pf.space_time_astar(g, 0, 5, r, max_expansions=0).status == pf.SearchStatus.FOUND
    assert pf.space_time_astar(g, 0, 5, r, max_expansions=1).status == pf.SearchStatus.LIMIT
    assert pf.space_time_astar(g, 3, 3, r, max_expansions=1).status == pf.SearchStatus.FOUND
    assert pf.space_time_astar(g, 0, 5, r, time_limit=5e-324).status == pf.SearchStatus.LIMIT
    assert pf.space_time_astar(g, 0, 5, r, time_limit=-0.0).status == pf.SearchStatus.FOUND
    assert pf.space_time_astar(g, 0, 5, r, time_limit=1e308).status == pf.SearchStatus.FOUND
    inst = pf.Instance(g, [pf.Agent(0, 2), pf.Agent(5, 3)])
    assert pf.prioritized_planning(inst, max_expansions=0).solved
    tiny = pf.prioritized_planning(inst, time_limit=1e-300)
    assert tiny.status == pf.PlanStatus.LIMIT and tiny.failed_agent == 0 and tiny.expansions == 0
    with pytest.raises(ValueError):
        pf.prioritized_planning(inst, time_limit=float("nan"))


# move_free's "cell checks" in the binding: a `to` outside the grid is an IndexError, like a `from` outside it
# (deleting the check of `to` survives the existing suite: the answer is then True or a ValueError).
def test_review_d2b_clean_move_free_checks_both_cells():
    g = pf.Grid(4, 3)
    r = pf.ReservationTable(g)
    with pytest.raises(IndexError):
        r.move_free(0, 12, 0)
    with pytest.raises(IndexError):
        r.move_free(12, 8, 0)
    r.forbid_move(11, 7, 0)
    with pytest.raises(IndexError):
        r.move_free(11, 15, 0)
