"""The day-4 second-round review (commit c75fdf5, reviewing ed1cf44..c75fdf5): the bug tests failed on that commit;
test_clean_* passed and record what was checked."""

import importlib.util
import io
import json
import os
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import pytest

import pathfinder as pf

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[3]))


def load_script(name):
    spec = importlib.util.spec_from_file_location(f"{name}_under_review_d4b", ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sweep_survives_instances_whose_agents_all_start_on_their_goals(tmp_path):
    """e2dd982 added soc_ratio so that a both-solved instance with SOC 0 (every agent starts on its goal) doesn't
    divide by zero, but the next cells divide each SOC by the lower bound, which is 0 on exactly those instances.
    Input: a 4 x 4 open map whose scenario's two agents have start == goal, `--agents 2 --scenarios 1`. Wrong (at
    c75fdf5): ZeroDivisionError in the SOC / LB cells. Right: a row (ratio 1.0000, SOC / LB 1 or "-") and exit 0.
    Fix: the same guard for the lower bound (`soc / bound if bound else 1.0`)."""
    (tmp_path / "mapf-map").mkdir()
    (tmp_path / "scen-random").mkdir()
    (tmp_path / "mapf-map" / "open4.map").write_text("type octile\nheight 4\nwidth 4\nmap\n....\n....\n....\n....\n")
    lines = ["version 1"] + [f"0\topen4.map\t4\t4\t{x}\t{y}\t{x}\t{y}\t0" for x, y in ((0, 0), (3, 3))]
    (tmp_path / "scen-random" / "open4-random-1.scen").write_text("\n".join(lines) + "\n")
    sweep = load_script("sweep")
    out = io.StringIO()
    with redirect_stdout(out), redirect_stderr(io.StringIO()):
        code = sweep.main(["--data", str(tmp_path), "--maps", "open4", "--agents", "2", "--scenarios", "1",
                           "--time-limit", "5"])
    assert code == 0
    assert "| 2 | 1 / 1 | 1 / 1 |" in out.getvalue()


def record(planner, scenario, status, soc, seconds=0.01, nodes=10):
    return {"map": "m", "agents": 10, "scenario": scenario, "planner": planner, "status": status, "seconds": seconds,
            "sum_of_costs": soc if status == "SOLVED" else None, "lower_bound": 90, "expansions": 0,
            "nodes_generated": nodes}


def test_sweep_summary_counts_repeated_tries_consistently():
    """sweep_summary.py takes several JSONL files ("results/day4/sweep.jsonl [more.jsonl ...]"); when two hold the
    same map and agent count (a re-run, or a sweep split across files), its solved count keys tries by scenario (a
    dict: the second record of a scenario replaces the first) while the tried count is every record. Input: the same
    run twice, PP and CBS each solving scenarios 1 and 2. Wrong (at c75fdf5): "2 / 4" solved for both, though every
    try solved. Right: "4 / 4" (or refuse the duplicates). Fix: count solved records in a list, as the tried ones, and
    pair the both-solved set per file (or reject a repeated (map, agents, planner, scenario))."""
    summary = load_script("sweep_summary")
    run = [record(p, n, "SOLVED", 100 + n + (p == "pp")) for n in (1, 2) for p in ("pp", "cbs")]
    # The fix chosen is the docstring's second: refuse the duplicates (the reviewer's assertion was "4 / 4"; changed
    # with the fix, since which run to pair with which is the ambiguity). One run alone still gives 2 / 2.
    with pytest.raises(ValueError, match="repeated try"):
        summary.summarize(run + run)
    cells = [c.strip() for c in summary.summarize(run)[2].strip("|").split("|")]
    assert cells[2] == "2 / 2" and cells[3] == "2 / 2"


def test_clean_sweep_summary_on_one_file_matches_a_recomputation():
    """One record per try (what one sweep.py run writes): solved / tried, CBS's median time and nodes over the
    solved tries only, the both-solved mean and max of PP over CBS with n, and "-" for a planner never tried."""
    summary = load_script("sweep_summary")
    records = [record("pp", 1, "SOLVED", 110), record("cbs", 1, "SOLVED", 100, 0.004, 30),
               record("pp", 2, "SOLVED", 105), record("cbs", 2, "LIMIT", None, 5.0, 999_999),
               record("pp", 3, "NO_PATH", None), record("cbs", 3, "SOLVED", 100, 0.002, 10),
               record("pp", 4, "SOLVED", 120), record("cbs", 4, "SOLVED", 100, 1.5, 50_000)]
    records += [dict(record("pp", n, "SOLVED", 200), agents=20) for n in (1, 2)]
    rows = summary.summarize(records)
    cells = [c.strip() for c in rows[2].strip("|").split("|")]
    assert cells == ["m", "10", "3 / 4", "3 / 4", "4.00 ms", "30", "+15.0%, +20.0% (2)"]
    cells = [c.strip() for c in rows[3].strip("|").split("|")]
    assert cells == ["m", "20", "2 / 2", "-", "-", "-", "-"]


def test_clean_following_moves_by_hand():
    """delays.following_moves against hand counts: the corridor follower (robot 0 enters the cell robot 1 leaves at
    each of 3 steps): 3; a 2 x 2 rotation step: 4; entering a cell left one step earlier: 0; a plan with one path
    shorter than the other (padding on the goal): 0."""
    delays = load_script("delays")
    assert delays.following_moves([[0, 1, 2, 3], [1, 2, 3, 4]]) == 3
    assert delays.following_moves([[0, 1], [1, 3], [3, 2], [2, 0]]) == 4
    assert delays.following_moves([[0, 0, 1, 2], [1, 2, 3, 4]]) == 0
    assert delays.following_moves([[5], [0, 1, 2]]) == 0


def test_clean_delays_single_delay_rows_against_a_recomputation():
    """delays.py --single on generated-warehouse, 10 agents, scenarios 1-3, d = 1 and 3, recomputed here from the
    simulator: instances kept only when both planners solve them, robots with planned cost 0 skipped, the delay at
    cost // 2; runs, runs with a timetable collision, ordered extra SOC per run, the part on other robots, other
    robots later. Both planners' rows match."""
    from pathfinder.simulator import Simulator, delay_robot

    delays = load_script("delays")
    out = io.StringIO()
    with redirect_stdout(out):
        assert delays.main(["--map", "generated-warehouse", "--agents", "10", "--scenarios", "1-3",
                            "--probabilities", "0.1", "--seeds", "1", "--time-limit", "5", "--single", "1", "3"]) == 0
    grid = pf.warehouse_map(pf.WarehouseLayout())
    kept = []
    for n in range(1, 4):
        instance = pf.make_instance(grid, pf.random_scenario(grid, 10, n, "warehouse.map"), 10)
        pp, cbs = pf.prioritized_planning(instance, time_limit=5), pf.cbs(instance, time_limit=5)
        if pp.solved and cbs.solved:
            kept.append((instance, {"pp": pp, "cbs": cbs}))
    for name in ("pp", "cbs"):
        for d in (1, 3):
            runs = collided = extra = others = later = 0
            for instance, results in kept:
                plan = results[name].plan
                planned = list(pf.validate(instance, plan).costs)
                for robot, cost in enumerate(planned):
                    if cost == 0:
                        continue
                    tt = Simulator(instance, plan, policy="timetable").run(delay_robot(robot, cost // 2, d))
                    od = Simulator(instance, plan).run(delay_robot(robot, cost // 2, d))
                    runs += 1
                    collided += bool(tt.collisions)
                    extra += od.sum_of_costs - sum(planned)
                    others += sum(c - p for i, (c, p) in enumerate(zip(od.costs, planned)) if i != robot)
                    later += sum(c > p for i, (c, p) in enumerate(zip(od.costs, planned)) if i != robot)
            want = (f"| {name} | {d} | {runs} | {collided / runs:.1%} | {extra / runs:.3f} | {others / runs:.3f} | "
                    f"{later / runs:.3f} |")
            assert want in out.getvalue().splitlines(), (want, out.getvalue())
