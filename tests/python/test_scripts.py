"""The experiment scripts' own arithmetic (scripts/sweep.py, scripts/delays.py), on cases worked out by hand. CTest
also runs both scripts end to end on the generated floor (python.sweep, python.delays)."""

import importlib.util
import os
from pathlib import Path

import pytest

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[2]))


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_following_moves():
    delays = load("delays")
    # Robot 0 follows robot 1 down a corridor: it moves into robot 1's cell as robot 1 moves out, three times.
    assert delays.following_moves([[0, 1, 2, 3], [1, 2, 3, 4]]) == 3
    # The same hand-off a step apart isn't following.
    assert delays.following_moves([[0, 0, 1], [1, 2, 2]]) == 0
    # A rotation of four on a 2 x 2 grid: every robot follows another.
    assert delays.following_moves([[0, 1], [1, 3], [3, 2], [2, 0]]) == 4
    # Paths of different lengths: the short one sits on its last cell, so nothing moves into it.
    assert delays.following_moves([[5], [4, 3, 2]]) == 0


@pytest.mark.parametrize("module", ["sweep", "delays"])
def test_scenario_numbers(module):
    numbers = load(module).scenario_numbers
    assert list(numbers("1-5")) == [1, 2, 3, 4, 5]
    assert list(numbers("7")) == [7]
    assert list(numbers("3-2")) == []


def test_sweep_summary():
    summary = load("sweep_summary")

    def rec(planner, scenario, status, soc=None, seconds=0.001, nodes=0, agents=10):
        return {"map": "m", "agents": agents, "scenario": scenario, "planner": planner, "status": status,
                "seconds": seconds, "sum_of_costs": soc, "lower_bound": 90, "expansions": 0, "nodes_generated": nodes}

    records = [
        rec("pp", 1, "SOLVED", 110), rec("cbs", 1, "SOLVED", 100, 0.002, 5),
        rec("pp", 2, "NO_PATH"), rec("cbs", 2, "SOLVED", 100, 0.004, 7),
        rec("pp", 3, "SOLVED", 105), rec("cbs", 3, "LIMIT", None, 5.0, 300_000),
        rec("pp", 1, "SOLVED", 200, agents=20), rec("cbs", 1, "LIMIT", None, 5.0, 1, agents=20),
        rec("pp", 1, "SOLVED", 300, agents=30),
    ]
    lines = summary.summarize(records)
    assert lines[2] == "| m | 10 | 2 / 3 | 2 / 3 | 3.00 ms | 6 | +10.0%, +10.0% (1) |"
    assert lines[3] == "| m | 20 | 1 / 1 | 0 / 1 | - | - | - |"
    assert lines[4] == "| m | 30 | 1 / 1 | - | - | - | - |"
    assert summary.duration(1.5) == "1.50 s"


def test_sweep_refuses_no_time_limit_and_sorts_the_agent_counts(capsys):
    sweep = load("sweep")
    with pytest.raises(SystemExit) as stop:
        sweep.main(["--maps", "generated-warehouse", "--agents", "4", "--scenarios", "1", "--time-limit", "0"])
    assert stop.value.code == 2 and "more than 0" in capsys.readouterr().err
    for limit in ("inf", "1e9", "nan"):  # also "no limit" to the planners (inf: the review's second round)
        with pytest.raises(SystemExit):
            sweep.main(["--maps", "generated-warehouse", "--agents", "4", "--scenarios", "1", "--time-limit", limit])
    capsys.readouterr()
    # Given 8 before 4, the rows still come 4 then 8 (the give-up rule relies on it).
    args = ["--maps", "generated-warehouse", "--agents", "8", "4", "--scenarios", "1", "--time-limit", "2"]
    assert sweep.main(args) == 0
    rows = [line.split("|")[1].strip() for line in capsys.readouterr().out.splitlines() if line[:3] in ("| 4", "| 8")]
    assert rows == ["4", "8"]
    assert sweep.soc_ratio(0, 0) == 1.0 and sweep.soc_ratio(11, 10) == 1.1
