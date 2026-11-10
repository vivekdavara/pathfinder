"""The experiment scripts (scripts/sweep.py, scripts/delays.py, scripts/replay.py): their arithmetic on cases worked
out by hand, and replay.py run in-process. CTest also runs all three end to end on the generated floor (python.sweep,
python.delays, python.replay)."""

import importlib.util
import json
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


# --- scripts/replay.py ---

def test_replay_writes_the_page_the_runs_and_the_gif(tmp_path, capsys):
    replay_script = load("replay")
    from pathfinder.replay import load as load_replay
    from pathfinder.replay import loads

    args = ["--agents", "12", "--scenario", "3", "--delay-probability", "0.1", "--seed", "4",
            "--html", str(tmp_path / "r.html"), "--json", str(tmp_path / "r"), "--gif-substeps", "1"]
    try:
        import PIL  # noqa: F401
        args += ["--gif", str(tmp_path / "r.gif")]
    except ImportError:
        pass
    assert replay_script.main(args) == 0
    timetable, ordered = load_replay(tmp_path / "r-timetable.json"), load_replay(tmp_path / "r-ordered.json")
    assert (timetable["policy"], ordered["policy"]) == ("timetable", "ordered")
    assert timetable["plan"] == ordered["plan"] and timetable["delayed"][0] == ordered["delayed"][0]
    page = (tmp_path / "r.html").read_text()
    start = page.index('type="application/json">') + len('type="application/json">')
    embedded = json.loads(page[start:page.index("</script>", start)])
    assert [loads(json.dumps(r)) for r in embedded] == [timetable, ordered]
    out = capsys.readouterr().out
    assert "timetable:" in out and "ordered  :" in out and "the check alone" in out


def test_replay_on_a_movingai_map(tmp_path):
    # The MovingAI branch without downloads: the small hand-written map and scenario of tests/data, laid out as
    # make maps lays out the benchmark set.
    replay_script = load("replay")
    (tmp_path / "mapf-map").mkdir()
    (tmp_path / "scen-random").mkdir()
    (tmp_path / "mapf-map" / "small.map").write_text((ROOT / "tests" / "data" / "small.map").read_text())
    (tmp_path / "scen-random" / "small-random-1.scen").write_text((ROOT / "tests" / "data" / "small.scen").read_text())
    out = tmp_path / "s"
    assert replay_script.main(["--map", "small", "--agents", "2", "--data", str(tmp_path), "--json", str(out),
                               "--policies", "ordered", "--delay-robot", "0", "--at", "1", "--steps", "2"]) == 0
    from pathfinder.replay import load as load_replay
    r = load_replay(tmp_path / "s-ordered.json")
    assert r["map"]["name"] == "small, random scenario 1" and r["delayed"][1:3] == [[0], [0]]
    # Too many agents for the scenario, a missing map: exit 2 with a message, no traceback.
    assert replay_script.main(["--map", "small", "--agents", "999", "--data", str(tmp_path), "--json", str(out)]) == 2
    assert replay_script.main(["--map", "nowhere", "--data", str(tmp_path), "--json", str(out)]) == 2


def test_replay_exits_1_when_the_planner_fails(tmp_path, capsys):
    replay_script = load("replay")
    assert replay_script.main(["--agents", "16", "--scenario", "1", "--time-limit", "1e-9",
                               "--html", str(tmp_path / "r.html")]) == 1
    assert "didn't solve" in capsys.readouterr().err and not (tmp_path / "r.html").exists()


@pytest.mark.parametrize("args", [
    [],                                                       # nothing to write
    ["--at", "3"],                                            # --at without --delay-robot
    ["--steps", "2"],
    ["--delay-robot", "99"],                                  # there are 12 robots
    ["--delay-robot", "1", "--delay-probability", "0.1"],    # one kind of delays or the other
    ["--delay-probability", "nan"],
    ["--delay-probability", "1.5"],
    ["--gif-cell", "3"],
    ["--gif-substeps", "0"],
    ["--time-limit", "0"],
    ["--time-limit", "inf"],
    ["--policies", "ordered", "ordered"],
    ["--agents", "0"],
])
def test_replay_usage_errors(args, tmp_path):
    replay_script = load("replay")
    out = [] if not args else ["--html", str(tmp_path / "r.html")]
    with pytest.raises(SystemExit) as e:
        replay_script.main(["--agents", "12"] + args + out)
    assert e.value.code == 2
