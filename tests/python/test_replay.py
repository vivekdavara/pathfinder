"""Replays (pathfinder.replay): what the recorder writes is the simulator's run, and check() refuses any document the
simulator wouldn't reproduce."""

import copy
import json
import os
import shutil
import subprocess

import pytest

import pathfinder as pf
from pathfinder.replay import (FORMAT, VERSION, VIEWER, ReplayError, check, collisions_at, counts_at, delayed_at,
                               dumps, html, load, loads, map_rows, planned_at, positions_at, record, robot_hue, save,
                               write_html)
from pathfinder.simulator import Simulator, delay_robot, random_delays


def corridor():
    # Robot 0 follows robot 1 one cell behind down a 5 x 1 corridor: no slack, so a delay of robot 1 under the
    # timetable puts robot 0 on top of it.
    g = pf.Grid(5, 1)
    return pf.Instance(g, [pf.Agent(0, 3), pf.Agent(1, 4)]), [[0, 1, 2, 3], [1, 2, 3, 4]]


def warehouse(agents=12, seed=3):  # CBS solves it in 45 nodes
    grid = pf.warehouse_map(pf.WarehouseLayout())
    instance = pf.make_instance(grid, pf.random_scenario(grid, agents, seed, "warehouse.map"), agents)
    result = pf.cbs(instance, max_nodes=50_000)
    assert result.solved
    return instance, result.plan


def test_a_replay_is_the_simulators_run():
    instance, plan = warehouse()
    for policy in ("timetable", "ordered"):
        replay = record(instance, plan, policy, random_delays(0.1, seed=4), planner="cbs", map_name="warehouse.map")
        sim = Simulator(instance, plan, policy)
        steps = []
        run = sim.run(random_delays(0.1, seed=4), on_step=steps.append)
        assert replay["format"] == FORMAT and replay["version"] == VERSION and replay["policy"] == policy
        assert replay["trajectories"] == sim.trajectories
        assert replay["delayed"] == [list(s.delayed) for s in steps]
        assert len(replay["delayed"]) == run.steps
        assert [(c["kind"], c["a"], c["b"], c["time"]) for c in replay["collisions"]] == [
            (c.kind, c.a, c.b, c.time) for c in run.collisions]
        s = replay["summary"]
        assert (s["completed"], s["sum_of_costs"], s["makespan"], s["delay_steps"], s["collisions"]) == (
            run.completed, run.sum_of_costs, run.makespan, run.delay_steps, len(run.collisions))
        assert replay["robots"] == [{"start": a.start, "goal": a.goal} for a in instance.agents]
        assert replay["plan"] == [list(p) for p in plan]


def test_the_map_is_stored_as_rows_and_rebuilt_exactly():
    grid = pf.warehouse_map(pf.WarehouseLayout())
    rows = map_rows(grid)
    assert len(rows) == grid.height and {len(r) for r in rows} == {grid.width}
    assert sum(r.count(".") for r in rows) == grid.passable_count
    for y, row in enumerate(rows):
        for x, c in enumerate(row):
            assert (c == ".") == grid.passable(x, y)


def test_the_timetable_replay_keeps_its_collisions_and_ordered_has_none():
    instance, plan = corridor()
    timetable = record(instance, plan, "timetable", delay_robot(1, at=0, steps=1))
    assert timetable["delayed"][0] == [1]
    assert timetable["collisions"] and timetable["summary"]["collisions"] == len(timetable["collisions"])
    assert timetable["collisions"][0] == {"kind": "vertex", "a": 0, "b": 1, "time": 1, "cell": 1, "to": None}
    ordered = record(instance, plan, "ordered", delay_robot(1, at=0, steps=1))
    assert ordered["collisions"] == [] and ordered["summary"]["completed"]
    assert ordered["trajectories"] == [[0, 0, 1, 2, 3], [1, 1, 2, 3, 4]]


def test_json_round_trip():
    instance, plan = warehouse()
    replay = record(instance, plan, "ordered", random_delays(0.05, seed=1), title="t </script> & <!--")
    text = dumps(replay)
    assert "\n" not in text
    assert loads(text) == replay


def test_save_and_load(tmp_path):
    instance, plan = corridor()
    replay = record(instance, plan, "timetable", delay_robot(1, at=1))
    save(replay, tmp_path / "r.json")
    assert load(tmp_path / "r.json") == replay


def test_a_stopped_run_records_as_not_completed():
    instance, plan = corridor()
    replay = record(instance, plan, "ordered", delay_robot(1, at=0, steps=100), max_steps=5)
    assert not replay["summary"]["completed"] and len(replay["delayed"]) == 5
    assert replay["summary"]["costs"] == [] and replay["summary"]["sum_of_costs"] == 0
    check(replay)


def test_a_run_with_nothing_to_do():
    g = pf.Grid(3, 1)
    replay = record(pf.Instance(g, [pf.Agent(0, 0)]), [[0]], "ordered")
    assert replay["delayed"] == [] and replay["trajectories"] == [[0]] and replay["summary"]["completed"]
    assert positions_at(replay, 0) == [(0.0, 0.0)] and positions_at(replay, 7.5) == [(0.0, 0.0)]


def test_positions_move_straight_between_cells():
    instance, plan = corridor()
    replay = record(instance, plan, "ordered")  # 0 -> 3 and 1 -> 4 along y = 0
    assert positions_at(replay, 0) == [(0.0, 0.0), (1.0, 0.0)]
    assert positions_at(replay, 1.25) == [(1.25, 0.0), (2.25, 0.0)]
    assert positions_at(replay, 3) == [(3.0, 0.0), (4.0, 0.0)]
    assert positions_at(replay, -2) == positions_at(replay, 0)
    assert positions_at(replay, 99) == positions_at(replay, 3)


def test_positions_use_the_map_width_for_rows():
    g = pf.Grid(3, 3)
    instance = pf.Instance(g, [pf.Agent(g.id(0, 0), g.id(0, 2))])
    replay = record(instance, [[g.id(0, 0), g.id(0, 1), g.id(0, 2)]], "ordered")
    assert positions_at(replay, 0.5) == [(0.0, 0.5)] and positions_at(replay, 2) == [(0.0, 2.0)]


# --- what check() refuses ---

@pytest.fixture(scope="module")
def good():
    instance, plan = warehouse()
    return record(instance, plan, "timetable", random_delays(0.1, seed=4), planner="cbs")


def edited(replay, change):
    r = copy.deepcopy(replay)
    change(r)
    return r


def refused(replay, change, match):
    with pytest.raises(ReplayError, match=match):
        check(edited(replay, change))


def test_the_fixture_has_delays_and_collisions(good):
    assert good["summary"]["delay_steps"] > 0 and good["collisions"]
    check(good)


def first_move(replay):
    # (robot, step) of the first move in the run.
    for t in range(len(replay["delayed"])):
        for i, path in enumerate(replay["trajectories"]):
            if path[t] != path[t + 1]:
                return i, t
    raise AssertionError("nothing moves")


def test_a_robot_jumping_is_refused(good):
    i, t = first_move(good)

    def jump(r):
        r["trajectories"][i][t + 1] = r["robots"][i]["goal"]
    refused(good, jump, rf"trajectories\[{i}\]\[{t + 1}\]")


def test_a_delayed_robot_that_moves_is_refused(good):
    i, t = first_move(good)

    def delay(r):
        r["delayed"][t] = sorted(set(r["delayed"][t]) | {i})
    refused(good, delay, "trajectories")


def test_a_left_out_delay_is_refused(good):
    t = next(t for t, d in enumerate(good["delayed"]) if d)
    refused(good, lambda r: r["delayed"][t].pop(), "trajectories|collisions|summary")


def test_a_left_out_collision_is_refused(good):
    refused(good, lambda r: r["collisions"].pop(), "collisions")


def test_a_changed_cost_is_refused(good):
    def cost(r):
        r["summary"]["sum_of_costs"] -= 1
    refused(good, cost, "summary.sum_of_costs")


@pytest.mark.parametrize("key, value", [("steps", 1), ("completed", False), ("makespan", 0), ("delay_steps", 0),
                                        ("collisions", 0), ("planned_sum_of_costs", 1), ("costs", [])])
def test_every_summary_field_is_checked(good, key, value):
    assert good["summary"][key] != value
    refused(good, lambda r: r["summary"].__setitem__(key, value), f"summary.{key}")


def test_a_float_where_an_integer_belongs_is_refused(good):
    def floats(r):
        r["trajectories"][0][0] = float(r["trajectories"][0][0])
    refused(good, floats, r"trajectories\[0\]\[0\]")

    def bool_version(r):
        r["version"] = True
    refused(good, bool_version, "version")

    def float_cost(r):
        r["summary"]["costs"][0] = float(r["summary"]["costs"][0])
    refused(good, float_cost, "summary.costs")


def test_a_wrong_policy_is_refused(good):
    def other(r):
        r["policy"] = "ordered"
    refused(good, other, "collisions|trajectories|summary")
    refused(good, lambda r: r.update(policy="teleport"), "policy")


def test_an_invalid_plan_is_refused(good):
    def conflict(r):
        r["plan"][1] = list(r["plan"][0])
        r["robots"][1] = dict(r["robots"][0])
    refused(good, conflict, "instance|plan")

    def blocked(r):
        r["map"]["rows"] = ["@" * r["map"]["width"]] * r["map"]["height"]
    refused(good, blocked, "instance")


def test_the_shape_is_checked(good):
    refused(good, lambda r: r.pop("summary"), "wrong keys")
    refused(good, lambda r: r.update(extra=1), "wrong keys")
    refused(good, lambda r: r.update(format="something-else"), "format")
    refused(good, lambda r: r.update(version=2), "version 2")
    refused(good, lambda r: r["map"].update(width=0), "map.width")
    refused(good, lambda r: r["map"]["rows"].pop(), "map.rows")
    refused(good, lambda r: r["map"]["rows"].__setitem__(0, "x" * r["map"]["width"]), r"map.rows\[0\]")
    refused(good, lambda r: r["plan"].pop(), "plan")
    refused(good, lambda r: r["trajectories"][0].pop(), r"trajectories\[0\]")
    refused(good, lambda r: r["robots"][0].update(start=10 ** 9), r"robots\[0\].start")
    refused(good, lambda r: r["delayed"].append([]), r"trajectories\[0\]")


def test_unsorted_delays_and_delays_after_the_end_are_refused():
    instance, plan = corridor()
    replay = record(instance, plan, "ordered", delay_robot(1, at=0, steps=1))

    def unsorted(r):
        r["delayed"][0] = [1, 0]
    refused(replay, unsorted, "increasing order")

    def twice(r):
        r["delayed"][0] = [1, 1]
    refused(replay, twice, "increasing order")

    def extra_step(r):
        r["delayed"].append([])
        for t in r["trajectories"]:
            t.append(t[-1])
        r["summary"]["steps"] += 1
    refused(replay, extra_step, "the run is over")


def test_a_delay_for_a_robot_that_has_finished_is_refused():
    # Robot 0 is done after one step; robot 1 needs two. The recorder never lists a finished robot as delayed.
    g = pf.Grid(5, 1)
    replay = record(pf.Instance(g, [pf.Agent(0, 1), pf.Agent(2, 4)]), [[0, 1], [2, 3, 4]], "ordered")
    assert replay["delayed"] == [[], []]

    def finished(r):
        r["delayed"][1] = [0]
    refused(replay, finished, "already finished")


def test_strict_json():
    instance, plan = corridor()
    text = dumps(record(instance, plan, "ordered"))
    with pytest.raises(ReplayError, match="not JSON"):
        loads(text[:-1])
    with pytest.raises(ReplayError, match="twice"):
        loads(text[:-1] + ',"title":"again"}')
    doc = json.loads(text)
    doc["title"] = "x"
    nan = json.dumps(doc).replace('"x"', "NaN")
    with pytest.raises(ReplayError, match="NaN"):
        loads(nan)
    with pytest.raises(ReplayError, match="must be an object"):
        loads("[]")


# --- the viewer page ---

def embedded(page):
    start = page.index('<script id="replays" type="application/json">') + len(
        '<script id="replays" type="application/json">')
    return json.loads(page[start:page.index("</script>", start)])


def test_the_page_embeds_the_replays_and_the_player(good):
    instance, plan = corridor()
    other = record(instance, plan, "ordered")
    page = html([good, other])
    assert embedded(page) == [good, other]
    player = (VIEWER / "player.js").read_text()
    assert player in page and "/*PLAYER*/" not in page and "/*REPLAYS*/" not in page
    assert page.count("</script>") == 2


def test_no_title_can_break_out_of_the_data_element():
    instance, plan = corridor()
    nasty = "</script><script>alert(1)</script><!-- /*PLAYER*/ /*REPLAYS*/ &  "
    replay = record(instance, plan, "ordered", title=nasty, map_name="</SCRIPT>")
    page = html([replay])
    assert page.lower().count("</script") == 2 and "<!--" not in page
    assert embedded(page)[0]["title"] == nasty and embedded(page)[0]["map"]["name"] == "</SCRIPT>"
    assert " " not in page


def test_the_page_refuses_bad_replays(good):
    with pytest.raises(ValueError, match="no replays"):
        html([])
    with pytest.raises(ReplayError):
        html([edited(good, lambda r: r["collisions"].pop())])


def test_write_html(tmp_path, good):
    write_html([good], tmp_path / "page.html")
    assert embedded((tmp_path / "page.html").read_text()) == [good]


def flat(points):
    return [v for p in points for v in p]


NODE = shutil.which("node")
needs_node = pytest.mark.skipif(NODE is None and not os.environ.get("PATHFINDER_REQUIRE_NODE"),
                                reason="node isn't installed (CI sets PATHFINDER_REQUIRE_NODE)")


@needs_node
def test_the_page_and_python_agree_on_every_moment(good, tmp_path):
    # player.js's positionsAt, plannedAt, delayedAt, collisionsAt, countsAt and robotHue against this module's, on a
    # real run with delays and a collision, at whole and fractional times, before the start and after the end.
    end = len(good["delayed"])
    times = [-1, -0.5, 0, 0.25, 0.5, 1, 2.75, end / 2, end - 0.5, end - 0.1, end, end + 3]
    times += [c["time"] + d for c in good["collisions"] for d in (-0.5, -0.01, 0, 0.49, 0.5)]
    (tmp_path / "replay.json").write_text(dumps(good))
    script = f"""
        const p = require({json.dumps(str(VIEWER / "player.js"))});
        const r = JSON.parse(require("fs").readFileSync({json.dumps(str(tmp_path / "replay.json"))}, "utf8"));
        const times = {json.dumps(times)};
        console.log(JSON.stringify({{
          moments: times.map((t) => [p.positionsAt(r, t), p.plannedAt(r, t), p.delayedAt(r, t), p.collisionsAt(r, t),
                                     p.countsAt(r, t)]),
          hues: r.robots.map((_, i) => p.robotHue(i)),
        }}));
    """
    out = json.loads(subprocess.run([NODE or "node", "-e", script], capture_output=True, text=True,
                                    check=True).stdout)
    for t, (positions, planned, delayed, collisions, counts) in zip(times, out["moments"]):
        assert flat(positions) == pytest.approx(flat(positions_at(good, t)), abs=1e-9), t
        assert flat(planned) == pytest.approx(flat(planned_at(good, t)), abs=1e-9), t
        assert delayed == delayed_at(good, t), t
        assert collisions == collisions_at(good, t), t
        assert counts == counts_at(good, t), t
    assert out["hues"] == pytest.approx([robot_hue(i) for i in range(len(good["robots"]))], abs=1e-9)
    assert any(collisions_at(good, t) for t in times) and any(delayed_at(good, t) for t in times)


def test_moments_of_the_corridor():
    instance, plan = corridor()
    r = record(instance, plan, "timetable", delay_robot(1, at=0, steps=1))
    assert r["trajectories"] == [[0, 1, 2, 3, 3], [1, 1, 2, 3, 4]]
    assert planned_at(r, 0.5) == [(0.5, 0.0), (1.5, 0.0)] and planned_at(r, 9) == [(3.0, 0.0), (4.0, 0.0)]
    assert delayed_at(r, 0.9) == [1] and delayed_at(r, 1) == [] and delayed_at(r, 4) == []
    assert [c["time"] for c in collisions_at(r, 1.49)] == [1] and [c["time"] for c in collisions_at(r, 1.5)] == [2]
    assert counts_at(r, 0) == {"delays": 1, "collisions": 0, "finished": 0}
    assert counts_at(r, 4) == {"delays": 1, "collisions": 3, "finished": 2}
    # A collision at the run's last moment shows there (the run stopped at 3 steps, robots together at time 3).
    stopped = record(instance, plan, "timetable", delay_robot(1, at=0, steps=1), max_steps=3)
    assert len(stopped["delayed"]) == 3 and [c["time"] for c in collisions_at(stopped, 3)] == [3]
    # A robot that starts on its goal and stays is done from time 0.
    g = pf.Grid(3, 1)
    still = record(pf.Instance(g, [pf.Agent(0, 0), pf.Agent(1, 2)]), [[0], [1, 2]], "ordered")
    assert counts_at(still, 0)["finished"] == 1 and counts_at(still, 1)["finished"] == 2
