"""Replays: one run of the simulator recorded as a JSON document, checked on load, for the viewer (DESIGN.md "The
replay viewer (day 5)").

    import pathfinder as pf
    from pathfinder.replay import record, save, load
    from pathfinder.simulator import random_delays

    result = pf.cbs(instance, max_nodes=10_000)
    replay = record(instance, result.plan, policy="ordered", delays=random_delays(0.05, seed=1), planner="cbs")
    save(replay, "run.json")
    load("run.json") == replay          # load() checks the file before returning it
    write_html([replay], "run.html")    # one self-contained page that plays it: open it in a browser

A replay holds the map, the robots' starts and goals, the plan, what the robots drove (one cell per robot per time
step), who was delayed at each step, the collisions the simulator recorded and the run's summary. The simulator is
deterministic given the plan and the delays, so :func:`check` checks a replay by running it again: it rebuilds the
instance, validates the plan, executes it under the policy with each step's recorded delays, and requires every
position, every delay, every collision and the summary to come out the same. A replay that passes is a run the
simulator really makes; a hand-edited one with a robot jumping a cell, a delayed robot moving, a collision left out
or a cost changed doesn't pass.

Cells are ids, as everywhere in the library: ``y * width + x``. The map is stored as rows of ``.`` (passable) and
``@`` (blocked).
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import asdict
from pathlib import Path
from typing import Any

from . import _core as pf
from .simulator import POLICIES, DelaySchedule, Simulator

FORMAT = "pathfinder-replay"
VERSION = 1
MAX_DIMENSION = 10_000  # pf.Grid's limit on a side


class ReplayError(ValueError):
    """A replay document that isn't one, or doesn't replay to what it says."""


def map_rows(grid: pf.Grid) -> list[str]:
    """The grid as rows of ``.`` (passable) and ``@`` (blocked), top row first."""
    return ["".join("." if grid.passable(x, y) else "@" for x in range(grid.width)) for y in range(grid.height)]


def record(instance: pf.Instance, plan: Sequence[Sequence[int]], policy: str = "ordered",
           delays: DelaySchedule | None = None, max_steps: int | None = None, *, planner: str | None = None,
           title: str = "", map_name: str = "") -> dict[str, Any]:
    """Executes ``plan`` with the simulator (as :meth:`Simulator.run` does, same arguments) and returns the run as a
    replay document. ``planner``, ``title`` and ``map_name`` are labels for the viewer."""
    sim = Simulator(instance, plan, policy)
    delayed: list[list[int]] = []
    run = sim.run(delays, max_steps, on_step=lambda step: delayed.append(list(step.delayed)))
    grid = instance.grid
    replay = {
        "format": FORMAT,
        "version": VERSION,
        "title": title,
        "map": {"name": map_name, "width": grid.width, "height": grid.height, "rows": map_rows(grid)},
        "robots": [{"start": a.start, "goal": a.goal} for a in instance.agents],
        "planner": planner,
        "policy": policy,
        "plan": [list(p) for p in plan],
        "trajectories": [list(t) for t in sim.trajectories],
        "delayed": delayed,
        "collisions": [asdict(c) for c in run.collisions],
        "summary": _summary(run),
    }
    check(replay)
    return replay


def _summary(run: Any) -> dict[str, Any]:
    return {
        "completed": run.completed,
        "steps": run.steps,
        "planned_sum_of_costs": run.planned_sum_of_costs,
        "costs": list(run.costs),
        "sum_of_costs": run.sum_of_costs,
        "makespan": run.makespan,
        "delay_steps": run.delay_steps,
        "collisions": len(run.collisions),
    }


# --- checking ---

def _fail(message: str) -> None:
    raise ReplayError(message)


def _int(value: Any, what: str, low: int = 0, high: int | None = None) -> int:
    # bool is an int in Python and 3.0 == 3; neither is a cell, a count or a robot here.
    if type(value) is not int:
        _fail(f"{what} must be an integer, not {value!r}")
    if value < low or (high is not None and value > high):
        _fail(f"{what} must be in {low}..{high if high is not None else 'inf'}, not {value}")
    return value


def _list(value: Any, what: str, length: int | None = None) -> list[Any]:
    if type(value) is not list:
        _fail(f"{what} must be a list")
    if length is not None and len(value) != length:
        _fail(f"{what} must have {length} entries, not {len(value)}")
    return value


def _object(value: Any, what: str, keys: Sequence[str]) -> dict[str, Any]:
    if type(value) is not dict:
        _fail(f"{what} must be an object")
    if set(value) != set(keys):
        missing, extra = sorted(set(keys) - set(value)), sorted(set(value) - set(keys))
        _fail(f"{what} has the wrong keys (missing {missing}, unexpected {extra})")
    return value


def _same(a: Any, b: Any) -> bool:
    """Equal, with the same JSON types all the way down (``[1.0] == [1]`` and ``True == 1`` in Python)."""
    if type(a) is not type(b):
        return False
    if type(a) is list:
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if type(a) is dict:
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    return bool(a == b)


def _str(value: Any, what: str) -> str:
    if type(value) is not str:
        _fail(f"{what} must be a string")
    return value


TOP_KEYS = ("format", "version", "title", "map", "robots", "planner", "policy", "plan", "trajectories", "delayed",
            "collisions", "summary")
SUMMARY_KEYS = ("completed", "steps", "planned_sum_of_costs", "costs", "sum_of_costs", "makespan", "delay_steps",
                "collisions")


def _grid(m: dict[str, Any]) -> pf.Grid:
    _object(m, "map", ("name", "width", "height", "rows"))
    _str(m["name"], "map.name")
    width = _int(m["width"], "map.width", 1, MAX_DIMENSION)
    height = _int(m["height"], "map.height", 1, MAX_DIMENSION)
    rows = _list(m["rows"], "map.rows", height)
    grid = pf.Grid(width, height)
    for y, row in enumerate(rows):
        _str(row, f"map.rows[{y}]")
        if len(row) != width or set(row) - {".", "@"}:
            _fail(f"map.rows[{y}] must be {width} characters, each . or @")
        for x, c in enumerate(row):
            if c == "@":
                grid.set_passable(x, y, False)
    return grid


def check(replay: Any) -> None:
    """Raises :class:`ReplayError` unless ``replay`` is a well-formed replay document that the simulator, run again
    with its recorded delays, reproduces exactly."""
    _object(replay, "the replay", TOP_KEYS)
    if replay["format"] != FORMAT:
        _fail(f"format must be {FORMAT!r}, not {replay['format']!r}")
    if _int(replay["version"], "version") != VERSION:
        _fail(f"version {replay['version']} isn't supported (this is version {VERSION})")
    _str(replay["title"], "title")
    if replay["planner"] is not None:
        _str(replay["planner"], "planner")
    if replay["policy"] not in POLICIES:
        _fail(f"policy must be one of {POLICIES}, not {replay['policy']!r}")
    grid = _grid(replay["map"])

    # The instance and the plan: the constructor and validate() enforce the model's rules.
    cells = grid.cell_count - 1
    robots = _list(replay["robots"], "robots")
    agents = []
    for i, r in enumerate(robots):
        _object(r, f"robots[{i}]", ("start", "goal"))
        agents.append(pf.Agent(_int(r["start"], f"robots[{i}].start", 0, cells),
                               _int(r["goal"], f"robots[{i}].goal", 0, cells)))
    try:
        instance = pf.Instance(grid, agents)
    except ValueError as e:
        raise ReplayError(f"the robots aren't a valid instance: {e}") from None
    plan = _list(replay["plan"], "plan", len(robots))
    for i, path in enumerate(plan):
        if not _list(path, f"plan[{i}]"):
            _fail(f"plan[{i}] is empty")
        for t, c in enumerate(path):
            _int(c, f"plan[{i}][{t}]", 0, cells)
    found = pf.validate(instance, plan)
    if not found.valid:
        _fail(f"the plan isn't valid: {pf.describe(found.problems[0], grid)}")

    # The run, again.
    delayed = _list(replay["delayed"], "delayed")
    trajectories = _list(replay["trajectories"], "trajectories", len(robots))
    for i, t in enumerate(trajectories):
        _list(t, f"trajectories[{i}]", len(delayed) + 1)
    summary = _object(replay["summary"], "summary", SUMMARY_KEYS)
    sim = Simulator(instance, plan, replay["policy"])
    for t, held in enumerate(delayed):
        _list(held, f"delayed[{t}]")
        for i in held:
            _int(i, f"delayed[{t}] entry", 0, len(robots) - 1)
        if held != sorted(set(held)):
            _fail(f"delayed[{t}] must be robots in increasing order, each once")
        if sim.done:
            _fail(f"the run is over after {t} steps, but delays are recorded for step {t}")
        step = sim.step(held)
        if list(step.delayed) != held:
            _fail(f"delayed[{t}] lists robots that had already finished")
    if summary["completed"] is not sim.done:
        _fail(f"summary.completed is {summary['completed']!r}, but the run {'is' if sim.done else 'is not'} over")
    for i, (recorded, driven) in enumerate(zip(trajectories, sim.trajectories)):
        if not _same(recorded, driven):
            t = next(t for t, (a, b) in enumerate(zip(recorded, driven)) if not _same(a, b))
            _fail(f"trajectories[{i}][{t}] is {recorded[t]!r}; the simulator drives robot {i} to {driven[t]} there")
    collisions = [asdict(c) for c in sim.collisions]
    if not _same(replay["collisions"], collisions):
        _fail(f"collisions don't match the run's {len(collisions)} collisions")
    expected = _summary(sim.execution())
    for key in SUMMARY_KEYS:
        if not _same(summary[key], expected[key]):
            _fail(f"summary.{key} is {summary[key]!r}, the run's is {expected[key]!r}")


# --- files ---

def _no_constants(name: str) -> None:
    raise ReplayError(f"{name} isn't JSON")


def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for k, v in pairs:
        if k in out:
            raise ReplayError(f"key {k!r} appears twice in one object")
        out[k] = v
    return out


def dumps(replay: dict[str, Any]) -> str:
    """The replay as compact JSON (one line, keys in the document's order)."""
    return json.dumps(replay, separators=(",", ":"), allow_nan=False)


def loads(text: str) -> dict[str, Any]:
    """A replay from JSON text, checked (:func:`check`). Strict JSON: no NaN or Infinity, no key twice in an
    object."""
    try:
        replay = json.loads(text, parse_constant=_no_constants, object_pairs_hook=_no_duplicates)
    except json.JSONDecodeError as e:
        raise ReplayError(f"not JSON: {e}") from None
    check(replay)
    return replay


def save(replay: dict[str, Any], path: str | Path) -> None:
    """Writes the replay as JSON, after checking it."""
    check(replay)
    Path(path).write_text(dumps(replay) + "\n", encoding="utf-8")


def load(path: str | Path) -> dict[str, Any]:
    return loads(Path(path).read_text(encoding="utf-8"))


# --- the viewer page ---

VIEWER = Path(__file__).resolve().parent / "viewer"


def _script_safe(text: str) -> str:
    """JSON text that can sit inside a <script> element: ``<``, ``>`` and ``&`` only appear inside JSON strings, where
    their \\u escapes mean the same thing, and without them no ``</script>`` or ``<!--`` in a title can end or change the
    element. (U+2028 and U+2029 are escaped too, for older JavaScript parsers.)"""
    for c, escape in (("<", "\\u003c"), (">", "\\u003e"), ("&", "\\u0026"), ("\u2028", "\\u2028"),
                      ("\u2029", "\\u2029")):
        text = text.replace(c, escape)
    return text


def html(replays: Sequence[dict[str, Any]]) -> str:
    """One self-contained page (viewer/viewer.html with viewer/player.js inlined) that plays the replays side by side
    on one timeline. Each replay is checked first."""
    if not replays:
        raise ValueError("no replays to show")
    for replay in replays:
        check(replay)
    template = (VIEWER / "viewer.html").read_text(encoding="utf-8")
    player = (VIEWER / "player.js").read_text(encoding="utf-8")
    if "</script" in player.lower():
        raise RuntimeError("player.js can't be inlined: it contains </script")
    # Split the template first, so a placeholder's text inside a replay (in a title, say) is never replaced.
    before, rest = template.split("/*REPLAYS*/")
    middle, after = rest.split("/*PLAYER*/")
    data = _script_safe(json.dumps(list(replays), separators=(",", ":"), allow_nan=False))
    return before + data + middle + player + after


def write_html(replays: Sequence[dict[str, Any]], path: str | Path) -> None:
    Path(path).write_text(html(replays), encoding="utf-8")


def positions_at(replay: dict[str, Any], time: float) -> list[tuple[float, float]]:
    """Every robot's (x, y) at a moment of the run, cell centres at whole times and straight between them in a step
    (the viewer's animation; the GIF renderer uses this too). ``time`` is clamped to the run."""
    width = replay["map"]["width"]
    steps = len(replay["delayed"])
    time = min(max(time, 0.0), float(steps))
    t = min(math.floor(time), max(steps - 1, 0))
    f = time - t
    out = []
    for path in replay["trajectories"]:
        a = path[t]
        b = path[t + 1] if t + 1 < len(path) else a
        ax, ay = a % width, a // width
        bx, by = b % width, b // width
        out.append((ax + (bx - ax) * f, ay + (by - ay) * f))
    return out
