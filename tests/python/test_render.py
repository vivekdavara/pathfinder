"""The GIF renderer (pathfinder.render): frames on one timeline, robots, ghosts, delays and collisions in their
palette colours where the replay puts them, and a GIF that loops with the right delays."""

import pytest

import pathfinder as pf
from pathfinder.replay import ReplayError, collisions_at, delayed_at, positions_at, record
from pathfinder.simulator import random_delays

render = pytest.importorskip("pathfinder.render")  # Pillow is in requirements-dev.txt, so CI always has it
from PIL import Image  # noqa: E402

CELL = 14


@pytest.fixture(scope="module")
def runs():
    grid = pf.warehouse_map(pf.WarehouseLayout())
    instance = pf.make_instance(grid, pf.random_scenario(grid, 12, 3, "warehouse.map"), 12)
    plan = pf.cbs(instance, max_nodes=50_000).plan
    timetable = record(instance, plan, "timetable", random_delays(0.1, seed=4), planner="cbs")
    ordered = record(instance, plan, "ordered", random_delays(0.1, seed=4), planner="cbs")
    assert timetable["collisions"] and not ordered["collisions"]
    return timetable, ordered


def panel(img, replays, k):
    """Replay k's map area of a frame."""
    x = render.GAP + sum(r["map"]["width"] * CELL + render.GAP for r in replays[:k])
    r = replays[k]
    return img.crop((x, render.HEADER, x + r["map"]["width"] * CELL, render.HEADER + r["map"]["height"] * CELL))


def pixel_at(img, p):
    return img.getpixel((round((p[0] + 0.5) * CELL - 0.5), round((p[1] + 0.5) * CELL - 0.5)))


def test_frames_cover_the_longest_run(runs):
    frames = render.frames(list(runs), cell=CELL, substeps=3)
    end = max(len(r["delayed"]) for r in runs)
    assert len(frames) == end * 3 + 1
    w = render.GAP + sum(r["map"]["width"] * CELL + render.GAP for r in runs)
    h = render.HEADER + max(r["map"]["height"] for r in runs) * CELL + render.GAP
    assert {f.size for f in frames} == {(w, h)} and {f.mode for f in frames} == {"P"}


def test_robots_are_drawn_where_the_replay_puts_them(runs):
    replays = list(runs)
    frames = render.frames(replays, cell=CELL, substeps=2)
    for k, replay in enumerate(replays):
        for f in (0, 7, 20, len(frames) - 1):
            time = f / 2
            img = panel(frames[f], replays, k)
            positions = positions_at(replay, time)
            for i, p in enumerate(positions):
                if any(j != i and abs(q[0] - p[0]) + abs(q[1] - p[1]) < 1 for j, q in enumerate(positions)):
                    continue  # another robot overlaps this one
                assert pixel_at(img, p) == render.robot_index(i), (k, f, i)


def test_delayed_robots_get_an_amber_ring(runs):
    replay = runs[1]
    t = next(t for t, d in enumerate(replay["delayed"]) if d)
    img = render.draw_map(replay, t + 0.5, CELL)
    for i in delayed_at(replay, t + 0.5):
        x, y = positions_at(replay, t + 0.5)[i]
        ring = img.crop((round(x * CELL), round(y * CELL), round((x + 1) * CELL), round((y + 1) * CELL)))
        assert render.DELAY in [c for _, c in ring.getcolors()]


def test_collisions_are_red_only_in_the_run_that_has_them(runs):
    timetable, ordered = runs
    c = timetable["collisions"][0]
    assert collisions_at(timetable, c["time"])
    colours = lambda img: {colour for _, colour in img.getcolors()}  # noqa: E731
    assert render.COLLISION in colours(render.draw_map(timetable, c["time"], CELL))
    assert render.COLLISION not in colours(render.draw_map(timetable, 0, CELL))
    for k in range(len(ordered["delayed"]) * 2 + 1):
        assert render.COLLISION not in colours(render.draw_map(ordered, k / 2, CELL))


def test_a_swap_is_a_red_bar_across_its_edge():
    # A real timetable swap: prioritized planning's plan for 8 agents on the floor (scenario seed 2), delays seed 1:
    # robots 3 and 7 swap cells 12 and 11 between times 6 and 7.
    grid = pf.warehouse_map(pf.WarehouseLayout())
    instance = pf.make_instance(grid, pf.random_scenario(grid, 8, 2, "warehouse.map"), 8)
    replay = record(instance, pf.prioritized_planning(instance).plan, "timetable", random_delays(0.1, seed=1))
    swap = next(c for c in replay["collisions"] if c["kind"] == "swap")
    assert (swap["a"], swap["b"], swap["time"], swap["cell"], swap["to"]) == (3, 7, 6, 12, 11)
    w = replay["map"]["width"]
    (ax, ay), (bx, by) = (swap["cell"] % w, swap["cell"] // w), (swap["to"] % w, swap["to"] // w)
    assert ay == by  # a horizontal edge: the bar runs along the row between the two centres
    img = render.draw_map(replay, swap["time"] + 0.5, CELL)
    rows = range(ay * CELL + CELL // 4, ay * CELL + CELL - CELL // 4)  # the bar's thickness, around the row's centre
    xs = range(min(ax, bx) * CELL + CELL // 2, max(ax, bx) * CELL + CELL // 2)  # from one centre to the other
    red = [x for x in xs if any(img.getpixel((x, y)) == render.COLLISION for y in rows)]
    assert len(red) == len(xs)
    # A step later the swap is over (the bar is gone unless another collision is shown on those cells).
    assert not any(c["kind"] == "swap" for c in collisions_at(replay, swap["time"] + 1))


def test_ghosts_show_the_plan_when_a_robot_is_behind(runs):
    replay = runs[1]
    t = next(t for t, d in enumerate(replay["delayed"]) if d) + 1
    i = replay["delayed"][t - 1][0]
    img = render.draw_map(replay, t, CELL)
    assert render.robot_index(i, ghost=True) in {c for _, c in img.getcolors()}


def test_the_gif_loops_with_the_frame_delays(runs, tmp_path):
    path = tmp_path / "r.gif"
    n = render.save_gif(list(runs), path, cell=CELL, substeps=4, steps_per_second=5, hold=1.5)
    with Image.open(path) as gif:
        assert gif.n_frames == n == max(len(r["delayed"]) for r in runs) * 4 + 1
        assert gif.info["loop"] == 0
        durations = []
        for k in range(gif.n_frames):
            gif.seek(k)
            durations.append(gif.info["duration"])
    assert set(durations[:-1]) == {50} and durations[-1] == 50 + 1500


def test_the_gif_holds_exactly_the_frames_drawn(runs, tmp_path):
    # Saving optimizes the palette; every frame read back must be the frame drawn, pixel for pixel.
    path = tmp_path / "r.gif"
    drawn = render.frames(list(runs), cell=CELL, substeps=2)
    render.save_gif(list(runs), path, cell=CELL, substeps=2)
    with Image.open(path) as gif:
        assert gif.n_frames == len(drawn)
        for k, frame in enumerate(drawn):
            gif.seek(k)
            assert gif.convert("RGB").tobytes() == frame.convert("RGB").tobytes(), k


def test_the_renderer_refuses_what_it_cant_draw(runs, tmp_path):
    with pytest.raises(ValueError, match="no replays"):
        render.frames([])
    with pytest.raises(ValueError, match="cell"):
        render.frames(list(runs), cell=3)
    with pytest.raises(ValueError, match="substeps"):
        render.frames(list(runs), substeps=0)
    bad = dict(runs[0], collisions=[])
    with pytest.raises(ReplayError):
        render.frames([bad])
    with pytest.raises(ValueError, match="steps_per_second"):
        render.save_gif(list(runs), tmp_path / "x.gif", steps_per_second=0)


def test_robot_colours_wrap_after_the_palette_is_full():
    assert render.robot_index(render.COLOURS) == render.robot_index(0)
    assert render.robot_index(render.COLOURS - 1, ghost=True) == 255
    assert len(render.palette()) == 768
