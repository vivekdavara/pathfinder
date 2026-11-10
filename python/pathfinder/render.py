"""Animated GIFs of replays, side by side on one timeline, drawn the way the viewer page draws them (DESIGN.md "The
replay viewer (day 5)"). Needs Pillow (in requirements-dev.txt); nothing else in the package does.

    from pathfinder.render import save_gif
    save_gif([timetable_replay, ordered_replay], "replay.gif", cell=14, substeps=4)

Every frame is drawn in palette mode with one fixed palette (the page's colours, two shades per robot), so the
colours are exact and the same in every frame. Saving with Pillow's ``optimize`` drops the palette entries no frame
uses, so each pixel takes fewer bits: the README's GIF went from 754.7 KiB to 234.3 KiB with every frame's pixels
unchanged (a test reads the frames back). What is drawn at a moment comes from
pathfinder.replay's positions_at, planned_at, delayed_at, collisions_at and counts_at, which a test checks against
the page's own functions.
"""

from __future__ import annotations

import colorsys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageFont

from .replay import check, collisions_at, counts_at, delayed_at, planned_at, positions_at, robot_hue

# Palette indices of the fixed colours (the page's light theme), then two per robot colour from ROBOTS on.
PAGE, FLOOR, WALL, TEXT, MUTED, DELAY, COLLISION, ON_ROBOT = range(8)
FIXED = [(0xF6, 0xF6, 0xF3), (0xEC, 0xEB, 0xE4), (0x3A, 0x3A, 0x36), (0x1D, 0x1D, 0x1B), (0x66, 0x66, 0x5F),
         (0xE0, 0xA1, 0x00), (0xD1, 0x24, 0x2F), (0xFF, 0xFF, 0xFF)]
ROBOTS = len(FIXED)
COLOURS = (256 - ROBOTS) // 2  # robot colours in the palette; robot i uses colour i % COLOURS

HEADER = 40  # pixels above each map for its label and counts
GAP = 12  # pixels between maps, and around them


def _hsl(hue: float, lightness: float) -> tuple[int, int, int]:
    r, g, b = colorsys.hls_to_rgb(hue / 360, lightness, 0.7)
    return round(r * 255), round(g * 255), round(b * 255)


def palette() -> list[int]:
    colours = list(FIXED)
    for i in range(COLOURS):
        colours += [_hsl(robot_hue(i), 0.5), _hsl(robot_hue(i), 0.78)]  # the robot, its ghost
    colours += [(0, 0, 0)] * (256 - len(colours))
    return [v for c in colours for v in c]


def robot_index(i: int, ghost: bool = False) -> int:
    return ROBOTS + 2 * (i % COLOURS) + (1 if ghost else 0)


def _font(size: int) -> ImageFont.ImageFont | ImageFont.FreeTypeFont:
    return ImageFont.load_default(size=size)


def _count(n: int, what: str) -> str:
    return f"{n} {what}{'' if n == 1 else 's'}"


def _label(replay: dict[str, Any]) -> str:
    parts = [replay["policy"]]
    if replay["planner"]:
        parts.insert(0, replay["planner"].upper())
    return " · ".join(parts)


def _base(replay: dict[str, Any], cell: int) -> Image.Image:
    """The map and the goals (they don't change)."""
    m = replay["map"]
    img = Image.new("P", (m["width"] * cell, m["height"] * cell), FLOOR)
    img.putpalette(palette())
    d = ImageDraw.Draw(img)
    for y, row in enumerate(m["rows"]):
        for x, c in enumerate(row):
            if c == "@":
                d.rectangle([x * cell, y * cell, (x + 1) * cell - 1, (y + 1) * cell - 1], fill=WALL)
    inset = max(1, round(cell * 0.18))
    for i, r in enumerate(replay["robots"]):
        x, y = r["goal"] % m["width"], r["goal"] // m["width"]
        d.rectangle([x * cell + inset, y * cell + inset, (x + 1) * cell - 1 - inset, (y + 1) * cell - 1 - inset],
                    outline=robot_index(i), width=max(1, cell // 10))
    return img


def _circle(d: ImageDraw.ImageDraw, centre: tuple[float, float], radius: float, **kw: Any) -> None:
    cx, cy = centre
    d.ellipse([cx - radius, cy - radius, cx + radius, cy + radius], **kw)


def draw_map(replay: dict[str, Any], time: float, cell: int, base: Image.Image | None = None) -> Image.Image:
    """One replay's map at a moment: the floor, walls and goals, the plan's ghosts, the robots (delayed ones outlined
    in amber), and its collisions (red ring or bar)."""
    img = (base if base is not None else _base(replay, cell)).copy()
    d = ImageDraw.Draw(img)

    def centre(p: tuple[float, float]) -> tuple[float, float]:
        return (p[0] + 0.5) * cell - 0.5, (p[1] + 0.5) * cell - 0.5

    robots = positions_at(replay, time)
    for i, (g, r) in enumerate(zip(planned_at(replay, time), robots)):
        if abs(g[0] - r[0]) + abs(g[1] - r[1]) > 1e-9:
            _circle(d, centre(g), cell * 0.32, outline=robot_index(i, ghost=True), width=max(1, cell // 12))
    delayed = set(delayed_at(replay, time))
    for i, r in enumerate(robots):
        if i in delayed:
            _circle(d, centre(r), cell * 0.42, fill=DELAY)
            _circle(d, centre(r), cell * 0.30, fill=robot_index(i))
        else:
            _circle(d, centre(r), cell * 0.38, fill=robot_index(i))
    width = replay["map"]["width"]
    for c in collisions_at(replay, time):
        a = centre((c["cell"] % width, c["cell"] // width))
        if c["kind"] == "vertex":
            _circle(d, a, cell * 0.58, outline=COLLISION, width=max(2, cell // 6))
        else:
            b = centre((c["to"] % width, c["to"] // width))
            d.line([a, b], fill=COLLISION, width=max(2, cell // 6))
    return img


def frames(replays: Sequence[dict[str, Any]], cell: int = 14, substeps: int = 4) -> list[Image.Image]:
    """The animation's frames: ``substeps`` per time step, from 0 to the longest run's end, each replay's map side by
    side under its label and its counts at that moment."""
    if not replays:
        raise ValueError("no replays to draw")
    if cell < 4 or substeps < 1:
        raise ValueError("cell must be 4 pixels or more and substeps 1 or more")
    for replay in replays:
        check(replay)
    end = max(len(r["delayed"]) for r in replays)
    sizes = [(r["map"]["width"] * cell, r["map"]["height"] * cell) for r in replays]
    width = GAP + sum(w + GAP for w, _ in sizes)
    height = HEADER + max(h for _, h in sizes) + GAP
    bases = [_base(r, cell) for r in replays]
    title, small = _font(13), _font(12)
    out = []
    for k in range(end * substeps + 1):
        time = k / substeps
        img = Image.new("P", (width, height), PAGE)
        img.putpalette(palette())
        d = ImageDraw.Draw(img)
        x = GAP
        for replay, base, (w, _) in zip(replays, bases, sizes):
            c = counts_at(replay, time)
            d.text((x, 6), f"{_label(replay)}   t = {time:.1f}", fill=TEXT, font=title)
            d.text((x, 23), f"{_count(c['delays'], 'delay step')} · {_count(c['collisions'], 'collision')} · "
                            f"{c['finished']}/{len(replay['robots'])} done",
                   fill=COLLISION if c["collisions"] else MUTED, font=small)
            img.paste(draw_map(replay, time, cell, base), (x, HEADER))
            x += w + GAP
        out.append(img)
    return out


def save_gif(replays: Sequence[dict[str, Any]], path: str | Path, cell: int = 14, substeps: int = 4,
             steps_per_second: float = 4.0, hold: float = 2.0) -> int:
    """Writes the frames as a looping GIF: ``steps_per_second`` time steps a second, the last frame held ``hold``
    seconds. Returns the number of frames drawn. (Pillow merges a frame identical to the one before into it, adding
    its time, so the file can hold fewer; the clock in each frame's header changes every frame unless more than ten
    frames share a tenth of a step.)"""
    if not steps_per_second > 0 or not hold >= 0:
        raise ValueError("steps_per_second must be positive and hold 0 or more")
    images = frames(replays, cell, substeps)
    # GIF frame delays are in hundredths of a second; Pillow rounds milliseconds down to them.
    each = max(20, round(1000 / (steps_per_second * substeps) / 10) * 10)
    durations = [each] * (len(images) - 1) + [each + round(hold * 1000 / 10) * 10]
    images[0].save(path, save_all=True, append_images=images[1:], duration=durations, loop=0, optimize=True,
                   disposal=1)
    return len(images)
