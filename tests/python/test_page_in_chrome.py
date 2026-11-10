"""The viewer page in a real browser: headless Chrome loads a page html() wrote and dumps the DOM after its script
has run, so these tests see what player.js's page code (startPage, show, render, drawReplay) did with the replays.
A probe script added to the test pages reads the canvas pixels at each robot's position and at a collision.

Chrome is found at $PATHFINDER_CHROME, the macOS application path, or google-chrome / chromium on the PATH; without
one these tests are skipped, unless PATHFINDER_REQUIRE_CHROME is set (CI sets it: its runners come with Chrome).
"""

import colorsys
import json
import os
import platform
import re
import shutil
import subprocess
from pathlib import Path

import pytest

import pathfinder as pf
from pathfinder.replay import html, positions_at, record
from pathfinder.simulator import random_delays


def find_chrome():
    candidates = [os.environ.get("PATHFINDER_CHROME"), "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"]
    candidates += [shutil.which(n) for n in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser")]
    return next((c for c in candidates if c and Path(c).exists()), None)


CHROME = find_chrome()
pytestmark = pytest.mark.skipif(CHROME is None and not os.environ.get("PATHFINDER_REQUIRE_CHROME"),
                                reason="no Chrome found (CI sets PATHFINDER_REQUIRE_CHROME)")

# Written into the page after player.js: once the page has loaded, the colour of the first canvas at each robot's
# position (a quarter of a cell above its centre: inside its disc, clear of its number and the pause sign) and just
# inside a vertex collision's ring, as JSON in a <pre>.
PROBE = """<pre id="probe"></pre><script>
window.addEventListener("load", () => {
  const replay = JSON.parse(document.getElementById("replays").textContent)[0];
  const canvas = document.querySelector("canvas");
  const cell = parseFloat(canvas.style.width) / replay.map.width;
  const dpr = canvas.width / parseFloat(canvas.style.width);
  const g = canvas.getContext("2d");
  const at = (x, y) => Array.from(g.getImageData(Math.floor(x * dpr), Math.floor(y * dpr), 1, 1).data.slice(0, 3));
  const time = Number((/t = ([0-9.]+)/.exec(document.getElementById("clock").textContent) || [0, 0])[1]);
  const positions = positionsAt(replay, time);  // player.js is a classic script: its functions are globals
  document.getElementById("probe").textContent = JSON.stringify({
    cell, time, collision: getComputedStyle(document.documentElement).getPropertyValue("--collision").trim(),
    robots: positions.map(([x, y]) => at((x + 0.5) * cell, (y + 0.25) * cell)),
    rings: collisionsAt(replay, time).filter((c) => c.kind === "vertex").map((c) => {
      const x = c.cell % replay.map.width, y = Math.floor(c.cell / replay.map.width);
      return at((x + 0.5 + 0.55) * cell, (y + 0.5) * cell);
    }),
  });
});
</script>"""


def page_for(replays, tmp_path, probe=True):
    text = html(replays)
    if probe:
        text = text.replace("</body>", PROBE + "\n</body>")
    path = tmp_path / "page.html"
    path.write_text(text)
    return path


def dump_dom(path, tmp_path, fragment=""):
    args = [CHROME or "google-chrome", "--headless", "--disable-gpu", "--no-first-run", "--no-default-browser-check",
            f"--user-data-dir={tmp_path / 'profile'}", "--window-size=1200,900", "--virtual-time-budget=3000",
            "--dump-dom", path.as_uri() + fragment]
    if platform.system() == "Linux":
        args.insert(1, "--no-sandbox")  # Ubuntu 24.04 restricts the user namespaces Chrome's sandbox needs
    r = subprocess.run(args, capture_output=True, text=True, timeout=90)
    assert r.returncode == 0, r.stderr[-2000:]
    return r.stdout


def text_of(dom, element_id):
    m = re.search(rf'id="{element_id}"[^>]*>([^<]*)<', dom)
    assert m, element_id
    return m.group(1)


def counts(dom):
    return re.findall(r'class="counts">([^<]*)<', dom)


@pytest.fixture(scope="module")
def runs():
    grid = pf.warehouse_map(pf.WarehouseLayout())
    instance = pf.make_instance(grid, pf.random_scenario(grid, 16, 1, "warehouse.map"), 16)
    plan = pf.cbs(instance, max_nodes=100_000).plan
    return [record(instance, plan, policy, random_delays(0.1, seed=1), planner="cbs")
            for policy in ("timetable", "ordered")]


def rgb(hue):
    return [round(v * 255) for v in colorsys.hls_to_rgb(hue / 360, 0.5, 0.7)]


def close(a, b, tolerance=4):
    return all(abs(x - y) <= tolerance for x, y in zip(a, b))


def test_the_page_draws_both_runs_at_the_start(runs, tmp_path):
    dom = dump_dom(page_for(runs, tmp_path), tmp_path)
    assert text_of(dom, "error") == ""
    assert text_of(dom, "clock") == f"t = 0.0 / {len(runs[0]['delayed'])}"
    assert counts(dom) == ["0/16 robots done · 3 delay steps · 0 collisions"] * 2
    assert len(re.findall(r'<canvas width="\d+" height="\d+"', dom)) == 2
    probe = json.loads(text_of(dom, "probe").replace("&quot;", '"'))
    for i, colour in enumerate(probe["robots"]):
        assert close(colour, rgb((i * 137.508) % 360)), (i, colour)


def test_a_link_opens_the_page_at_a_moment_with_its_collision(runs, tmp_path):
    c = runs[0]["collisions"][0]
    dom = dump_dom(page_for(runs, tmp_path), tmp_path, f"#t={c['time']}")
    assert text_of(dom, "clock") == f"t = {c['time']:.1f} / {len(runs[0]['delayed'])}"
    timetable, ordered = counts(dom)
    assert "· 1 collision" in timetable and "· 0 collisions" in ordered
    probe = json.loads(text_of(dom, "probe").replace("&quot;", '"'))
    assert probe["time"] == c["time"] and probe["rings"]
    # The ring is the theme's --collision colour (headless Chrome follows the system's light or dark mode).
    assert probe["collision"].lower() in ("#d1242f", "#ff5a5f")
    red = [int(probe["collision"][k:k + 2], 16) for k in (1, 3, 5)]
    assert all(close(colour, red, 8) for colour in probe["rings"]), probe["rings"]
    positions = positions_at(runs[0], c["time"])
    for i, colour in enumerate(probe["robots"]):
        overlapped = any(j != i and abs(q[0] - positions[i][0]) + abs(q[1] - positions[i][1]) < 1
                         for j, q in enumerate(positions))
        if not overlapped:
            assert close(colour, rgb((i * 137.508) % 360)), (i, colour)


@pytest.mark.parametrize("fragment, expected", [("#t=999", "31.0"), ("#t=-3", "0.0"), ("#t=abc", "0.0")])
def test_a_link_past_the_end_or_malformed(runs, tmp_path, fragment, expected):
    dom = dump_dom(page_for(runs, tmp_path, probe=False), tmp_path, fragment)
    assert text_of(dom, "clock") == f"t = {expected} / 31"


def test_a_broken_embedded_replay_is_reported(runs, tmp_path):
    text = html(runs).replace('"format":"pathfinder-replay"', '"format":"something-else"', 1)
    path = tmp_path / "broken.html"
    path.write_text(text)
    dom = dump_dom(path, tmp_path)
    assert "can't be shown" in text_of(dom, "error") and "replay 1" in text_of(dom, "error")
