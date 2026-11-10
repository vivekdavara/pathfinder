// The replay viewer (DESIGN.md "The replay viewer (day 5)"): plays pathfinder replay documents (pathfinder/replay.py)
// on canvases, several side by side on one timeline. The first half is pure functions of a replay and a moment of the
// run, which player.test.cjs tests under Node; the second half draws and wires up the page.
//
// Time is continuous: at a whole time t every robot is on the centre of its cell trajectories[i][t]; during step t
// (from t to t + 1) it moves in a straight line to trajectories[i][t + 1]. A robot delayed in step t holds its cell.
"use strict";

const FORMAT = "pathfinder-replay";
const VERSION = 1;

function steps(replay) {
  return replay.delayed.length;
}

function clampTime(replay, time) {
  return Math.min(Math.max(time, 0), steps(replay));
}

// The step a moment falls in, and how far through it: (t, f) with time = t + f. The run's end is the end of its last
// step (f = 1), so that positions at the end are the last cells.
function stepOf(replay, time) {
  const n = steps(replay);
  const clamped = clampTime(replay, time);
  const t = Math.min(Math.floor(clamped), Math.max(n - 1, 0));
  return [t, clamped - t];
}

function xy(replay, cell) {
  const w = replay.map.width;
  return [cell % w, Math.floor(cell / w)];
}

function between(replay, a, b, f) {
  const [ax, ay] = xy(replay, a);
  const [bx, by] = xy(replay, b);
  return [ax + (bx - ax) * f, ay + (by - ay) * f];
}

// Every robot's (x, y) at a moment, as pathfinder.replay.positions_at computes it.
function positionsAt(replay, time) {
  const [t, f] = stepOf(replay, time);
  return replay.trajectories.map((path) => between(replay, path[t], t + 1 < path.length ? path[t + 1] : path[t], f));
}

// Where the plan has every robot at the same moment (a robot past its path's end is on its last cell): the viewer
// draws it as a ghost, so the gap between a robot and its ghost is how far behind the plan the delays have put it.
function plannedAt(replay, time) {
  const tt = Math.max(time, 0);
  const t = Math.floor(tt);
  const f = tt - t;
  return replay.plan.map((path) => {
    const a = path[Math.min(t, path.length - 1)];
    const b = path[Math.min(t + 1, path.length - 1)];
    return between(replay, a, b, f);
  });
}

// The robots delayed in the step a moment falls in (none at or after the end).
function delayedAt(replay, time) {
  if (time < 0 || time >= steps(replay)) return [];
  return replay.delayed[Math.floor(time)];
}

// The collisions to show at a moment: a vertex collision at time T from T - 0.5 to T + 0.5 (so one at the run's last
// moment shows at its end), a swap during its step [t, t + 1).
function collisionsAt(replay, time) {
  return replay.collisions.filter((c) => {
    if (c.kind === "swap") return c.time <= time && time < c.time + 1;
    return c.time - 0.5 <= time && time < c.time + 0.5;
  });
}

// What has happened by a moment: robot-steps of delay in the steps begun, collisions reached (a vertex collision at
// its time, a swap once its step has begun), robots on their goals for good (their cost reached).
function countsAt(replay, time) {
  let delays = 0;
  for (let t = 0; t < steps(replay) && t <= time; t++) delays += replay.delayed[t].length;
  // A swap's time is its step's start, a vertex collision's the moment itself: either way, reached at c.time.
  const collisions = replay.collisions.filter((c) => c.time <= time).length;
  let finished = 0;
  replay.trajectories.forEach((path, i) => {
    const goal = replay.robots[i].goal;
    let arrived = path.length - 1;
    while (arrived > 0 && path[arrived - 1] === goal) arrived--;
    if (path[path.length - 1] === goal && arrived <= time) finished++;
  });
  return { delays, collisions, finished };
}

// A colour per robot: hues spread by the golden angle, so neighbouring indices differ. The same formula as
// pathfinder/render.py's, so a GIF and the page agree.
function robotHue(i) {
  return (i * 137.508) % 360;
}

// The shape the drawing code relies on. The page can't run the simulator, so it can't check a replay the way
// pathfinder.replay.check does (every replay the Python side writes has passed that); this stops a hand-made or
// truncated file from drawing garbage or throwing half way through.
function shapeProblems(doc) {
  const problems = [];
  const isInt = (v) => Number.isInteger(v);
  if (doc === null || typeof doc !== "object" || Array.isArray(doc)) return ["not a JSON object"];
  if (doc.format !== FORMAT) problems.push(`format is not "${FORMAT}"`);
  if (doc.version !== VERSION) problems.push(`version ${doc.version} isn't supported (this viewer reads ${VERSION})`);
  const m = doc.map;
  if (!m || !isInt(m.width) || !isInt(m.height) || m.width < 1 || m.height < 1 || !Array.isArray(m.rows) ||
      m.rows.length !== m.height || !m.rows.every((r) => typeof r === "string" && r.length === m.width)) {
    problems.push("map must have a width, a height and that many rows of that many characters");
    return problems;
  }
  const cells = m.width * m.height;
  const isCell = (c) => isInt(c) && c >= 0 && c < cells;
  const n = Array.isArray(doc.robots) ? doc.robots.length : -1;
  if (n < 0 || !doc.robots.every((r) => r && isCell(r.start) && isCell(r.goal))) problems.push("robots must be starts and goals");
  if (!Array.isArray(doc.delayed) || !doc.delayed.every((d) => Array.isArray(d) && d.every((i) => isInt(i) && i >= 0 && i < n))) {
    problems.push("delayed must be a list of robot lists");
    return problems;
  }
  const length = doc.delayed.length + 1;
  for (const key of ["plan", "trajectories"]) {
    const paths = doc[key];
    if (!Array.isArray(paths) || paths.length !== n || !paths.every((p) => Array.isArray(p) && p.length > 0 && p.every(isCell))) {
      problems.push(`${key} must be one path of cells per robot`);
    }
  }
  if (Array.isArray(doc.trajectories) && !doc.trajectories.every((p) => Array.isArray(p) && p.length === length)) {
    problems.push(`every trajectory must have ${length} cells (one per time step)`);
  }
  if (!Array.isArray(doc.collisions) || !doc.collisions.every((c) => c && (c.kind === "vertex" || c.kind === "swap") &&
      isInt(c.a) && isInt(c.b) && isInt(c.time) && isCell(c.cell) && (c.kind === "vertex" || isCell(c.to)))) {
    problems.push("collisions must be vertex or swap collisions");
  }
  if (!doc.summary || typeof doc.summary !== "object") problems.push("summary is missing");
  return problems;
}

// The replays a page or a loaded file holds: one replay, or a list of them shown side by side.
function asReplays(doc) {
  const list = Array.isArray(doc) ? doc : [doc];
  if (list.length === 0) throw new Error("no replays in the file");
  list.forEach((r, k) => {
    const problems = shapeProblems(r);
    if (problems.length) throw new Error(`replay ${k + 1}: ${problems.join("; ")}`);
  });
  return list;
}

// The moment a link asks for, "#t=18" or "#t=2.5"; null for anything else (a negative or non-numeric time, another
// fragment). The page clamps it to the run.
function timeFromHash(hash) {
  const m = /^#t=(\d+(?:\.\d+)?)$/.exec(hash || "");
  return m ? Number(m[1]) : null;
}

function count(n, what) {
  return `${n} ${what}${n === 1 ? "" : "s"}`;
}

function label(replay) {
  const parts = [replay.policy];
  if (replay.planner) parts.unshift(replay.planner.toUpperCase());
  if (replay.map.name) parts.push(replay.map.name);
  return parts.join(" · ");
}

// --- the page ---

function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function drawReplay(canvas, replay, time, cellSize) {
  const dpr = window.devicePixelRatio || 1;
  const w = replay.map.width * cellSize;
  const h = replay.map.height * cellSize;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    canvas.style.width = `${w}px`;
    canvas.style.height = `${h}px`;
  }
  const g = canvas.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.fillStyle = css("--floor");
  g.fillRect(0, 0, w, h);
  g.fillStyle = css("--wall");
  replay.map.rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) if (row[x] === "@") g.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
  });
  const centre = ([x, y]) => [(x + 0.5) * cellSize, (y + 0.5) * cellSize];
  const n = replay.robots.length;
  const colour = (i, l = 50, a = 1) => `hsla(${robotHue(i)}, 70%, ${l}%, ${a})`;
  // Goals: a square outline in the robot's colour.
  replay.robots.forEach((r, i) => {
    const [x, y] = xy(replay, r.goal);
    g.strokeStyle = colour(i, 50, 0.9);
    g.lineWidth = Math.max(1, cellSize / 10);
    const inset = cellSize * 0.18;
    g.strokeRect(x * cellSize + inset, y * cellSize + inset, cellSize - 2 * inset, cellSize - 2 * inset);
  });
  // The plan's ghosts.
  const ghosts = plannedAt(replay, time);
  const robots = positionsAt(replay, time);
  ghosts.forEach((p, i) => {
    const [gx, gy] = centre(p);
    const [rx, ry] = centre(robots[i]);
    if (Math.hypot(gx - rx, gy - ry) < 1e-6) return;
    g.strokeStyle = colour(i, 50, 0.55);
    g.setLineDash([Math.max(2, cellSize / 6), Math.max(2, cellSize / 6)]);
    g.lineWidth = Math.max(1, cellSize / 12);
    g.beginPath();
    g.arc(gx, gy, cellSize * 0.32, 0, 2 * Math.PI);
    g.stroke();
    g.setLineDash([]);
  });
  // Robots; the ones delayed in this step with a thick outline and a pause sign.
  const delayed = new Set(delayedAt(replay, time));
  robots.forEach((p, i) => {
    const [cx, cy] = centre(p);
    g.fillStyle = colour(i);
    g.beginPath();
    g.arc(cx, cy, cellSize * 0.38, 0, 2 * Math.PI);
    g.fill();
    if (delayed.has(i)) {
      g.strokeStyle = css("--delay");
      g.lineWidth = Math.max(1.5, cellSize / 7);
      g.stroke();
      g.fillStyle = css("--on-robot");
      const bw = cellSize * 0.08;
      const bh = cellSize * 0.3;
      g.fillRect(cx - bw * 1.6, cy - bh / 2, bw, bh);
      g.fillRect(cx + bw * 0.6, cy - bh / 2, bw, bh);
    } else if (cellSize >= 18 && n <= 99) {
      g.fillStyle = css("--on-robot");
      g.font = `600 ${Math.round(cellSize * 0.38)}px system-ui, sans-serif`;
      g.textAlign = "center";
      g.textBaseline = "middle";
      g.fillText(String(i), cx, cy + 0.5);
    }
  });
  // Collisions: a red ring on the cell, or a red bar across the swapped edge.
  g.strokeStyle = css("--collision");
  g.lineWidth = Math.max(2, cellSize / 6);
  for (const c of collisionsAt(replay, time)) {
    const [ax, ay] = centre(xy(replay, c.cell));
    g.beginPath();
    if (c.kind === "vertex") {
      g.arc(ax, ay, cellSize * 0.55, 0, 2 * Math.PI);
    } else {
      const [bx, by] = centre(xy(replay, c.to));
      g.moveTo(ax, ay);
      g.lineTo(bx, by);
    }
    g.stroke();
  }
}

const PANEL_PADDING = 12; // viewer.html's figure padding
const WIDE = 1.6;

function startPage() {
  const state = { replays: [], time: 0, playing: false, speed: 2, last: null };
  const $ = (id) => document.getElementById(id);
  const panels = $("panels");
  const slider = $("time");
  const play = $("play");

  const end = () => Math.max(...state.replays.map(steps));

  // Cells as large as the panel's width allows (its padding taken off), with the map at most 62% of the window's
  // height, between 2 and 36 pixels.
  function cellSize(replay, panel) {
    const width = panel.clientWidth - 2 * PANEL_PADDING;
    const height = window.innerHeight * 0.62;
    return Math.max(2, Math.min(36, Math.floor(Math.min(width / replay.map.width, height / replay.map.height))));
  }

  function render() {
    slider.value = String(state.time);
    $("clock").textContent = `t = ${state.time.toFixed(1)} / ${end()}`;
    state.replays.forEach((replay, k) => {
      const panel = panels.children[k];
      drawReplay(panel.querySelector("canvas"), replay, state.time, cellSize(replay, panel));
      const c = countsAt(replay, state.time);
      panel.querySelector(".counts").textContent = `${c.finished}/${replay.robots.length} robots done · ` +
        `${count(c.delays, "delay step")} · ${count(c.collisions, "collision")}`;
    });
  }

  function show(replays) {
    state.replays = replays;
    state.time = 0;
    panels.replaceChildren(...replays.map((replay) => {
      const panel = document.createElement("figure");
      const title = document.createElement("figcaption");
      const s = replay.summary;
      const result = s.completed ? `SOC ${s.sum_of_costs} (planned ${s.planned_sum_of_costs}), makespan ${s.makespan}` :
        `stopped after ${s.steps} steps`;
      title.innerHTML = "<strong></strong><span></span>";
      title.firstChild.textContent = label(replay);
      title.lastChild.textContent = ` — ${replay.robots.length} robots, ${result}`;
      const counts = document.createElement("div");
      counts.className = "counts";
      panel.append(title, document.createElement("canvas"), counts);
      return panel;
    }));
    // Maps much wider than tall get the page's full width each, one above the other.
    panels.classList.toggle("stacked", replays.every((r) => r.map.width >= WIDE * r.map.height));
    slider.max = String(end());
    document.title = replays[0].title || "Pathfinder replay";
    $("heading").textContent = replays[0].title || "Pathfinder replay";
    render();
  }

  function setPlaying(on) {
    state.playing = on && end() > 0;
    play.textContent = state.playing ? "Pause" : "Play";
    state.last = null;
    if (state.playing) {
      if (state.time >= end()) state.time = 0;
      requestAnimationFrame(tick);
    }
  }

  function tick(now) {
    if (!state.playing) return;
    if (state.last !== null) state.time = Math.min(end(), state.time + ((now - state.last) / 1000) * state.speed);
    state.last = now;
    render();
    if (state.time >= end()) setPlaying(false);
    else requestAnimationFrame(tick);
  }

  function stepBy(d) {
    setPlaying(false);
    state.time = Math.min(end(), Math.max(0, Math.round(state.time) + d));
    render();
  }

  play.addEventListener("click", () => setPlaying(!state.playing));
  $("back").addEventListener("click", () => stepBy(-1));
  $("forward").addEventListener("click", () => stepBy(1));
  $("speed").addEventListener("change", (e) => { state.speed = Number(e.target.value); });
  slider.addEventListener("input", () => { setPlaying(false); state.time = Number(slider.value); render(); });
  window.addEventListener("resize", render);
  document.addEventListener("keydown", (e) => {
    // Keys typed into the slider, the speed menu or the file chooser are theirs; Space on a focused button is that
    // button's click (handling it here too would toggle twice).
    if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
    if (e.key === " " && e.target instanceof HTMLButtonElement) return;
    if (e.key === " ") { e.preventDefault(); setPlaying(!state.playing); }
    if (e.key === "ArrowLeft") stepBy(-1);
    if (e.key === "ArrowRight") stepBy(1);
  });
  $("file").addEventListener("change", async (e) => {
    const file = e.target.files[0];
    if (!file) return;
    try {
      show(asReplays(JSON.parse(await file.text())));
      $("error").textContent = "";
    } catch (err) {
      $("error").textContent = `Couldn't show ${file.name}: ${err.message}`;
    }
  });

  const embedded = $("replays");
  if (embedded && embedded.textContent.trim()) {
    try {
      show(asReplays(JSON.parse(embedded.textContent)));
      // A link can open the page at a moment: page.html#t=18.
      const start = timeFromHash(location.hash);
      if (start !== null) {
        state.time = Math.min(start, end());
        render();
      }
    } catch (err) {
      $("error").textContent = `The embedded replay can't be shown: ${err.message}`;
    }
  } else {
    $("error").textContent = "No replay in this page: choose a replay JSON file.";
  }
}

if (typeof document !== "undefined") {
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", startPage);
  else startPage();
}

if (typeof module !== "undefined") {
  module.exports = { positionsAt, plannedAt, delayedAt, collisionsAt, countsAt, robotHue, shapeProblems, asReplays,
                     stepOf, label, count, timeFromHash };
}
