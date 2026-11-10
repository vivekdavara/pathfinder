// The replay viewer's pure functions (python/pathfinder/viewer/player.js) under Node's test runner:
//   node --test tests/viewer/player.test.cjs
// CTest runs it as viewer.node. The replays here are written by hand in the format pathfinder/replay.py records.
"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const path = require("node:path");
const player = require(path.join(__dirname, "..", "..", "python", "pathfinder", "viewer", "player.js"));

// A 5 x 1 corridor, as tests/python/test_replay.py's: robot 0 follows robot 1 one cell behind; robot 1 is delayed in
// step 0, and as a timetable robot 0 drives into it (a vertex collision at time 1 in cell 1).
function corridor() {
  return {
    format: "pathfinder-replay",
    version: 1,
    title: "corridor",
    map: { name: "corridor", width: 5, height: 1, rows: ["....."] },
    robots: [{ start: 0, goal: 3 }, { start: 1, goal: 4 }],
    planner: null,
    policy: "timetable",
    plan: [[0, 1, 2, 3], [1, 2, 3, 4]],
    trajectories: [[0, 1, 2, 3, 3], [1, 1, 2, 3, 4]],
    delayed: [[1], [], [], []],
    collisions: [
      { kind: "vertex", a: 0, b: 1, time: 1, cell: 1, to: null },
      { kind: "vertex", a: 0, b: 1, time: 2, cell: 2, to: null },
      { kind: "vertex", a: 0, b: 1, time: 3, cell: 3, to: null },
    ],
    summary: { completed: true, steps: 4, planned_sum_of_costs: 7, costs: [3, 4], sum_of_costs: 7, makespan: 4,
               delay_steps: 1, collisions: 3 },
  };
}

// Two robots swapping across one edge of a 2 x 2 grid between times 0 and 1 (a timetable run of a bad schedule).
function swap() {
  return {
    format: "pathfinder-replay", version: 1, title: "", planner: null, policy: "timetable",
    map: { name: "", width: 2, height: 2, rows: ["..", ".."] },
    robots: [{ start: 0, goal: 1 }, { start: 1, goal: 0 }],
    plan: [[0, 1], [1, 0]],
    trajectories: [[0, 1], [1, 0]],
    delayed: [[]],
    collisions: [{ kind: "swap", a: 0, b: 1, time: 0, cell: 0, to: 1 }],
    summary: {},
  };
}

test("positions are cell centres at whole times and straight lines between them", () => {
  const r = corridor();
  assert.deepEqual(player.positionsAt(r, 0), [[0, 0], [1, 0]]);
  assert.deepEqual(player.positionsAt(r, 0.5), [[0.5, 0], [1, 0]]); // robot 1 is delayed in step 0
  assert.deepEqual(player.positionsAt(r, 1.25), [[1.25, 0], [1.25, 0]]);
  assert.deepEqual(player.positionsAt(r, 4), [[3, 0], [4, 0]]);
});

test("positions are clamped to the run", () => {
  const r = corridor();
  assert.deepEqual(player.positionsAt(r, -3), player.positionsAt(r, 0));
  assert.deepEqual(player.positionsAt(r, 99), player.positionsAt(r, 4));
  assert.deepEqual(player.stepOf(r, 4), [3, 1]);
  assert.deepEqual(player.stepOf(r, 2.5), [2, 0.5]);
});

test("rows come from the map width", () => {
  const r = {
    ...corridor(), map: { name: "", width: 2, height: 3, rows: ["..", "..", ".."] },
    robots: [{ start: 0, goal: 4 }], plan: [[0, 2, 4]], trajectories: [[0, 2, 4]], delayed: [[], []], collisions: [],
  };
  assert.deepEqual(player.positionsAt(r, 1.5), [[0, 1.5]]);
});

test("a run with no steps stays at its start", () => {
  const r = { ...corridor(), robots: [{ start: 2, goal: 2 }], plan: [[2]], trajectories: [[2]], delayed: [],
              collisions: [] };
  assert.deepEqual(player.positionsAt(r, 0), [[2, 0]]);
  assert.deepEqual(player.positionsAt(r, 5), [[2, 0]]);
  assert.deepEqual(player.delayedAt(r, 0), []);
});

test("the plan's ghost runs on the plan's clock and stops at its path's end", () => {
  const r = corridor();
  assert.deepEqual(player.plannedAt(r, 0.5), [[0.5, 0], [1.5, 0]]);
  assert.deepEqual(player.plannedAt(r, 3.5), [[3, 0], [4, 0]]);
  assert.deepEqual(player.plannedAt(r, 10), [[3, 0], [4, 0]]);
});

test("delays belong to their step", () => {
  const r = corridor();
  assert.deepEqual(player.delayedAt(r, 0), [1]);
  assert.deepEqual(player.delayedAt(r, 0.99), [1]);
  assert.deepEqual(player.delayedAt(r, 1), []);
  assert.deepEqual(player.delayedAt(r, 4), []);
  assert.deepEqual(player.delayedAt(r, -1), []);
});

test("a vertex collision shows for half a step either side of its time", () => {
  const r = corridor();
  const times = (t) => player.collisionsAt(r, t).map((c) => c.time);
  assert.deepEqual(times(0.4), []);
  assert.deepEqual(times(0.5), [1]);
  assert.deepEqual(times(1), [1]);
  assert.deepEqual(times(1.49), [1]);
  assert.deepEqual(times(1.5), [2]);
  assert.deepEqual(times(3.6), []); // time 3's collision ends at 3.5; the run goes on to 4
});

test("a collision at the run's last moment shows at its end", () => {
  const r = corridor();
  r.delayed = r.delayed.slice(0, 3);
  r.trajectories = r.trajectories.map((p) => p.slice(0, 4));
  assert.deepEqual(player.collisionsAt(r, 3).map((c) => c.time), [3]);
});

test("a swap shows during its step", () => {
  const r = swap();
  assert.equal(player.collisionsAt(r, 0).length, 1);
  assert.equal(player.collisionsAt(r, 0.99).length, 1);
  assert.equal(player.collisionsAt(r, 1).length, 0);
});

test("counts: delays of steps begun, collisions reached, robots on their goals for good", () => {
  const r = corridor();
  assert.deepEqual(player.countsAt(r, 0), { delays: 1, collisions: 0, finished: 0 });
  assert.deepEqual(player.countsAt(r, 1), { delays: 1, collisions: 1, finished: 0 });
  assert.deepEqual(player.countsAt(r, 3), { delays: 1, collisions: 3, finished: 1 });
  assert.deepEqual(player.countsAt(r, 4), { delays: 1, collisions: 3, finished: 2 });
});

test("robot colours differ between neighbours", () => {
  const hues = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9].map(player.robotHue);
  for (let i = 1; i < hues.length; i++) {
    const d = Math.abs(hues[i] - hues[i - 1]);
    assert.ok(Math.min(d, 360 - d) > 100, `robots ${i - 1} and ${i}`);
  }
});

test("a well-formed replay has no shape problems", () => {
  assert.deepEqual(player.shapeProblems(corridor()), []);
  assert.deepEqual(player.shapeProblems(swap()), []);
});

test("shape problems are found", () => {
  const broken = (change) => {
    const r = corridor();
    change(r);
    return player.shapeProblems(r);
  };
  assert.match(player.shapeProblems(null)[0], /object/);
  assert.match(player.shapeProblems([])[0], /object/);
  assert.match(broken((r) => { r.format = "x"; }).join(), /format/);
  assert.match(broken((r) => { r.version = 2; }).join(), /version 2/);
  assert.match(broken((r) => { r.map.rows = ["...."]; }).join(), /map/);
  assert.match(broken((r) => { r.trajectories[0].pop(); }).join(), /5 cells/);
  assert.match(broken((r) => { r.trajectories[1][2] = 5; }).join(), /trajectories/);
  assert.match(broken((r) => { r.plan.pop(); }).join(), /plan/);
  assert.match(broken((r) => { r.delayed[0] = [2]; }).join(), /delayed/);
  assert.match(broken((r) => { r.robots[0].goal = 1.5; }).join(), /robots/);
  assert.match(broken((r) => { r.collisions[0].kind = "edge"; }).join(), /collisions/);
  assert.match(broken((r) => { delete r.summary; }).join(), /summary/);
});

test("a file holds one replay or a list", () => {
  assert.equal(player.asReplays(corridor()).length, 1);
  assert.equal(player.asReplays([corridor(), swap()]).length, 2);
  assert.throws(() => player.asReplays([]), /no replays/);
  assert.throws(() => player.asReplays([corridor(), { format: "x" }]), /replay 2/);
});

test("labels", () => {
  assert.equal(player.label(corridor()), "timetable · corridor");
  assert.equal(player.label({ ...corridor(), planner: "cbs" }), "CBS · timetable · corridor");
});
