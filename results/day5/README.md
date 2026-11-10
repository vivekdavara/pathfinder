# Day 5: replays

The README's animation and the page that plays it, from one command (Apple M3, macOS 26.6, Python 3.11.12, Pillow
13.0.0, the Release build of the module):

```bash
make release
PYTHONPATH=build/release/python .venv/bin/python scripts/replay.py --agents 16 --scenario 1 \
  --delay-probability 0.1 --seed 1 --html results/day5/warehouse-16.html --gif results/day5/warehouse-16.gif
```

| File | What |
|---|---|
| `warehouse-16.gif` | the animation in the README: 16 robots on the generated floor (synthetic: `pf.warehouse_map`'s default layout, `pf.random_scenario` seed 1), CBS's plan executed as a timetable (left) and ordered (right) under the same random delays |
| `warehouse-16.html` | the same two runs as a self-contained page: download it and open it in a browser (play, pause, step, scrub; or open another replay JSON file) |
| `warehouse-16.txt` | what the command printed |
| `movingai-100.txt` | what the two MovingAI runs below printed |

What it printed:

```
generated-warehouse (synthetic), scenario seed 1: 16 robots, cbs plan SOC 232, makespan 29
  timetable: SOC 260 (+28), makespan 31, 28 robot-steps of delay, 6 collisions; recorded and checked in 1.4 ms (the check alone: 0.6 ms)
  ordered  : SOC 265 (+33), makespan 31, 28 robot-steps of delay, 0 collisions; recorded and checked in 1.5 ms (the check alone: 0.7 ms)
wrote results/day5/warehouse-16.html (33.5 KiB)
wrote results/day5/warehouse-16.gif (125 frames, 234.3 KiB, in 0.30 s)
```

Both runs see the same delay draws (`random_delays(0.1, seed=1)` draws for every robot at every step, finished or
not). The timetable run's six collisions: robots 6 and 9 at t = 3, 5 and 13 at t = 4, 4 and 14 at t = 5, 6 and 8
(one pair meeting three times), and 6 and 11 at t = 12. Its sum of costs is lower than the ordered run's (260 vs
265) only because a robot that drives into another isn't stopped by it: a timetable's cost can't be had without the
collisions.

The same command on bigger MovingAI maps (downloaded by `make maps`, so their pages aren't committed), delays at
p = 0.05, seed 1, prioritized planning:

| Map, robots | Steps | Recorded and checked (timetable / ordered) | Check alone | Page | GIF (6 px cells, 2 frames a step) |
|---|---|---|---|---|---|
| empty-32-32, 100 | 53 | 9.7 / 13.2 ms | 6.2 / 7.0 ms | 100.5 KiB | 107 frames, 453.7 KiB, 0.28 s |
| warehouse-10-20-10-2-1, 100 | 206 | 42.1 / 55.7 ms | 23.3 / 31.2 ms | 356.1 KiB | 413 frames, 2,629.1 KiB, 4.64 s |

```bash
PYTHONPATH=build/release/python .venv/bin/python scripts/replay.py --map empty-32-32 --agents 100 --planner pp \
  --scenario 1 --delay-probability 0.05 --seed 1 --html empty.html --gif empty.gif --gif-cell 6 --gif-substeps 2
```

(and the same with `--map warehouse-10-20-10-2-1`; output in `movingai-100.txt`). On empty-32-32 the timetable run
had 77 collisions, on the warehouse 132; the ordered runs none. Single runs, on AC, with another session's work
running alongside (load average 4.7 to 5.5): the times are indications, not benchmarks. (An earlier run at load
3.4, before the page gained robot following and the GIF palette optimization, recorded the same runs in 8.4 / 10.7 and
31.6 / 41.4 ms, with 94.5 and 350.1 KiB pages and 813.8 KiB and 7,664.9 KiB GIFs.)
