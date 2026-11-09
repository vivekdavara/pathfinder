# Day-4 raw results

The outputs behind README "Results (day 4)", as the scripts printed them. Measured on 2026-11-09 on an Apple M3,
macOS 26.6.2, Apple Clang 21 (the `release` preset's module), Python 3.11.12, on battery; the MovingAI maps and random
scenarios from `make maps`. The sweeps ran one try at a time on one core, alongside a review agent in its own
worktree and the delay experiments (load averages 4 to 8), so times near the 5 s limit carry that load. Every
map of all three sweeps printed "asleep for 0 s of it" (`sweep.err`, `sweep_generated.err`, `sweep_pp_large.err`).

| File | Command (with `PYTHONPATH=build/release/python .venv/bin/python`) | Code |
|---|---|---|
| `sweep.jsonl`, `sweep.md`, `sweep.err` | `scripts/sweep.py --maps empty-32-32 random-32-32-20 room-32-32-4 warehouse-10-20-10-2-1 --agents 10 20 30 40 50 60 70 80 90 100 --scenarios 1-25 --time-limit 5 --jsonl sweep.jsonl` (11:23 to 11:48) | `6808315` |
| `sweep_pp_large.jsonl`, `sweep_pp_large.md` | `scripts/sweep.py --planners pp --maps empty-32-32 random-32-32-20 room-32-32-4 warehouse-10-20-10-2-1 --agents 150 200 300 400 --scenarios 1-25 --time-limit 5 --jsonl sweep_pp_large.jsonl` | `e2dd982` |
| `sweep_generated.jsonl`, `sweep_generated.md` | `scripts/sweep.py --maps generated-warehouse --agents 5 10 15 20 25 30 35 40 50 60 --scenarios 1-25 --time-limit 5 --jsonl sweep_generated.jsonl` (synthetic floor) | `023f5ce` |
| `sweep_random_quiet.jsonl`, `.md`, `.err` | the first command with `--maps random-32-32-20 --agents 10 20 30 40 50`, re-run at 12:08-12:13 on a quiet machine on AC (load 3) to measure the load's effect | `7818c10` |
| `summary.md` | `.venv/bin/python scripts/sweep_summary.py results/day4/sweep.jsonl results/day4/sweep_pp_large.jsonl` | `e2dd982` |
| `delays_warehouse-10-20-10-2-1_20.md` | `scripts/delays.py --map warehouse-10-20-10-2-1 --agents 20 --scenarios 1-25 --probabilities 0.01 0.05 0.1 0.2 --seeds 20 --single 1 3 5 --time-limit 5` | `b1a1fcd` |
| `delays_random-32-32-20_20.md` | the same with `--map random-32-32-20` | `c7f4139` |

The warehouse delay run printed the same tables at `c7f4139` (the simulator's first, quadratic step) and at `b1a1fcd`
(the linear one); the random-map run used the first. A record in the JSONL files is one try: map, agents, scenario,
planner, status, seconds (the planner's own clock), sum of costs (solved only), the lower bound, expansions and
CBS's nodes generated.
