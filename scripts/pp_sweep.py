"""Prioritized planning on MovingAI benchmark maps with their random scenarios: how often it solves the first k
agents of a scenario, how long it takes, and how far its sum of costs is above the lower bound (the sum of the agents'
distances alone). Day 4 runs the full comparison with CBS; this is the baseline's first look.

    make maps release
    PYTHONPATH=build/release/python .venv/bin/python scripts/pp_sweep.py \\
        --maps empty-32-32 random-32-32-20 warehouse-10-20-10-2-1 den520d --agents 50 100 200 --scenarios 1-5

Prints a Markdown table: map, agents, solved / tried, the median and the largest wall time over every try, and the
mean SOC / lower bound over the solved ones. Each try has a time limit (--time-limit, default 30 s): a try stopped by
it counts as not solved.
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

import pathfinder as pf

ROOT = Path(__file__).resolve().parents[1]


def scenario_numbers(text):
    first, _, last = text.partition("-")
    return range(int(first), int(last or first) + 1)


def lower_bound(grid, agents):
    # One BFS per distinct goal; distances() returns a list over every cell, so this is the slow part on big maps.
    return sum(pf.distances(grid, a.goal)[a.start] for a in agents)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--data", type=Path, default=ROOT / "data")
    p.add_argument("--maps", nargs="+", required=True, help="map names, e.g. random-32-32-20")
    p.add_argument("--agents", nargs="+", type=int, required=True)
    p.add_argument("--scenarios", default="1-5", help="random scenarios to use, e.g. 1-5 (of 25)")
    p.add_argument("--time-limit", type=float, default=30.0, help="seconds per try")
    args = p.parse_args(argv)

    print(f"time limit {args.time_limit:g} s per try; scenarios {args.scenarios} (random)", file=sys.stderr)
    print("| map | agents | solved | median time | max time | SOC / lower bound |")
    print("|---|---:|---:|---:|---:|---:|")
    for name in args.maps:
        grid = pf.load_map(args.data / "mapf-map" / f"{name}.map")
        scens = [pf.load_scenario(args.data / "scen-random" / f"{name}-random-{n}.scen")
                 for n in scenario_numbers(args.scenarios)]
        for k in args.agents:
            if any(len(s) < k for s in scens):
                print(f"| {name} | {k} | (the scenarios have fewer entries) | | | |")
                continue
            solved, times, ratios = 0, [], []
            for s in scens:
                inst = pf.make_instance(grid, s, k)
                started = time.perf_counter()
                r = pf.prioritized_planning(inst, time_limit=args.time_limit)
                times.append(time.perf_counter() - started)
                if r.solved:
                    check = pf.validate(inst, r.plan)
                    if not check.valid or check.sum_of_costs != r.sum_of_costs:
                        sys.exit(f"{name}, {k} agents: the planner's plan doesn't check out")
                    solved += 1
                    ratios.append(r.sum_of_costs / max(1, lower_bound(grid, inst.agents)))
            ratio = f"{statistics.mean(ratios):.3f}" if ratios else "-"
            print(f"| {name} | {k} | {solved} / {len(scens)} | {statistics.median(times) * 1000:.1f} ms "
                  f"| {max(times) * 1000:.1f} ms | {ratio} |", flush=True)


if __name__ == "__main__":
    main()
