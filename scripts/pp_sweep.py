"""Prioritized planning on MovingAI benchmark maps with their random scenarios: how often it solves the first k
agents of a scenario, how long it takes, and how far its sum of costs is above the lower bound (the sum of the agents'
distances alone). Day 4 runs the full comparison with CBS; this is the baseline's first look.

    make maps release
    PYTHONPATH=build/release/python .venv/bin/python scripts/pp_sweep.py \\
        --maps empty-32-32 random-32-32-20 warehouse-10-20-10-2-1 den520d --agents 50 100 200 --scenarios 1-5

Prints a Markdown table: map, agents, order, solved / tried, the median and the largest wall time over every try, and
the mean SOC / lower bound over the solved ones. Each try has a time limit (--time-limit, default 30 s): a try stopped
by it counts as not solved. --orders compares agent orders, as `pathfinder plan --order` does: the scenario's,
shortest or longest distance alone first (ties by scenario order), or its reverse.
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


def alone(grid, agents):
    # Each agent's distance to its goal alone: one BFS per agent; distances() returns a list over every cell, so this
    # is the slow part on big maps.
    return [pf.distances(grid, a.goal)[a.start] for a in agents]


def agent_order(name, dist):
    agents = list(range(len(dist)))
    if name == "scenario":
        return agents
    if name == "reverse":
        return agents[::-1]
    if name == "shortest":
        return sorted(agents, key=lambda i: dist[i])  # sorted is stable: ties keep the scenario's order
    if name == "longest":
        return sorted(agents, key=lambda i: -dist[i])
    raise ValueError(name)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--data", type=Path, default=ROOT / "data")
    p.add_argument("--maps", nargs="+", required=True, help="map names, e.g. random-32-32-20")
    p.add_argument("--agents", nargs="+", type=int, required=True)
    p.add_argument("--scenarios", default="1-5", help="random scenarios to use, e.g. 1-5 (of 25)")
    p.add_argument("--time-limit", type=float, default=30.0, help="seconds per try")
    p.add_argument("--orders", nargs="+", default=["scenario"], choices=["scenario", "reverse", "shortest", "longest"])
    args = p.parse_args(argv)

    print(f"time limit {args.time_limit:g} s per try; scenarios {args.scenarios} (random)", file=sys.stderr)
    print("| map | agents | order | solved | median time | max time | SOC / lower bound |")
    print("|---|---:|---|---:|---:|---:|---:|")
    for name in args.maps:
        grid = pf.load_map(args.data / "mapf-map" / f"{name}.map")
        scens = [pf.load_scenario(args.data / "scen-random" / f"{name}-random-{n}.scen")
                 for n in scenario_numbers(args.scenarios)]
        for k in args.agents:
            if any(len(s) < k for s in scens):
                print(f"| {name} | {k} | | (the scenarios have fewer entries) | | | |")
                continue
            insts = [pf.make_instance(grid, s, k) for s in scens]
            dists = [alone(grid, inst.agents) for inst in insts]
            for order in args.orders:
                solved, times, ratios = 0, [], []
                for inst, dist in zip(insts, dists):
                    started = time.perf_counter()
                    r = pf.prioritized_planning(inst, time_limit=args.time_limit, order=agent_order(order, dist))
                    times.append(time.perf_counter() - started)
                    if r.solved:
                        check = pf.validate(inst, r.plan)
                        if not check.valid or check.sum_of_costs != r.sum_of_costs:
                            sys.exit(f"{name}, {k} agents, {order}: the planner's plan doesn't check out")
                        solved += 1
                        ratios.append(r.sum_of_costs / max(1, sum(dist)))
                ratio = f"{statistics.mean(ratios):.3f}" if ratios else "-"
                print(f"| {name} | {k} | {order} | {solved} / {len(scens)} | {statistics.median(times) * 1000:.1f} ms "
                      f"| {max(times) * 1000:.1f} ms | {ratio} |", flush=True)


if __name__ == "__main__":
    main()
