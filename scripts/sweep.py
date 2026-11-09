"""Prioritized planning against CBS on benchmark maps: how often each solves the first k agents of a scenario within a
time limit, how long it takes, and what the plans cost (DESIGN.md "Measurements (day 4)").

    make maps release
    PYTHONPATH=build/release/python .venv/bin/python scripts/sweep.py \\
        --maps empty-32-32 random-32-32-20 room-32-32-4 warehouse-10-20-10-2-1 \\
        --agents 10 20 30 40 50 60 --scenarios 1-25 --time-limit 5 --jsonl sweep.jsonl

A map name is a MovingAI map in data/mapf-map with its random scenarios in data/scen-random, or `generated-warehouse`
(synthetic: pf.warehouse_map's default floor, with pf.random_scenario(grid, max agents, seed n) as scenario n, so CI
can run this without the downloaded maps). Each try runs one planner on one instance with the time limit; a try
stopped by it counts as not solved, and so does prioritized planning's NO_PATH (it is incomplete). Every plan is
checked with validate(), and its SOC against the lower bound (the sum of the agents' distances alone): the script
exits 1 if any plan fails either check, or if CBS's SOC is ever above prioritized planning's.

Prints a Markdown table per map: for each agent count, solved / tried for each planner, the median and largest wall
time (C++'s own clock, over every try, solved or not), and on the instances both solve, prioritized planning's SOC
over CBS's (the mean and the largest) and each one's SOC over the lower bound. Once a planner solves none at some
agent count, larger counts aren't tried for it (--keep-going tries them anyway). --jsonl writes one record per try.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import pathfinder as pf

ROOT = Path(__file__).resolve().parents[1]
GENERATED = "generated-warehouse"
PLANNERS = ("pp", "cbs")


def scenario_numbers(text: str) -> range:
    first, _, last = text.partition("-")
    return range(int(first), int(last or first) + 1)


def load(name: str, numbers: range, data: Path, most: int):
    """The grid and, for each scenario number, its scenario."""
    if name == GENERATED:
        grid = pf.warehouse_map(pf.WarehouseLayout())
        return grid, {n: pf.random_scenario(grid, most, n, "warehouse.map") for n in numbers}
    grid = pf.load_map(data / "mapf-map" / f"{name}.map")
    return grid, {n: pf.load_scenario(data / "scen-random" / f"{name}-random-{n}.scen") for n in numbers}


def run(planner: str, instance: pf.Instance, time_limit: float) -> pf.PlanResult:
    if planner == "pp":
        return pf.prioritized_planning(instance, time_limit=time_limit)
    return pf.cbs(instance, time_limit=time_limit)


def ms(seconds: float) -> str:
    if seconds < 0.01:
        return f"{seconds * 1000:.2f} ms"
    return f"{seconds * 1000:.1f} ms" if seconds < 1 else f"{seconds:.2f} s"


def asleep() -> float:
    # On macOS time.monotonic() stops while the machine sleeps and time.time() doesn't, so their difference grows by
    # every sleep: printed per map, so a run that straddled one can be spotted (the C++ clocks stop too, so a sleep
    # doesn't use up a try's time limit, but wall-clock comparisons across maps would be off).
    return time.time() - time.monotonic()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--data", type=Path, default=ROOT / "data")
    p.add_argument("--maps", nargs="+", required=True)
    p.add_argument("--agents", nargs="+", type=int, required=True)
    p.add_argument("--scenarios", default="1-25", help="random scenarios to use, e.g. 1-25")
    p.add_argument("--time-limit", type=float, default=5.0, help="seconds per try")
    p.add_argument("--planners", nargs="+", default=list(PLANNERS), choices=PLANNERS)
    p.add_argument("--keep-going", action="store_true", help="try every agent count even after a planner solves none")
    p.add_argument("--jsonl", type=Path, help="write one JSON record per try here")
    args = p.parse_args(argv)

    numbers = scenario_numbers(args.scenarios)
    out = args.jsonl.open("w") if args.jsonl else None
    print(f"time limit {args.time_limit:g} s per try; scenarios {args.scenarios} (random); "
          f"pathfinder {pf.__version__}", file=sys.stderr)
    failures = 0
    for name in args.maps:
        slept = asleep()
        grid, scens = load(name, numbers, args.data, max(args.agents))
        print(f"\n**{name}** ({grid.width} x {grid.height}, {grid.passable_count} free cells)\n")
        print("| agents | PP solved | CBS solved | PP median / max time | CBS median / max time | both solved "
              "| PP SOC / CBS SOC (mean, max) | PP SOC / LB | CBS SOC / LB |")
        print("|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
        gave_up: set[str] = set()
        for k in args.agents:
            if any(len(s) < k for s in scens.values()):
                print(f"| {k} | (the scenarios have fewer entries) | | | | | | | |")
                continue
            tries = {planner: [] for planner in args.planners if planner not in gave_up}
            if not tries:
                break
            for n, scen in scens.items():
                instance = pf.make_instance(grid, scen, k)
                bound = sum(pf.distances(grid, a.goal)[a.start] for a in instance.agents)
                for planner in tries:
                    r = run(planner, instance, args.time_limit)
                    record = {"map": name, "agents": k, "scenario": n, "planner": planner, "status": r.status.name,
                              "seconds": r.seconds, "sum_of_costs": r.sum_of_costs if r.solved else None,
                              "lower_bound": bound, "expansions": r.expansions,
                              "nodes_generated": r.nodes_generated}
                    if r.solved:
                        check = pf.validate(instance, r.plan)
                        if not check.valid or check.sum_of_costs != r.sum_of_costs or r.sum_of_costs < bound:
                            print(f"{name}, {k} agents, scenario {n}, {planner}: the plan doesn't check out",
                                  file=sys.stderr)
                            failures += 1
                    tries[planner].append(record)
                    if out:
                        out.write(json.dumps(record) + "\n")
                        out.flush()
            cells = []
            for planner in PLANNERS:
                got = tries.get(planner)
                cells.append(f"{sum(r['status'] == 'SOLVED' for r in got)} / {len(got)}" if got else "-")
            for planner in PLANNERS:
                got = tries.get(planner)
                if got:
                    times = [r["seconds"] for r in got]
                    cells.append(f"{ms(statistics.median(times))} / {ms(max(times))}")
                else:
                    cells.append("-")
            pp = {r["scenario"]: r for r in tries.get("pp", []) if r["status"] == "SOLVED"}
            cbs = {r["scenario"]: r for r in tries.get("cbs", []) if r["status"] == "SOLVED"}
            both = sorted(pp.keys() & cbs.keys())
            for n in both:
                if cbs[n]["sum_of_costs"] > pp[n]["sum_of_costs"]:
                    print(f"{name}, {k} agents, scenario {n}: CBS's SOC is above prioritized planning's",
                          file=sys.stderr)
                    failures += 1
            cells.append(str(len(both)))
            if both:
                ratios = [pp[n]["sum_of_costs"] / cbs[n]["sum_of_costs"] for n in both]
                cells.append(f"{statistics.mean(ratios):.4f}, {max(ratios):.4f}")
                for got in (pp, cbs):
                    cells.append(f"{statistics.mean(got[n]['sum_of_costs'] / got[n]['lower_bound'] for n in both):.4f}")
            else:
                cells += ["-", "-", "-"]
            print(f"| {k} | " + " | ".join(cells) + " |", flush=True)
            for planner, got in tries.items():
                if not args.keep_going and not any(r["status"] == "SOLVED" for r in got):
                    gave_up.add(planner)
        print(f"{name}: asleep for {max(0.0, asleep() - slept):.0f} s of it", file=sys.stderr)
    if out:
        out.close()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
