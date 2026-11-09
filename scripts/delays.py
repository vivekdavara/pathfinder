"""What a plan's lack of slack costs: plans executed by the step-by-step simulator with robots delayed at random
(DESIGN.md "Python and the simulator").

    make maps release
    PYTHONPATH=build/release/python .venv/bin/python scripts/delays.py \\
        --map random-32-32-20 --agents 20 --scenarios 1-25 --probabilities 0.01 0.05 0.1 0.2 --seeds 20

For each scenario (the first k agents; `generated-warehouse` is pf.warehouse_map's default floor with
pf.random_scenario seeds, synthetic) it plans with prioritized planning and with CBS (each within --time-limit; an
instance either doesn't solve is skipped for that planner) and executes every plan many times under random delays:
each robot, at each step, independently delayed with probability p (simulator.random_delays, the same draws for both
policies). Each delay is executed twice:

- as a timetable (every other robot keeps to the plan's clock): the runs with a collision, and collisions per run;
- ordered (each cell entered in the plan's order, waits dropped; never a collision): the SOC above the plan's, per
  robot-step of delay (1.0 would mean each delay costs only the delayed robot its own step; above 1, delays spread to
  other robots; below 1, some are absorbed), and the makespan above the plan's.

Prints a Markdown table, one row per planner and probability. It exits 1 if an ordered run doesn't complete or drives
an invalid plan (the simulator raises on a collision itself).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pathfinder as pf
from pathfinder.simulator import Simulator, random_delays

ROOT = Path(__file__).resolve().parents[1]
GENERATED = "generated-warehouse"


def scenario_numbers(text: str) -> range:
    first, _, last = text.partition("-")
    return range(int(first), int(last or first) + 1)


def instances(name: str, agents: int, numbers: range, data: Path):
    if name == GENERATED:
        grid = pf.warehouse_map(pf.WarehouseLayout())
        for n in numbers:
            yield n, pf.make_instance(grid, pf.random_scenario(grid, agents, n, "warehouse.map"), agents)
        return
    grid = pf.load_map(data / "mapf-map" / f"{name}.map")
    for n in numbers:
        scen = pf.load_scenario(data / "scen-random" / f"{name}-random-{n}.scen")
        yield n, pf.make_instance(grid, scen, agents)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--data", type=Path, default=ROOT / "data")
    p.add_argument("--map", required=True)
    p.add_argument("--agents", type=int, required=True)
    p.add_argument("--scenarios", default="1-25")
    p.add_argument("--probabilities", nargs="+", type=float, default=[0.01, 0.05, 0.1, 0.2])
    p.add_argument("--seeds", type=int, default=20, help="delay seeds per plan and probability")
    p.add_argument("--time-limit", type=float, default=5.0, help="seconds per planner call")
    args = p.parse_args(argv)

    plans: dict[str, list[tuple[pf.Instance, pf.PlanResult]]] = {"pp": [], "cbs": []}
    for n, instance in instances(args.map, args.agents, scenario_numbers(args.scenarios), args.data):
        for name, planner in (("pp", pf.prioritized_planning), ("cbs", pf.cbs)):
            r = planner(instance, time_limit=args.time_limit)
            if r.solved:
                plans[name].append((instance, r))
    print(f"{args.map}, {args.agents} agents, scenarios {args.scenarios}: prioritized planning solved "
          f"{len(plans['pp'])}, CBS {len(plans['cbs'])} (time limit {args.time_limit:g} s); {args.seeds} delay seeds "
          f"per plan and probability\n")
    print("| planner | p | runs | timetable: runs with a collision | collisions per run | ordered: delay "
          "robot-steps per run | extra SOC per delay | extra makespan per run |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|")
    failures = 0
    for name in ("pp", "cbs"):
        for prob in args.probabilities:
            runs = collided = collisions = 0
            delay_steps = extra_soc = extra_makespan = 0
            for instance, result in plans[name]:
                for seed in range(args.seeds):
                    timetable = Simulator(instance, result.plan, policy="timetable").run(random_delays(prob, seed))
                    sim = Simulator(instance, result.plan, policy="ordered")
                    ordered = sim.run(random_delays(prob, seed))
                    if not ordered.completed or not pf.validate(instance, sim.trajectories).valid:
                        print(f"{name}, p = {prob}, seed {seed}: the ordered run failed", file=sys.stderr)
                        failures += 1
                        continue
                    runs += 1
                    collided += bool(timetable.collisions)
                    collisions += len(timetable.collisions)
                    delay_steps += ordered.delay_steps
                    extra_soc += ordered.extra_cost
                    extra_makespan += ordered.makespan - result.makespan
            if not runs:
                print(f"| {name} | {prob:g} | 0 | | | | | |")
                continue
            per_delay = f"{extra_soc / delay_steps:.3f}" if delay_steps else "-"
            print(f"| {name} | {prob:g} | {runs} | {collided / runs:.1%} | {collisions / runs:.2f} | "
                  f"{delay_steps / runs:.2f} | {per_delay} | {extra_makespan / runs:.2f} |", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
