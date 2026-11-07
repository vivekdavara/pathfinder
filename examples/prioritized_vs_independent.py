"""What prioritized planning buys over independent shortest paths, and what it costs, on the same warehouse floor.

    PYTHONPATH=build/dev/python .venv/bin/python examples/prioritized_vs_independent.py --agents 30 --seed 1

On a generated warehouse floor (synthetic) it draws a scenario and plans it twice: every agent's own shortest path,
ignoring the others (day 1's example: the sum of costs is the lower bound, and the plan has conflicts), and
prioritized planning (day 2: no conflicts, a higher sum of costs). Then it tries every agent count from 1 up and
reports the first that prioritized planning can't solve in the scenario's order, and whether the reverse order
solves it. It exits 1 if anything it checks is wrong (a prioritized plan the validator rejects, a sum of costs below
the lower bound), so CTest runs it.
"""

from __future__ import annotations

import argparse
import sys

import pathfinder as pf


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--agents", type=int, default=30)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    grid = pf.warehouse_map(pf.WarehouseLayout())
    print(f"{args.agents} agents on a {grid.width}x{grid.height} warehouse floor ({grid.passable_count} free cells), "
          f"seed {args.seed}")
    instance = pf.make_instance(grid, pf.random_scenario(grid, args.agents, args.seed, "warehouse.map"), args.agents)

    # Independent shortest paths: space-time A* with an empty table is a shortest path for each agent alone.
    empty = pf.ReservationTable(grid)
    independent = [pf.space_time_astar(grid, a.start, a.goal, empty).path for a in instance.agents]
    alone = pf.validate(instance, independent)
    conflicts = len(alone.problems)
    print(f"independent:  sum of costs {alone.sum_of_costs} (the lower bound), makespan {alone.makespan}, "
          f"{conflicts} conflicts")

    r = pf.prioritized_planning(instance)
    if not r.solved:
        print(f"prioritized:  not solved ({r.status.name}, agent {r.failed_agent}); try fewer agents")
        return 1
    check = pf.validate(instance, r.plan)
    if not check.valid or check.sum_of_costs != r.sum_of_costs or r.sum_of_costs < alone.sum_of_costs:
        print("prioritized planning's plan doesn't check out")
        return 1
    extra = r.sum_of_costs - alone.sum_of_costs
    print(f"prioritized:  sum of costs {r.sum_of_costs} (+{extra}, {100 * extra / max(1, alone.sum_of_costs):.0f}% "
          f"over the bound), makespan {r.makespan}, 0 conflicts, {r.expansions} expansions")

    # Where it stops: the first agent count of this scenario it can't solve in order (random_scenario draws a new
    # scenario for every count, so draw the largest once and take its first k agents).
    most = grid.passable_count // 2
    scenario = pf.random_scenario(grid, most, args.seed, "warehouse.map")
    for k in range(1, most + 1):
        inst = pf.make_instance(grid, scenario, k)
        if pf.prioritized_planning(inst).solved:
            continue
        reverse = pf.prioritized_planning(inst, order=list(reversed(range(k))))
        print(f"first failure: {k} agents in the scenario's order; the reverse order "
              f"{'solves it' if reverse.solved else 'fails too'}")
        if reverse.solved and not pf.validate(inst, reverse.plan).valid:
            print("the reverse order's plan doesn't check out")
            return 1
        break
    else:
        print(f"solved every count up to {most} agents")
    return 0


if __name__ == "__main__":
    sys.exit(main())
