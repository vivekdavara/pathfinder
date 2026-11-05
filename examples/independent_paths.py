"""Why MAPF needs a planner: give every agent its own shortest path, ignoring the others, and let the validator count
what goes wrong.

    PYTHONPATH=build/dev/python .venv/bin/python examples/independent_paths.py --agents 40 --seed 7

On a generated warehouse floor (synthetic) it draws a scenario, walks each agent down its BFS distances from start to
goal (a shortest path for that agent alone), writes the map, scenario and plan files to a temporary directory and
reads them back, and validates the plan. Every path is legal and as short as possible, so the plan's sum of costs is
the lower bound conflict-based search starts from; the conflicts are what the planners of days 2 and 3 must remove.
It exits 1 if anything it checks along the way is wrong, so CTest runs it.
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from collections import Counter
from pathlib import Path

import pathfinder as pf


def shortest_path(grid: pf.Grid, start: int, goal: int) -> list[int]:
    """One agent's shortest path: from start, always step to the first neighbour (north, east, south, west) one closer
    to the goal by BFS distance."""
    to_goal = pf.distances(grid, goal)
    path = [start]
    while path[-1] != goal:
        here = path[-1]
        path.append(next(n for n in grid.neighbors(here) if to_goal[n] == to_goal[here] - 1))
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--agents", type=int, default=40)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    grid = pf.warehouse_map(pf.WarehouseLayout())
    scenario = pf.random_scenario(grid, args.agents, args.seed, "warehouse.map")
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        (d / "warehouse.map").write_text(pf.write_map(grid))
        (d / "warehouse.scen").write_text(pf.write_scenario(scenario))
        grid = pf.load_map(d / "warehouse.map")
        instance = pf.make_instance(grid, pf.load_scenario(d / "warehouse.scen"), args.agents)
        plan = [shortest_path(grid, a.start, a.goal) for a in instance.agents]
        (d / "independent.plan").write_text(pf.write_plan(plan, grid))
        plan_back = pf.load_plan(d / "independent.plan", grid)
    if plan_back != plan:
        print("the plan didn't read back as written")
        return 1

    check = pf.validate(instance, plan_back)
    distances = [pf.distances(grid, a.start)[a.goal] for a in instance.agents]
    if check.costs != distances:
        print("an independent path isn't a shortest path (or isn't legal)")
        return 1
    kinds = Counter(p.kind for p in check.problems)
    print(f"{args.agents} agents on a {grid.width}x{grid.height} warehouse floor ({grid.passable_count} free cells)")
    print(f"independent shortest paths: sum of costs {check.sum_of_costs} (a lower bound), makespan {check.makespan}")
    print(f"conflicts: {kinds[pf.ProblemKind.VERTEX_CONFLICT]} vertex, {kinds[pf.ProblemKind.EDGE_CONFLICT]} swap")
    for problem in check.problems[:3]:
        print("  " + pf.describe(problem, grid))
    if any(k not in (pf.ProblemKind.VERTEX_CONFLICT, pf.ProblemKind.EDGE_CONFLICT) for k in kinds):
        print("a problem other than a conflict")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
