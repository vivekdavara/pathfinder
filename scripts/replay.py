"""Plan one instance, execute the plan under delays with each policy, and write the runs as a replay page (and
optionally replay JSON files and an animated GIF): DESIGN.md "The replay viewer (day 5)".

    PYTHONPATH=build/release/python .venv/bin/python scripts/replay.py --map generated-warehouse --agents 12 \\
        --scenario 3 --delay-probability 0.1 --seed 4 --html replay.html --gif replay.gif

`--map generated-warehouse` is pf.warehouse_map's default floor with a pf.random_scenario seed (synthetic, no
downloads); any other name is a MovingAI benchmark map from `make maps` (data/mapf-map/<name>.map with scenario
data/scen-random/<name>-random-<n>.scen). Both policies (or the ones given) execute the same plan under the same
delays: random_delays draws for every robot at every step, so with one seed both runs see the same delays at the same
steps. Prints one line per run. Exits 1 if the planner doesn't solve the instance.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pathfinder as pf
from pathfinder.replay import record, save, write_html
from pathfinder.simulator import POLICIES, delay_robot, random_delays

ROOT = Path(__file__).resolve().parents[1]
GENERATED = "generated-warehouse"


def load_instance(name: str, scenario: int, agents: int, data: Path) -> tuple[pf.Instance, str]:
    if name == GENERATED:
        grid = pf.warehouse_map(pf.WarehouseLayout())
        return pf.make_instance(grid, pf.random_scenario(grid, agents, scenario, "warehouse.map"), agents), \
            f"{GENERATED} (synthetic), scenario seed {scenario}"
    grid = pf.load_map(data / "mapf-map" / f"{name}.map")
    scen = pf.load_scenario(data / "scen-random" / f"{name}-random-{scenario}.scen")
    return pf.make_instance(grid, scen, agents), f"{name}, random scenario {scenario}"


def positive(text: str) -> int:
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError("must be 1 or more")
    return value


def non_negative(text: str) -> int:
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError("must be 0 or more")
    return value


def probability(text: str) -> float:
    value = float(text)
    if not 0 <= value <= 1:  # also refuses nan
        raise argparse.ArgumentTypeError("must be between 0 and 1")
    return value


def time_limit(text: str) -> float:
    value = float(text)
    if not 0 < value < float("inf"):
        raise argparse.ArgumentTypeError("must be a positive number of seconds")
    return value


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--map", default=GENERATED, help=f"{GENERATED} or a MovingAI map name")
    ap.add_argument("--scenario", type=positive, default=1, help="scenario number (the seed, for the generated floor)")
    ap.add_argument("--agents", type=positive, default=12)
    ap.add_argument("--planner", choices=("cbs", "pp"), default="cbs")
    ap.add_argument("--time-limit", type=time_limit, default=10.0, help="seconds for the planner")
    ap.add_argument("--policies", nargs="+", choices=POLICIES, default=list(POLICIES))
    delays = ap.add_mutually_exclusive_group()
    delays.add_argument("--delay-probability", type=probability, default=0.05,
                        help="each robot delayed at each step with this probability (default 0.05)")
    delays.add_argument("--delay-robot", type=non_negative, metavar="ROBOT",
                        help="instead: delay one robot, --at and --steps")
    ap.add_argument("--at", type=non_negative, default=0, help="with --delay-robot: the first step delayed")
    ap.add_argument("--steps", type=non_negative, default=1, help="with --delay-robot: how many steps")
    ap.add_argument("--seed", type=int, default=1, help="the random delays' seed")
    ap.add_argument("--html", type=Path, help="write the replay page here")
    ap.add_argument("--json", type=Path, metavar="PREFIX", help="write each run to PREFIX-<policy>.json")
    ap.add_argument("--gif", type=Path, help="write an animated GIF of the runs side by side (needs Pillow)")
    ap.add_argument("--gif-cell", type=positive, default=14, help="GIF pixels per cell")
    ap.add_argument("--gif-substeps", type=positive, default=4, help="GIF frames per time step")
    ap.add_argument("--data", type=Path, default=ROOT / "data")
    args = ap.parse_args(argv)
    if len(set(args.policies)) != len(args.policies):
        ap.error("a policy is given twice")
    if not (args.html or args.json or args.gif):
        ap.error("nothing to write: give --html, --json or --gif")

    instance, where = load_instance(args.map, args.scenario, args.agents, args.data)
    if args.delay_robot is not None and args.delay_robot >= len(instance.agents):
        ap.error(f"--delay-robot {args.delay_robot}: there are {len(instance.agents)} robots")
    if args.planner == "cbs":
        result = pf.cbs(instance, time_limit=args.time_limit)
    else:
        result = pf.prioritized_planning(instance, time_limit=args.time_limit)
    if not result.solved:
        print(f"{args.planner} didn't solve {where} with {args.agents} agents: {result.status}", file=sys.stderr)
        return 1
    print(f"{where}: {args.agents} robots, {args.planner} plan SOC {result.sum_of_costs}, makespan {result.makespan}")

    if args.delay_robot is not None:
        how = f"robot {args.delay_robot} delayed {args.steps} steps from t = {args.at}"
    else:
        how = f"each robot delayed with probability {args.delay_probability} per step (seed {args.seed})"
    replays = []
    for policy in args.policies:
        schedule = (delay_robot(args.delay_robot, args.at, args.steps) if args.delay_robot is not None
                    else random_delays(args.delay_probability, args.seed))
        replay = record(instance, result.plan, policy, schedule, planner=args.planner,
                        title=f"{args.agents} robots, {how}", map_name=where)
        s = replay["summary"]
        outcome = (f"SOC {s['sum_of_costs']} (+{s['sum_of_costs'] - s['planned_sum_of_costs']}), makespan "
                   f"{s['makespan']}" if s["completed"] else f"stopped after {s['steps']} steps")
        print(f"  {policy:9}: {outcome}, {s['delay_steps']} robot-steps of delay, {s['collisions']} collisions")
        replays.append(replay)

    if args.json:
        for replay in replays:
            path = args.json.with_name(f"{args.json.name}-{replay['policy']}.json")
            save(replay, path)
            print(f"wrote {path}")
    if args.html:
        write_html(replays, args.html)
        print(f"wrote {args.html}")
    if args.gif:
        from pathfinder.render import save_gif  # Pillow only when a GIF is asked for

        frames = save_gif(replays, args.gif, cell=args.gif_cell, substeps=args.gif_substeps)
        print(f"wrote {args.gif} ({frames} frames)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
