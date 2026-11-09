"""One compact table from sweep.py's --jsonl records: per map and agent count, each planner's solved / tried, CBS's
median time and nodes over the tries it solved, and prioritized planning's SOC over CBS's on the instances both
solve (DESIGN.md "Measurements (day 4)").

    .venv/bin/python scripts/sweep_summary.py results/day4/sweep.jsonl [more.jsonl ...]

Rows come in the order the records do. A planner's second try on the same instance (map, agents, scenario) is an
error: which one to count would be ambiguous. A planner sweep.py stopped trying (it solved none at a smaller count)
shows "-". Needs no build: it reads only the JSON records.
"""

from __future__ import annotations

import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def duration(seconds: float) -> str:
    return f"{seconds * 1000:.2f} ms" if seconds < 1 else f"{seconds:.2f} s"


def summarize(records: list[dict]) -> list[str]:
    tries: dict[tuple[str, int], dict[str, list[dict]]] = defaultdict(lambda: defaultdict(list))
    seen = set()
    for r in records:
        key = (r["map"], r["agents"], r["planner"], r["scenario"])
        if key in seen:
            # Two tries of one planner on one instance (a re-run, or files that overlap): which to count, and which
            # to pair with the other planner's, is ambiguous, so refuse (the day-4 review's second round: the solved
            # count kept one while the tried count kept both).
            raise ValueError(f"a repeated try: map {key[0]}, {key[1]} agents, {key[2]}, scenario {key[3]}")
        seen.add(key)
        tries[(r["map"], r["agents"])][r["planner"]].append(r)
    lines = [
        "| map | agents | PP solved | CBS solved | CBS median time, solved | CBS median nodes, solved "
        "| PP SOC over CBS's, both solved (mean, max) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for (name, agents), by in tries.items():
        cells = [name, str(agents)]
        solved = {p: {r["scenario"]: r for r in by.get(p, []) if r["status"] == "SOLVED"} for p in ("pp", "cbs")}
        for p in ("pp", "cbs"):
            cells.append(f"{len(solved[p])} / {len(by[p])}" if by.get(p) else "-")
        if solved["cbs"]:
            cells.append(duration(statistics.median(r["seconds"] for r in solved["cbs"].values())))
            cells.append(f"{statistics.median(r['nodes_generated'] for r in solved['cbs'].values()):,.0f}")
        else:
            cells += ["-", "-"]
        both = sorted(solved["pp"].keys() & solved["cbs"].keys())
        if both:
            socs = [(solved["pp"][n]["sum_of_costs"], solved["cbs"][n]["sum_of_costs"]) for n in both]
            ratios = [pp / cbs if cbs else 1.0 for pp, cbs in socs]  # both 0: every agent starts on its goal
            cells.append(f"+{(statistics.mean(ratios) - 1) * 100:.1f}%, +{(max(ratios) - 1) * 100:.1f}% ({len(both)})")
        else:
            cells.append("-")
        lines.append("| " + " | ".join(cells) + " |")
    return lines


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__.split("\n\n")[1].strip(), file=sys.stderr)
        return 2
    records = [json.loads(line) for path in argv for line in Path(path).read_text().splitlines() if line]
    try:
        print("\n".join(summarize(records)))
    except ValueError as e:
        print(f"sweep_summary.py: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
