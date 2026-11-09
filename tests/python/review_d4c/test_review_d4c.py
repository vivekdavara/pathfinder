"""The day-4 third-round review (commit dbd9dad, reviewing c75fdf5..dbd9dad in scripts/): each test failed there."""

import importlib.util
import io
import os
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import pytest

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[3]))


def load_script(name):
    spec = importlib.util.spec_from_file_location(f"{name}_under_review_d4c", ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("limit", ["0", "inf"])
def test_delays_refuses_no_time_limit(limit):
    """4729f63 made sweep.py refuse a --time-limit of 0 or >= 1e9 (both mean "no limit" to the planners, and CBS on
    an unsolvable instance whose goals are reachable never stops), but delays.py passes the same flag to the same
    pf.cbs unchecked. Input: delays.py --time-limit 0 (and inf) on a 4-agent generated-warehouse instance (solvable,
    so the run here ends). Wrong (at dbd9dad): it runs with no limit and exits 0. Right: an argparse error (exit 2),
    as sweep.py. Fix: the same `0 < t < 1e9` check in delays.py."""
    delays = load_script("delays")
    with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), pytest.raises(SystemExit) as stop:
        delays.main(["--map", "generated-warehouse", "--agents", "4", "--scenarios", "1", "--probabilities", "0",
                     "--seeds", "1", "--single", "--time-limit", limit])
    assert stop.value.code == 2


def test_sweep_output_with_a_repeated_map_is_summarizable(tmp_path):
    """sweep.py now sorts and dedupes --agents, but not --maps: `--maps generated-warehouse generated-warehouse`
    runs the map twice and writes every try twice to --jsonl, and since 4729f63 sweep_summary.py refuses a repeated
    try. Input: that sweep, 2 agents, scenario 1, then sweep_summary.py on its JSONL. Wrong (at dbd9dad): exit 1
    ("a repeated try"), so sweep.py's own output can't be summarized. Right: exit 0 (each map run once). Fix: dedupe
    --maps in sweep.py keeping the order (dict.fromkeys), or refuse a repeated map."""
    sweep = load_script("sweep")
    summary = load_script("sweep_summary")
    out = tmp_path / "sweep.jsonl"
    with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
        assert sweep.main(["--maps", "generated-warehouse", "generated-warehouse", "--agents", "2", "--scenarios",
                           "1", "--time-limit", "5", "--jsonl", str(out)]) == 0
        assert summary.main([str(out)]) == 0
