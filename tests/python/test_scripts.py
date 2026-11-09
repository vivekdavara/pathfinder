"""The experiment scripts' own arithmetic (scripts/sweep.py, scripts/delays.py), on cases worked out by hand. CTest
also runs both scripts end to end on the generated floor (python.sweep, python.delays)."""

import importlib.util
import os
from pathlib import Path

import pytest

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[2]))


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_following_moves():
    delays = load("delays")
    # Robot 0 follows robot 1 down a corridor: it moves into robot 1's cell as robot 1 moves out, three times.
    assert delays.following_moves([[0, 1, 2, 3], [1, 2, 3, 4]]) == 3
    # The same hand-off a step apart isn't following.
    assert delays.following_moves([[0, 0, 1], [1, 2, 2]]) == 0
    # A rotation of four on a 2 x 2 grid: every robot follows another.
    assert delays.following_moves([[0, 1], [1, 3], [3, 2], [2, 0]]) == 4
    # Paths of different lengths: the short one sits on its last cell, so nothing moves into it.
    assert delays.following_moves([[5], [4, 3, 2]]) == 0


@pytest.mark.parametrize("module", ["sweep", "delays"])
def test_scenario_numbers(module):
    numbers = load(module).scenario_numbers
    assert list(numbers("1-5")) == [1, 2, 3, 4, 5]
    assert list(numbers("7")) == [7]
    assert list(numbers("3-2")) == []
