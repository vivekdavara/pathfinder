"""The day-1 review (commit 5ef5763): each test failed on that commit and passes with the fix that names it."""

import importlib.util
import io
import os
import zipfile
from pathlib import Path

import pytest

import pathfinder as pf

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[3]))


def test_an_instances_grid_cant_be_changed_after_the_rules_were_checked():
    # python/bindings.cpp: Instance's docstring says "the constructor enforces DESIGN.md's rules", but def_readonly
    # hands out a reference to the C++ grid and pybind11 ignores const, so set_passable on it blocks a checked start.
    inst = pf.Instance(pf.Grid(3, 1), [pf.Agent(0, 2)])
    try:
        inst.grid.set_passable(0, 0, False)
    except (AttributeError, TypeError, ValueError):
        pass
    assert inst.grid.passable(0, 0), "the instance's start was blocked after check_instance passed"


def test_make_instance_refuses_a_point_outside_the_map_from_python():
    # src/instance.cpp make_instance: (4, 0) on a 4x4 map silently becomes cell 4, (0, 1).
    g = pf.Grid(4, 4)
    s = pf.Scenario([pf.ScenarioEntry(0, "m.map", 4, 4, pf.Point(4, 0), pf.Point(3, 3), 1.0)])
    with pytest.raises(ValueError):
        pf.make_instance(g, s, 1)


def test_points_and_grids_compare_unequal_to_other_types():
    # python/bindings.cpp: __eq__ bound as a plain method raises TypeError for a non-Point instead of returning
    # NotImplemented, so `p == None`, `p in [None, p]` and mixed-key lookups raise.
    p = pf.Point(1, 2)
    assert (p == None) is False  # noqa: E711
    assert p in [None, p]
    assert (pf.Grid(2, 2) == "grid") is False


def load_fetch_maps():
    spec = importlib.util.spec_from_file_location("fetch_maps_under_review", ROOT / "scripts" / "fetch_maps.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_fetch_maps_refuses_nested_map_members_instead_of_flattening_them():
    # scripts/fetch_maps.py members(): a map-archive member in a subdirectory is "unexpected" (the real archive's are
    # all at its top level), but it is flattened to mapf-map/<name>, so two members with the same name in different
    # directories are both accepted and the second silently overwrites the first.
    fetch = load_fetch_maps()
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("a/x.map", "type octile\nheight 1\nwidth 1\nmap\n.\n")
        z.writestr("b/x.map", "type octile\nheight 1\nwidth 1\nmap\n@\n")
    buf.seek(0)
    with zipfile.ZipFile(buf) as z, pytest.raises(SystemExit):
        fetch.members(z, "mapf-map", ".map")


def test_fetch_maps_refuses_duplicate_members():
    fetch = load_fetch_maps()
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("x.map", "type octile\nheight 1\nwidth 1\nmap\n.\n")
        z.writestr("mapf-map/x.map", "type octile\nheight 1\nwidth 1\nmap\n@\n")
    buf.seek(0)
    with zipfile.ZipFile(buf) as z, pytest.raises(SystemExit, match="would land on the same file"):
        fetch.members(z, "mapf-map", ".map")
