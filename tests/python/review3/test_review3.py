"""The day-1 third-round review (commit e77c300, reviewing 8b29b30..e77c300): each test failed on that commit."""

import hashlib
import importlib.util
import io
import os
import sys
import zipfile
from pathlib import Path

import pytest

import pathfinder as pf

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[3]))
MAP_A = "type octile\nheight 1\nwidth 1\nmap\n.\n"
MAP_B = "type octile\nheight 1\nwidth 1\nmap\n@\n"

# Two spellings APFS stores as one file (checked on this Mac: write the first, the second then exists), which the
# key unicodedata.normalize("NFC", s).casefold() tells apart: casefold() of U+0390 gives iota + U+0308 + U+0301,
# which is not NFC (NFC would be U+03CA + U+0301, what the second spelling's key is). Normalising after case folding
# (or comparing NFD(casefold(NFD(s))), Unicode's canonical caseless match) would catch it.
GREEK_A = "ΐ.map"  # GREEK SMALL LETTER IOTA WITH DIALYTIKA AND TONOS
GREEK_B = "Ϊ́.map"  # capital iota + combining diaeresis + combining acute


def load_fetch_maps():
    spec = importlib.util.spec_from_file_location("fetch_maps_under_review3", ROOT / "scripts" / "fetch_maps.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def archive(names):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for name, text in zip(names, [MAP_A, MAP_B]):
            z.writestr(name, text)
    return buf.getvalue()


# scripts/fetch_maps.py members(): the duplicate key normalises before case folding, and casefold() can produce a
# string that isn't NFC, so two names that are one file on APFS get different keys and both pass.
def test_fetch_maps_refuses_members_that_differ_only_in_case_after_normalising():
    fetch = load_fetch_maps()
    with zipfile.ZipFile(io.BytesIO(archive([GREEK_A, GREEK_B]))) as z, pytest.raises(SystemExit):
        fetch.members(z, "mapf-map", ".map")


def test_fetch_maps_doesnt_unpack_two_greek_spellings_onto_one_file(tmp_path, monkeypatch):
    probe = tmp_path / "probe"
    probe.mkdir()
    (probe / GREEK_A).write_text("1")
    if not (probe / GREEK_B).exists():
        pytest.skip("this file system keeps the two spellings apart")
    fetch = load_fetch_maps()
    data = tmp_path / "data"
    (data / "downloads").mkdir(parents=True)
    blob = archive([GREEK_A, GREEK_B])
    (data / "downloads" / "mapf-map.zip").write_bytes(blob)
    monkeypatch.setattr(
        fetch, "ARCHIVES", {"mapf-map.zip": (hashlib.sha256(blob).hexdigest(), len(blob), "mapf-map", ".map")}
    )
    monkeypatch.setattr(sys, "argv", ["fetch_maps.py", "--data", str(data)])
    try:
        fetch.main()
    except SystemExit:
        return  # refusing the archive is the fix
    unpacked = sorted(p.name for p in (data / "mapf-map").iterdir())
    assert len(unpacked) == 2, f"2 members unpacked into {[ascii(n) for n in unpacked]}: one overwrote the other"


# python/bindings.cpp inside_or_throw(): the IndexError's message is built with py::str(x), and Python (3.11+)
# refuses to turn an int of more than 4,300 digits into a string, so set_passable and id raise ValueError for such a
# coordinate instead of the documented IndexError (passable is fine: it never builds the message).
@pytest.mark.parametrize("call", ["id", "set_passable"])
def test_a_coordinate_too_long_to_print_is_still_outside_the_grid(call):
    g = pf.Grid(5, 4)
    args = (10**5000, 0) if call == "id" else (10**5000, 0, False)
    with pytest.raises(IndexError):
        getattr(g, call)(*args)


class Index:
    """An integer-like that isn't an int (numpy.int64 is the common one): it has __index__."""

    def __init__(self, value):
        self.value = value

    def __index__(self):
        return self.value


# python/bindings.cpp: passable/set_passable/id now take py::int_, whose caster accepts only int instances; the
# std::int32_t parameters they had (and that Point, distances and every other binding still have) take anything with
# __index__, numpy integers included. So g.passable(np.int64(3), np.int64(2)) went from working to a TypeError, while
# pf.Point(np.int64(3), np.int64(2)) still works.
def test_coordinates_still_take_integer_likes_as_the_other_bindings_do():
    g = pf.Grid(5, 4)
    assert pf.Point(Index(3), Index(2)) == pf.Point(3, 2)  # the int32 parameters accept it
    assert g.passable(Index(3), Index(2))
    assert g.id(Index(3), Index(2)) == 13
    g.set_passable(Index(3), Index(2), False)
    assert not g.passable(3, 2)
