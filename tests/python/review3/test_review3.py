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
