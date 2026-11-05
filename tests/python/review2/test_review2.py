"""The day-1 second-round review (commit 8b29b30): each test failed on that commit and passes with its fix."""

import importlib.util
import io
import os
import sys
import unicodedata
import zipfile
from pathlib import Path

import pytest

ROOT = Path(os.environ.get("PATHFINDER_SOURCE_DIR", Path(__file__).resolve().parents[3]))
MAP_A = "type octile\nheight 1\nwidth 1\nmap\n.\n"
MAP_B = "type octile\nheight 1\nwidth 1\nmap\n@\n"


def load_fetch_maps():
    spec = importlib.util.spec_from_file_location("fetch_maps_under_review2", ROOT / "scripts" / "fetch_maps.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def archive(names):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for name, text in zip(names, [MAP_A, MAP_B]):
            z.writestr(name, text)
    return buf.getvalue()


# scripts/fetch_maps.py members(): the duplicate check compares PurePosixPaths exactly, but data/ is on a
# case-insensitive, normalisation-insensitive file system on macOS (APFS's default, where this project is developed),
# so two members that differ only in case or in Unicode normalisation pass the check and land on the same file: the
# second silently overwrites the first, which is what the day-1 fix ("a duplicate stops it too") set out to prevent.
# (The pinned archives have no such pair: checked, 33 + 825 ASCII names, no casefold collisions.)
@pytest.mark.parametrize(
    "names",
    [
        ["x.map", "X.map"],
        ["x.map", "mapf-map/X.map"],
        [unicodedata.normalize("NFC", "café.map"), unicodedata.normalize("NFD", "café.map")],
    ],
    ids=["case", "case-across-layouts", "nfc-nfd"],
)
def test_fetch_maps_refuses_members_that_would_land_on_the_same_file(names):
    fetch = load_fetch_maps()
    with zipfile.ZipFile(io.BytesIO(archive(names))) as z, pytest.raises(SystemExit):
        fetch.members(z, "mapf-map", ".map")


def test_fetch_maps_doesnt_unpack_two_members_onto_one_file(tmp_path, monkeypatch):
    # End to end through main(), offline: the pinned digest is pointed at a crafted archive already in downloads/.
    probe = tmp_path / "probe"
    probe.mkdir()
    (probe / "a").write_text("1")
    if not (probe / "A").exists():
        pytest.skip("this file system is case-sensitive; x.map and X.map are two files here")
    fetch = load_fetch_maps()
    data = tmp_path / "data"
    (data / "downloads").mkdir(parents=True)
    blob = archive(["x.map", "X.map"])
    (data / "downloads" / "mapf-map.zip").write_bytes(blob)
    import hashlib

    monkeypatch.setattr(
        fetch, "ARCHIVES", {"mapf-map.zip": (hashlib.sha256(blob).hexdigest(), len(blob), "mapf-map", ".map")}
    )
    monkeypatch.setattr(sys, "argv", ["fetch_maps.py", "--data", str(data)])
    try:
        fetch.main()
    except SystemExit:
        return  # refusing the archive is the fix
    unpacked = sorted(p.name for p in (data / "mapf-map").iterdir())
    assert len(unpacked) == 2, f"2 members unpacked into {unpacked}: one overwrote the other"
