#!/usr/bin/env python3
"""Downloads the MovingAI MAPF benchmark maps and random scenarios into data/ (not committed; see DESIGN.md).

    python3 -I scripts/fetch_maps.py            # data/mapf-map/*.map, data/scen-random/*.scen
    python3 -I scripts/fetch_maps.py --check    # only verify what's already there

The archives come from movingai.com/benchmarks/mapf (Stern et al., SoCS 2019). Each is checked against the SHA-256
pinned below before anything is unpacked; only .map and .scen files are written, each to a plain path under data/,
and a mismatch, an unexpected member or a path that would leave data/ stops the script. Standard library only.
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path, PurePosixPath

BASE_URL = "https://movingai.com/benchmarks/mapf/"

# name: (SHA-256, size in bytes, the directory under data/ it unpacks to, the member suffix it may contain).
# Pinned from the archives downloaded on 2026-11-05 (both last modified 2021-08-05 on the server).
ARCHIVES = {
    "mapf-map.zip": ("9da2e4c5ce03aa4e063b3a283ce874590b36cc4f31a297fe7ecb00d105abf288", 75157, "mapf-map", ".map"),
    "mapf-scen-random.zip": (
        "20b7838f7a51f13e90a63ee138e9435fb4e41b0381becbc7313b7d3a7d859276",
        8308447,
        "scen-random",
        ".scen",
    ),
}

MAX_MEMBER_BYTES = 10 * 1024 * 1024  # the largest member is under 1 MB
MAX_TOTAL_BYTES = 100 * 1024 * 1024  # 39 MB in all


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def download(name: str, dest: Path, expected_size: int) -> None:
    part = dest.with_suffix(dest.suffix + ".part")
    request = urllib.request.Request(BASE_URL + name, headers={"User-Agent": "pathfinder-fetch-maps"})
    with urllib.request.urlopen(request, timeout=60) as response, part.open("wb") as out:
        copied = 0
        while block := response.read(1 << 20):
            copied += len(block)
            if copied > expected_size:
                raise SystemExit(f"{name}: more than the expected {expected_size} bytes; stopping")
            out.write(block)
    part.replace(dest)


def members(archive: zipfile.ZipFile, directory: str, suffix: str) -> list[tuple[zipfile.ZipInfo, PurePosixPath]]:
    """Every file member and where under data/ it goes; refuses anything unexpected."""
    out = []
    seen: set[PurePosixPath] = set()
    total = 0
    for info in archive.infolist():
        if info.is_dir():
            continue
        path = PurePosixPath(info.filename)
        if path.is_absolute() or ".." in path.parts or "\\" in info.filename or not path.name.endswith(suffix):
            raise SystemExit(f"unexpected member {info.filename!r}; stopping")
        if info.external_attr >> 28 == 0o12:  # a symbolic link
            raise SystemExit(f"member {info.filename!r} is a symbolic link; stopping")
        if info.file_size > MAX_MEMBER_BYTES:
            raise SystemExit(f"member {info.filename!r} is {info.file_size} bytes; stopping")
        total += info.file_size
        # The map archive's members are at its top level; the scenario archive's directly under scen-random/.
        # Anything deeper would have to be flattened, and two members could then land on the same file (the day-1
        # review), so it stops instead, as it does for a duplicate.
        if len(path.parts) == 1:
            relative = PurePosixPath(directory) / path.name
        elif len(path.parts) == 2 and path.parts[0] == directory:
            relative = path
        else:
            raise SystemExit(f"unexpected member {info.filename!r} (not at the top level or in {directory}/); stopping")
        if relative in seen:
            raise SystemExit(f"member {info.filename!r} appears twice; stopping")
        seen.add(relative)
        out.append((info, relative))
    if total > MAX_TOTAL_BYTES:
        raise SystemExit(f"the archive unpacks to {total} bytes; stopping")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="verify the downloads and unpacked files only")
    parser.add_argument("--data", type=Path, default=Path(__file__).resolve().parent.parent / "data")
    args = parser.parse_args()
    data: Path = args.data.resolve()
    downloads = data / "downloads"
    downloads.mkdir(parents=True, exist_ok=True)

    for name, (digest, size, directory, suffix) in ARCHIVES.items():
        archive_path = downloads / name
        if archive_path.exists() and sha256(archive_path) == digest:
            print(f"{name}: already downloaded, SHA-256 matches")
        elif args.check:
            print(f"{name}: missing or wrong SHA-256 (run without --check to download)")
            return 1
        else:
            print(f"{name}: downloading {BASE_URL + name} ({size:,} bytes)")
            download(name, archive_path, size)
            actual = sha256(archive_path)
            if actual != digest:
                archive_path.unlink()
                raise SystemExit(f"{name}: SHA-256 {actual}, expected {digest}; deleted it, stopping")

        with zipfile.ZipFile(archive_path) as archive:
            files = members(archive, directory, suffix)
            if args.check:
                missing = [r for info, r in files if not (data / r).is_file() or (data / r).stat().st_size != info.file_size]
                print(f"{name}: {len(files) - len(missing)} of {len(files)} files unpacked")
                if missing:
                    return 1
                continue
            target = data / directory
            if target.exists():
                shutil.rmtree(target)
            target.mkdir(parents=True)
            for info, relative in files:
                out = (data / relative).resolve()
                if out.parent != target.resolve():
                    raise SystemExit(f"member {info.filename!r} would leave {target}; stopping")
                with archive.open(info) as src, out.open("wb") as dst:
                    shutil.copyfileobj(src, dst)
            print(f"{name}: unpacked {len(files)} {suffix} files into {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
