#!/usr/bin/env python3
"""Compare Horizon CAD's reading of the STEP corpus with OpenCASCADE's.

    occt_check.py check <corpus dir> <summary.json> [--out occt.json]
    occt_check.py merge <occt.json> <manifest.json>

check (needs OpenCASCADE's Python binding, OCP: `pip install cadquery-ocp`)
reads every file of the corpus with OpenCASCADE, an independent reader, and
compares what it finds with what hz_step_acceptance wrote in summary.json:

- the number of solids, the volume and the bounding box of the original
  file, against Horizon CAD's bodies, volume (on its ideal surfaces) and
  bounds, within the manifest's tolerances;
- Horizon CAD's export of it: OpenCASCADE must read it, find it valid, with
  as many solids, and a volume between Horizon CAD's faceted and ideal ones.

A file the manifest lists as not read, or a check its known_gaps name
("volume", "bounds", "bodies", "export", "export-valid", "export-bodies",
"export-volume", "occt"), each with why, is reported and not failed. Exits 1 on any other mismatch. --out writes what
OpenCASCADE found, for merge.

merge (plain Python) writes what OpenCASCADE found into the manifest's
expect fields (bodies, volume, bounds), where the acceptance test holds every
build to them. Run it when a file is added, with the occt.json a CI run kept.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

VOLUME_TOLERANCE = 1e-3  # relative, when the manifest gives none
BOUNDS_TOLERANCE = 0.005  # of the box's diagonal, when the manifest gives none
EXPORT_SLACK = 1e-3  # relative, around Horizon CAD's faceted..ideal volumes


@dataclass
class Reading:
    """What OpenCASCADE found in one file."""

    solids: int
    volume: float
    bounds: list[list[float]] | None
    valid: bool


def static(owner: Any, name: str) -> Any:
    """A static method of an OCP class: OCP before 8 names it name_s."""
    return getattr(owner, name + "_s", None) or getattr(owner, name)


def occt_read(path: Path) -> Reading | None:
    """Read @path with OpenCASCADE, in millimetres; None when it cannot."""
    # Imported here: `merge` needs no OpenCASCADE.
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer

    static(Interface_Static, "SetCVal")("xstep.cascade.unit", "MM")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        return None
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        return None
    solids = 0
    volume = 0.0
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        props = GProp_GProps()
        static(BRepGProp, "VolumeProperties")(explorer.Current(), props)
        volume += abs(props.Mass())
        solids += 1
        explorer.Next()
    box = Bnd_Box()
    static(BRepBndLib, "AddOptimal")(shape, box, False, False)
    bounds = None
    if not box.IsVoid():
        # CornerMin/CornerMax: OpenCASCADE 8's Get() returns a struct OCP
        # does not expose.
        low, high = box.CornerMin(), box.CornerMax()
        bounds = [[low.X(), low.Y(), low.Z()], [high.X(), high.Y(), high.Z()]]
    return Reading(solids, volume, bounds, BRepCheck_Analyzer(shape).IsValid())


def near(value: float, expected: float, relative: float) -> bool:
    return abs(value - expected) <= relative * max(abs(expected), 1e-9)


def bounds_off(ours: list[list[float]], theirs: list[list[float]], fraction: float) -> float | None:
    """The largest corner difference, when over @fraction of the diagonal."""
    diagonal = math.dist(theirs[0], theirs[1])
    worst = max(abs(ours[c][a] - theirs[c][a]) for c in range(2) for a in range(3))
    return worst if worst > fraction * diagonal else None


def check(corpus: Path, summary_path: Path, out: Path | None) -> int:
    manifest = json.loads((corpus / "manifest.json").read_text(encoding="utf-8"))
    entries = {entry["file"]: entry for entry in manifest["files"]}
    summary = {s["file"]: s for s in json.loads(summary_path.read_text(encoding="utf-8"))["files"]}
    found: dict[str, Any] = {}
    rows = ["| File | OpenCASCADE | Horizon CAD | Export | Result |", "|---|---|---|---|---|"]
    failures = 0
    for name, entry in entries.items():
        ours = summary.get(name)
        # Each check that fails, by its name: a known gap names it too.
        known: dict[str, str] = entry.get("known_gaps", {})
        problems: dict[str, str] = {}
        checked: set[str] = set()  # the checks made, to tell a gap that closed
        theirs = occt_read(corpus / name)
        if theirs is None:
            problems["occt"] = "OpenCASCADE cannot read it"
        else:
            found[name] = {"solids": theirs.solids, "volume": theirs.volume,
                           "bounds": theirs.bounds, "valid": theirs.valid}
        if ours is None:
            problems["summary"] = "not in summary.json"
        elif "not_read" in entry:
            pass  # a known gap: the acceptance test holds it
        elif ours["error"]:
            problems["read"] = f"Horizon CAD did not read it: {ours['error']}"
        elif theirs is not None:
            checked.update({"bodies", "volume", "bounds"})
            expect = entry.get("expect", {})
            if ours["bodies"] != theirs.solids:
                problems["bodies"] = f"bodies {ours['bodies']}, OpenCASCADE {theirs.solids}"
            tolerance = expect.get("volume_tolerance", VOLUME_TOLERANCE)
            if not near(ours["idealVolume"], theirs.volume, tolerance):
                problems["volume"] = (f"volume {ours['idealVolume']:.6g}, "
                                      f"OpenCASCADE {theirs.volume:.6g}")
            if ours["bounds"] and theirs.bounds:
                off = bounds_off(ours["bounds"], theirs.bounds,
                                 expect.get("bounds_tolerance", BOUNDS_TOLERANCE))
                if off is not None:
                    problems["bounds"] = f"bounds off by {off:.4g} mm"
        export = "—"
        if ours and ours.get("exportPath") and not ours["error"]:
            checked.update({"export", "export-valid", "export-bodies", "export-volume"})
            exported = occt_read(Path(ours["exportPath"]))
            if exported is None:
                problems["export"] = "OpenCASCADE cannot read the export"
                export = "unreadable"
            else:
                low = min(ours["facetedVolume"], ours["idealVolume"]) * (1 - EXPORT_SLACK)
                high = max(ours["facetedVolume"], ours["idealVolume"]) * (1 + EXPORT_SLACK)
                export = f"{exported.solids} solids, {exported.volume:.6g} mm³"
                if not exported.valid:
                    problems["export-valid"] = "the export is not a valid shape to OpenCASCADE"
                if exported.solids != ours["bodies"]:
                    problems["export-bodies"] = f"the export has {exported.solids} solids"
                if not low <= exported.volume <= high:
                    problems["export-volume"] = (f"the export's volume {exported.volume:.6g} "
                                                 f"is outside {low:.6g}..{high:.6g}")
        unexpected = {key: why for key, why in problems.items() if key not in known}
        # A known gap that no longer fails: the manifest is to say so.
        for key in sorted(known.keys() & checked - problems.keys()):
            unexpected[key] = f"{key} is as OpenCASCADE reads it now: take it out of known_gaps"
        result = "ok"
        if unexpected:
            result = "**" + "; ".join(unexpected.values()) + "**"
            failures += 1
        elif problems:
            result = "known gap: " + "; ".join(f"{problems[key]} ({known[key]})" for key in problems)
        occt = f"{theirs.solids} solids, {theirs.volume:.6g} mm³" if theirs else "unreadable"
        mine = (f"{ours['bodies']} bodies, {ours['idealVolume']:.6g} mm³"
                if ours and not ours["error"] else "not read")
        rows.append(f"| {name} | {occt} | {mine} | {export} | {result} |")
    table = "\n".join(rows)
    print(table)
    step_summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if step_summary:
        with open(step_summary, "a", encoding="utf-8") as handle:
            handle.write("## STEP corpus against OpenCASCADE\n\n" + table + "\n")
    if out:
        out.write_text(json.dumps({"files": found}, indent=2) + "\n", encoding="utf-8")
    if failures:
        print(f"{failures} file(s) disagree with OpenCASCADE", file=sys.stderr)
    return 1 if failures else 0


def merge(occt_path: Path, manifest_path: Path) -> int:
    found = json.loads(occt_path.read_text(encoding="utf-8"))["files"]
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for entry in manifest["files"]:
        reading = found.get(entry["file"])
        if reading is None or "not_read" in entry:
            continue
        expect = entry.setdefault("expect", {})
        expect["bodies"] = reading["solids"]
        expect["volume"] = round(reading["volume"], 6)
        if reading["bounds"]:
            expect["bounds"] = [[round(v, 6) for v in corner] for corner in reading["bounds"]]
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
                             encoding="utf-8")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("check")
    run.add_argument("corpus", type=Path)
    run.add_argument("summary", type=Path)
    run.add_argument("--out", type=Path)
    combine = commands.add_parser("merge")
    combine.add_argument("occt", type=Path)
    combine.add_argument("manifest", type=Path)
    args = parser.parse_args()
    if args.command == "check":
        return check(args.corpus, args.summary, args.out)
    return merge(args.occt, args.manifest)


if __name__ == "__main__":
    sys.exit(main())
