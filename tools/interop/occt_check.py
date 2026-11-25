#!/usr/bin/env python3
"""Compare Horizon CAD's reading of the STEP corpus with OpenCASCADE's.

    occt_check.py check <corpus dir> <summary.json> [--out occt.json] [--kept occt.json]
    occt_check.py merge <occt.json> <manifest.json>

check (needs OpenCASCADE's Python binding, OCP: `pip install cadquery-ocp`)
reads every file of the corpus with OpenCASCADE, an independent reader, and
compares what it finds with what hz_step_acceptance wrote in summary.json:

- the number of solids, the volume and the bounding box of the original
  file, against Horizon CAD's bodies, exact volume (its solids integrated on
  their exact faces, to a millionth of OpenCASCADE's unless the manifest's
  volume_tolerance says otherwise) and bounds;
- Horizon CAD's export of it: OpenCASCADE must read it, find it valid, with
  as many solids, Horizon CAD's exact volume (to the same tolerance), and as
  many faces of each kind of surface as the original. A surface of linear
  extrusion may go out as a B-spline: Horizon CAD has no typed extrusion yet.
  A face with no surface (a tessellated copy beside the part, which Horizon
  CAD reads no body of) is not compared.

A surface model the manifest marks occt_reads_surfaces (its shells close,
but OpenCASCADE makes no solids of them) has its bounds compared, not its
bodies or volume.

A file the manifest lists as not read, or a check its known_gaps name
("volume", "bounds", "bodies", "export", "export-valid", "export-bodies",
"export-volume", "export-kinds", "occt"), each with why, is reported and not
failed; so is a file not read because each solid of it is on its regression
list (exact_failures), which the acceptance test holds. Exits 1 on any other
mismatch. --out writes what OpenCASCADE found, for merge and for --kept.

--kept takes what OpenCASCADE found from the occt.json a run kept, in place
of reading with it: the same comparison on a machine without OCP. Its
readings of the exports are of the exports that run made.

merge (plain Python) writes what OpenCASCADE found into the manifest's
expect fields (bodies, volume, bounds; an occt_reads_surfaces model's bounds
alone), where the acceptance test holds every build to them. Run it when a
file is added, with the occt.json a CI run kept.
"""

# OCP is installed in CI's STEP interop job alone, and imported only where OpenCASCADE reads a
# file: elsewhere its imports cannot be resolved, by design.
# pyright: reportMissingImports=false

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Relative, when the manifest gives no volume_tolerance: Horizon CAD's exact volume
# against OpenCASCADE's reading of the file, and of Horizon CAD's export of it,
# which it integrates to INTEGRATION_EPS.
EXACT_VOLUME_TOLERANCE = 1e-6
BOUNDS_TOLERANCE = 0.005  # of the box's diagonal, when the manifest gives none
# Volumes and areas integrated adaptively, to this relative error. OpenCASCADE's
# default, a Gauss rule of fixed points, measures an analytic surface exactly
# but a rational B-spline one (an export's cylinder) a little out, and a part
# far from the origin further out in volume.
INTEGRATION_EPS = 1e-6
# The kinds of surface (OpenCASCADE's GeomAbs_SurfaceType, without its prefix)
# an export may change: Horizon CAD writes a surface of linear extrusion as
# its B-spline. A face with no surface has no kind ("none").
EXTRUSION = "SurfaceOfExtrusion"
BSPLINE = "BSplineSurface"
NO_SURFACE = "none"


@dataclass
class Reading:
    """What OpenCASCADE found in one file."""

    solids: int
    volume: float
    bounds: list[list[float]] | None
    valid: bool
    # By OpenCASCADE's default fixed Gauss rule; None where not measured.
    fixed_volume: float | None = None
    # Its faces counted by the kind of their surface (Plane, Cylinder,
    # BSplineSurface, SurfaceOfExtrusion, ...; "none" for a face with no
    # surface); None where not counted.
    kinds: dict[str, int] | None = None
    # OpenCASCADE's own estimate of its volume's relative error, the largest
    # of its solids'; None where it gave none.
    volume_error: float | None = None

    def to_json(self) -> dict[str, Any]:
        return {
            "solids": self.solids,
            "volume": self.volume,
            "bounds": self.bounds,
            "valid": self.valid,
            "fixed_volume": self.fixed_volume,
            "kinds": self.kinds,
            "volume_error": self.volume_error,
        }

    @staticmethod
    def from_json(data: dict[str, Any] | None) -> Reading | None:
        """A reading as to_json wrote it; None for none, and for an entry that
        holds the export's reading alone (OpenCASCADE could not read the
        file)."""
        if data is None or "solids" not in data:
            return None
        return Reading(
            solids=int(data["solids"]),
            volume=float(data["volume"]),
            bounds=data.get("bounds"),
            valid=bool(data.get("valid", True)),
            fixed_volume=data.get("fixed_volume"),
            kinds=data.get("kinds"),
            volume_error=data.get("volume_error"),
        )


def static(owner: Any, name: str) -> Any:
    """A static method of an OCP class: OCP before 8 names it name_s."""
    return getattr(owner, name + "_s", None) or getattr(owner, name)


def surface_kind(face: Any) -> str:
    """The kind of @face's surface, as OpenCASCADE names it (GeomAbs_Plane:
    "Plane"); "none" for a face with no surface (a tessellated one)."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface

    # A face with no surface cannot be adapted: OpenCASCADE throws.
    if not static(BRep_Tool, "IsGeometric")(face):
        return "none"
    kind = BRepAdaptor_Surface(face).GetType()
    return getattr(kind, "name", str(kind)).removeprefix("GeomAbs_")


def occt_read(path: Path) -> Reading | None:
    """Read @path with OpenCASCADE, in millimetres; None when it cannot."""
    # Imported here: `merge`, and `check --kept`, need no OpenCASCADE.
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

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
    fixed_volume = 0.0
    volume_error: float | None = None
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        props = GProp_GProps()
        # It returns the relative error it reached.
        reached = static(BRepGProp, "VolumeProperties")(
            explorer.Current(), props, INTEGRATION_EPS
        )
        if isinstance(reached, (int, float)):
            volume_error = max(volume_error or 0.0, abs(float(reached)))
        volume += abs(props.Mass())
        fixed = GProp_GProps()
        static(BRepGProp, "VolumeProperties")(explorer.Current(), fixed)
        fixed_volume += abs(fixed.Mass())
        solids += 1
        explorer.Next()
    kinds: dict[str, int] = {}
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    while faces.More():
        kind = surface_kind(static(TopoDS, "Face")(faces.Current()))
        kinds[kind] = kinds.get(kind, 0) + 1
        faces.Next()
    box = Bnd_Box()
    static(BRepBndLib, "AddOptimal")(shape, box, False, False)
    bounds = None
    if not box.IsVoid():
        # CornerMin/CornerMax: OpenCASCADE 8's Get() returns a struct OCP
        # does not expose.
        low, high = box.CornerMin(), box.CornerMax()
        bounds = [[low.X(), low.Y(), low.Z()], [high.X(), high.Y(), high.Z()]]
    valid = BRepCheck_Analyzer(shape).IsValid()
    return Reading(solids, volume, bounds, valid, fixed_volume, kinds, volume_error)


def occt_faces(path: Path) -> list[dict[str, Any]]:
    """Each face of @path as OpenCASCADE reads it: its surface's kind, area
    and centre, in millimetres; none when it cannot be read."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    static(Interface_Static, "SetCVal")("xstep.cascade.unit", "MM")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        return []
    reader.TransferRoots()
    shape = reader.OneShape()
    faces: list[dict[str, Any]] = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = static(TopoDS, "Face")(explorer.Current())
        props = GProp_GProps()
        static(BRepGProp, "SurfaceProperties")(face, props, INTEGRATION_EPS)
        fixed = GProp_GProps()
        static(BRepGProp, "SurfaceProperties")(face, fixed)
        centre = props.CentreOfMass()
        faces.append(
            {
                "kind": surface_kind(face),
                "area": props.Mass(),
                "fixed area": fixed.Mass(),
                "centre": [centre.X(), centre.Y(), centre.Z()],
            }
        )
        explorer.Next()
    return faces


def face_diff(original: Path, export: Path) -> dict[str, list[dict[str, Any]]]:
    """The faces of @export no face of @original has the area and centre of,
    and the other way round: where an export measures differently."""
    ours, theirs = occt_faces(original), occt_faces(export)

    def unmatched(
        these: list[dict[str, Any]], those: list[dict[str, Any]]
    ) -> list[dict[str, Any]]:
        left = list(those)
        out: list[dict[str, Any]] = []
        for face in these:
            match = next(
                (
                    other
                    for other in left
                    if near(other["area"], face["area"], 1e-4)
                    and math.dist(other["centre"], face["centre"])
                    <= 1e-3 * max(1.0, math.sqrt(face["area"]))
                ),
                None,
            )
            if match is None:
                out.append(face)
            else:
                left.remove(match)
        return out

    return {
        "original only": unmatched(ours, theirs),
        "export only": unmatched(theirs, ours),
    }


def near(value: float, expected: float, relative: float) -> bool:
    return abs(value - expected) <= relative * max(abs(expected), 1e-9)


def bounds_off(
    ours: list[list[float]], theirs: list[list[float]], fraction: float
) -> float | None:
    """The largest corner difference, when over @fraction of the diagonal."""
    diagonal = math.dist(theirs[0], theirs[1])
    worst = max(abs(ours[c][a] - theirs[c][a]) for c in range(2) for a in range(3))
    return worst if worst > fraction * diagonal else None


def kinds_off(original: dict[str, int], export: dict[str, int]) -> str | None:
    """Where @export's faces, counted by the kind of their surface, are not
    @original's; None where they are. A surface of linear extrusion may go out
    as a B-spline: so many fewer extrusions, so many more B-splines. Faces with
    no surface are not compared: a tessellated copy beside the part (CATIA
    writes one), which Horizon CAD reads no body of."""
    moved = original.get(EXTRUSION, 0) - export.get(EXTRUSION, 0)
    off: list[str] = []
    for kind in sorted((original.keys() | export.keys()) - {NO_SURFACE}):
        want = original.get(kind, 0)
        got = export.get(kind, 0)
        if kind == EXTRUSION:
            if got > want:
                off.append(f"{kind} {got}, not {want} or fewer")
            continue
        if kind == BSPLINE:
            want += max(moved, 0)
        if got != want:
            off.append(f"{kind} {got}, not {want}")
    if not off:
        return None
    note = f" ({EXTRUSION} may go out as {BSPLINE})" if original.get(EXTRUSION) else ""
    return "the export's faces by kind: " + "; ".join(off) + note


def volume_off(exact: float, reading: Reading, tolerance: float) -> str | None:
    """@reading's volume, how far from @exact, and OpenCASCADE's own estimate
    of its error, where it is further than @tolerance, relative; None where it
    is not."""
    if near(exact, reading.volume, tolerance):
        return None
    apart = (exact - reading.volume) / reading.volume if reading.volume else math.inf
    own = (
        f", its own error estimate {reading.volume_error:.2g}"
        if reading.volume_error is not None
        else ""
    )
    return f"{reading.volume:.12g}{own}: {apart:+.3g} apart, over {tolerance:g}"


def exact_volume(ours: dict[str, Any]) -> float:
    """Horizon CAD's exact volume of what it read: its exact solids' summed."""
    return sum(s["exactVolume"] for s in ours.get("solids", []) if s["exact"])


def on_regression_list(entry: dict[str, Any], ours: dict[str, Any]) -> bool:
    """Whether Horizon CAD left solids of it out for want of their exact form,
    each on @entry's regression list (exact_failures) with its why."""
    listed: list[dict[str, str]] = entry.get("exact_failures", [])
    failures = [s for s in ours.get("solids", []) if s["exactProblem"]]
    return bool(failures) and all(
        any(
            item["solid"] == s["solid"] and item["why"] in s["exactProblem"]
            for item in listed
        )
        for s in failures
    )


def compare(
    entry: dict[str, Any],
    ours: dict[str, Any] | None,
    theirs: Reading | None,
    exported: Reading | None,
) -> tuple[dict[str, str], set[str]]:
    """One file's checks: each that fails, by name, with what it found; and
    each made, to tell a known gap that closed. @theirs is OpenCASCADE's
    reading of the file, @exported of Horizon CAD's export of it."""
    problems: dict[str, str] = {}
    checked: set[str] = set()
    if theirs is None:
        problems["occt"] = "OpenCASCADE cannot read it"
    if ours is None:
        problems["summary"] = "not in summary.json"
        return problems, checked
    if "not_read" in entry:
        return problems, checked  # a known gap: the acceptance test holds it
    if ours["error"]:
        problems["read"] = f"Horizon CAD did not read it: {ours['error']}"
        return problems, checked
    expect: dict[str, Any] = entry.get("expect", {})
    tolerance: float = expect.get("volume_tolerance", EXACT_VOLUME_TOLERANCE)
    exact = exact_volume(ours)
    if theirs is not None:
        checked.update({"bodies", "volume", "bounds"})
        # A surface model whose shells close: solids to Horizon CAD,
        # surfaces to OpenCASCADE, whose bounds alone are compared.
        surfaces = "occt_reads_surfaces" in entry
        if not surfaces and ours["bodies"] != theirs.solids:
            problems["bodies"] = f"bodies {ours['bodies']}, OpenCASCADE {theirs.solids}"
        off = None if surfaces else volume_off(exact, theirs, tolerance)
        if off is not None:
            problems["volume"] = f"exact volume {exact:.12g}, OpenCASCADE {off}"
        if ours["bounds"] and theirs.bounds:
            off = bounds_off(
                ours["bounds"],
                theirs.bounds,
                expect.get("bounds_tolerance", BOUNDS_TOLERANCE),
            )
            if off is not None:
                problems["bounds"] = f"bounds off by {off:.4g} mm"
    if not ours.get("exportPath"):
        return problems, checked
    checked.update({"export", "export-valid", "export-bodies", "export-volume"})
    if exported is None:
        problems["export"] = "OpenCASCADE cannot read the export"
        return problems, checked
    if not exported.valid:
        problems["export-valid"] = "the export is not a valid shape to OpenCASCADE"
    if exported.solids != ours["bodies"]:
        problems["export-bodies"] = f"the export has {exported.solids} solids"
    off = volume_off(exact, exported, tolerance)
    if off is not None:
        problems["export-volume"] = (
            f"exact volume {exact:.12g}, the export's volume {off}"
        )
    if theirs is not None and theirs.kinds is not None and exported.kinds is not None:
        checked.add("export-kinds")
        off = kinds_off(theirs.kinds, exported.kinds)
        if off is not None:
            problems["export-kinds"] = off
    return problems, checked


def check(
    corpus: Path, summary_path: Path, out: Path | None, kept: Path | None = None
) -> int:
    manifest = json.loads((corpus / "manifest.json").read_text(encoding="utf-8"))
    entries = {entry["file"]: entry for entry in manifest["files"]}
    summary = {
        s["file"]: s
        for s in json.loads(summary_path.read_text(encoding="utf-8"))["files"]
    }
    readings: dict[str, Any] | None = None
    if kept is not None:
        readings = json.loads(kept.read_text(encoding="utf-8"))["files"]
    found: dict[str, Any] = {}
    rows = [
        "| File | OpenCASCADE | Horizon CAD | Export | Result |",
        "|---|---|---|---|---|",
    ]
    failures = 0
    for name, entry in entries.items():
        ours = summary.get(name)
        # Each check that fails, by its name: a known gap names it too.
        known: dict[str, str] = dict(entry.get("known_gaps", {}))
        if ours is not None and ours["error"] and on_regression_list(entry, ours):
            known["read"] = "each solid left out is on its regression list"
        kept_reading: dict[str, Any] = (readings or {}).get(name) or {}
        theirs = (
            occt_read(corpus / name)
            if readings is None
            else Reading.from_json(kept_reading or None)
        )
        exported = None
        if ours and ours.get("exportPath") and not ours["error"]:
            exported = (
                occt_read(Path(ours["exportPath"]))
                if readings is None
                else Reading.from_json(kept_reading.get("export"))
            )
        problems, checked = compare(entry, ours, theirs, exported)
        if theirs is not None:
            found[name] = theirs.to_json()
        if exported is not None:
            found.setdefault(name, {})["export"] = exported.to_json()
        # Where the fixed rule measures otherwise, in the log.
        for which, reading in (("the file", theirs), ("the export", exported)):
            if (
                reading is not None
                and reading.fixed_volume is not None
                and not near(reading.fixed_volume, reading.volume, 1e-5)
            ):
                print(
                    f"{name}: {which}: {reading.volume:.8g} mm³ integrated adaptively, "
                    f"{reading.fixed_volume:.8g} by the fixed rule"
                )
        if "export-volume" in problems and readings is None and ours:
            # Which faces measure differently: in the log, and kept.
            diff = face_diff(corpus / name, Path(ours["exportPath"]))
            found.setdefault(name, {})["export faces"] = diff
            for side, faces in diff.items():
                for face in faces[:12]:
                    print(
                        f"{name}: {side}: {face['kind']} {face['area']:.6g} mm² "
                        f"({face['fixed area']:.6g} by the fixed rule) at "
                        + ", ".join(f"{c:.4g}" for c in face["centre"])
                    )
        unexpected = {key: why for key, why in problems.items() if key not in known}
        # A known gap that no longer fails: the manifest is to say so.
        for key in sorted(known.keys() & checked - problems.keys()):
            unexpected[key] = (
                f"{key} is as OpenCASCADE reads it now: take it out of known_gaps"
            )
        result = "ok"
        if unexpected:
            result = "**" + "; ".join(unexpected.values()) + "**"
            failures += 1
        elif problems:
            result = "known gap: " + "; ".join(
                f"{problems[key]} ({known[key]})" for key in problems
            )
        occt = (
            f"{theirs.solids} solids, {theirs.volume:.10g} mm³"
            if theirs
            else "unreadable"
        )
        mine = (
            f"{ours['bodies']} bodies, {exact_volume(ours):.10g} mm³ exact"
            if ours and not ours["error"]
            else "not read"
        )
        export = (
            f"{exported.solids} solids, {exported.volume:.10g} mm³" if exported else "—"
        )
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
        # An entry with the export's reading alone: OpenCASCADE could not read
        # the file, and there is nothing of it to write.
        if reading is None or "solids" not in reading or "not_read" in entry:
            continue
        expect = entry.setdefault("expect", {})
        # A surface model OpenCASCADE makes no solids of keeps Horizon CAD's
        # bodies and volume: its bounds alone are OpenCASCADE's.
        if "occt_reads_surfaces" not in entry:
            expect["bodies"] = reading["solids"]
            expect["volume"] = round(reading["volume"], 6)
        if reading["bounds"]:
            expect["bounds"] = [
                [round(v, 6) for v in corner] for corner in reading["bounds"]
            ]
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("check")
    run.add_argument("corpus", type=Path)
    run.add_argument("summary", type=Path)
    run.add_argument("--out", type=Path)
    run.add_argument(
        "--kept",
        type=Path,
        help="what OpenCASCADE found, as a run kept it (its occt.json), read in "
        "place of OpenCASCADE",
    )
    combine = commands.add_parser("merge")
    combine.add_argument("occt", type=Path)
    combine.add_argument("manifest", type=Path)
    args = parser.parse_args()
    if args.command == "check":
        return check(args.corpus, args.summary, args.out, args.kept)
    return merge(args.occt, args.manifest)


if __name__ == "__main__":
    sys.exit(main())
