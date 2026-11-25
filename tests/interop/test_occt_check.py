"""The OpenCASCADE check itself (tools/interop/occt_check.py): what it compares
and what it fails, run on readings a run kept (--kept), without OpenCASCADE."""

from __future__ import annotations

import contextlib
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

TOOL = Path(__file__).resolve().parents[2] / "tools" / "interop" / "occt_check.py"
sys.path.insert(0, str(TOOL.parent))

import occt_check as oc

BOX = [[0.0, 0.0, 0.0], [10.0, 10.0, 10.0]]
TORUS = (
    "its curve stands 3.09 off its face's surface, beyond the limit of 0.0508 "
    "(a thousandth of the solid's size) (face #709, edge #426)"
)


def solid(name: str, volume: float) -> dict[str, Any]:
    """A solid Horizon CAD read exact, as summary.json gives it."""
    return {
        "solid": name,
        "exact": True,
        "exactVolume": volume,
        "exactProblem": "",
        "meshVolume": volume,
        "meshAllowance": 1.0,
        "meshProblem": "",
    }


def left_out(name: str, why: str) -> dict[str, Any]:
    """A solid Horizon CAD left out for want of its exact form."""
    return {
        "solid": name,
        "exact": False,
        "exactVolume": 0.0,
        "exactProblem": why,
        "meshVolume": 0.0,
        "meshAllowance": 0.0,
        "meshProblem": "",
    }


def read(name: str, volume: float) -> dict[str, Any]:
    """A file Horizon CAD read: one body, exact, @volume mm³."""
    return {
        "file": name,
        "error": "",
        "bodies": 1,
        "facetedVolume": volume,
        "idealVolume": volume,
        "bounds": BOX,
        "exportPath": f"/export/{name}",
        "solids": [solid("solid 1", volume)],
    }


def not_read(name: str, error: str, solids: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "file": name,
        "error": error,
        "bodies": 0,
        "facetedVolume": 0.0,
        "idealVolume": 0.0,
        "bounds": None,
        "exportPath": f"/export/{name}",
        "solids": solids,
    }


def reading(volume: float, kinds: dict[str, int] | None = None) -> dict[str, Any]:
    """What OpenCASCADE found in a file, as occt.json keeps it."""
    return {
        "solids": 1,
        "volume": volume,
        "bounds": BOX,
        "valid": True,
        "fixed_volume": None,
        "kinds": kinds if kinds is not None else {"Plane": 6},
        "volume_error": 1e-7,
    }


class Corpus:
    """A corpus as check reads it: its manifest, Horizon CAD's summary.json and
    the occt.json a run kept, in a folder of their own. The STEP files
    themselves are never opened."""

    def __init__(self) -> None:
        self.dir: Path = Path(tempfile.mkdtemp())
        self.entries: list[dict[str, Any]] = []
        self.summary: list[dict[str, Any]] = []
        self.kept: dict[str, Any] = {}

    def add(
        self,
        name: str,
        ours: dict[str, Any],
        theirs: dict[str, Any] | None,
        **manifest: Any,
    ) -> None:
        self.entries.append({"file": name, "expect": {}, **manifest})
        self.summary.append(ours)
        if theirs is not None:
            self.kept[name] = theirs

    def check(self, out: Path | None = None) -> tuple[int, str]:
        """check's exit code and what it printed."""
        _ = (self.dir / "manifest.json").write_text(
            json.dumps({"files": self.entries}), encoding="utf-8"
        )
        summary = self.dir / "summary.json"
        _ = summary.write_text(json.dumps({"files": self.summary}), encoding="utf-8")
        kept = self.dir / "occt.json"
        _ = kept.write_text(json.dumps({"files": self.kept}), encoding="utf-8")
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed), contextlib.redirect_stderr(printed):
            code = oc.check(self.dir, summary, out, kept)
        return code, printed.getvalue()


def exported(
    original: dict[str, Any], export_kinds: dict[str, int] | None = None
) -> dict[str, Any]:
    """@original with OpenCASCADE's reading of the export beside it: the same,
    or its faces of the kinds given."""
    kept = dict(original)
    kept["export"] = reading(original["volume"], export_kinds or original["kinds"])
    return kept


class KindsTest(unittest.TestCase):
    def test_the_same_faces_agree(self) -> None:
        self.assertIsNone(
            oc.kinds_off({"Plane": 6, "Cylinder": 2}, {"Cylinder": 2, "Plane": 6})
        )

    def test_an_extrusion_may_go_out_as_a_bspline(self) -> None:
        original = {"Plane": 4, "SurfaceOfExtrusion": 1, "BSplineSurface": 2}
        self.assertIsNone(oc.kinds_off(original, {"Plane": 4, "BSplineSurface": 3}))
        self.assertIsNone(oc.kinds_off(original, original))

    def test_a_face_of_another_kind_is_said(self) -> None:
        off = oc.kinds_off(
            {"Plane": 6, "Cylinder": 2},
            {"Plane": 6, "Cylinder": 1, "BSplineSurface": 1},
        )
        assert off is not None
        self.assertIn("Cylinder 1, not 2", off)
        self.assertIn("BSplineSurface 1, not 0", off)

    def test_only_an_extrusion_may_become_a_bspline(self) -> None:
        off = oc.kinds_off(
            {"Plane": 6, "SurfaceOfExtrusion": 1}, {"Plane": 5, "BSplineSurface": 2}
        )
        assert off is not None
        self.assertIn("BSplineSurface 2, not 1", off)
        self.assertIn("Plane 5, not 6", off)

    def test_no_more_extrusions_than_the_original(self) -> None:
        off = oc.kinds_off({"Plane": 6}, {"Plane": 6, "SurfaceOfExtrusion": 1})
        assert off is not None
        self.assertIn("SurfaceOfExtrusion 1, not 0 or fewer", off)


class CheckTest(unittest.TestCase):
    def test_an_exact_volume_within_a_millionth_agrees(self) -> None:
        corpus = Corpus()
        corpus.add("a.step", read("a.step", 1000.0005), exported(reading(1000.0)))
        code, printed = corpus.check()
        self.assertEqual(code, 0, printed)
        self.assertIn("| a.step |", printed)

    def test_an_exact_volume_further_off_fails(self) -> None:
        corpus = Corpus()
        corpus.add("a.step", read("a.step", 1000.002), exported(reading(1000.0)))
        code, printed = corpus.check()
        self.assertEqual(code, 1, printed)
        self.assertIn("exact volume 1000.002", printed)
        self.assertIn("its own error estimate 1e-07", printed)

    def test_the_manifests_tolerance_is_taken(self) -> None:
        corpus = Corpus()
        corpus.add(
            "a.step",
            read("a.step", 1000.002),
            exported(reading(1000.0)),
            expect={"volume_tolerance": 5e-6},
        )
        code, printed = corpus.check()
        self.assertEqual(code, 0, printed)

    def test_a_file_not_read_for_its_listed_solids_is_a_known_gap(self) -> None:
        corpus = Corpus()
        error = f"solid #12 could not be made exact: {TORUS}"
        corpus.add(
            "screw.step",
            not_read("screw.step", error, [left_out("solid #12", TORUS)]),
            reading(3788.27),
            exact_failures=[{"solid": "solid #12", "why": "its curve stands 3.09 off"}],
        )
        code, printed = corpus.check()
        self.assertEqual(code, 0, printed)
        self.assertIn("known gap: Horizon CAD did not read it", printed)

    def test_a_file_not_read_for_an_unlisted_solid_fails(self) -> None:
        corpus = Corpus()
        error = f"solid #12 could not be made exact: {TORUS}"
        corpus.add(
            "screw.step",
            not_read("screw.step", error, [left_out("solid #12", TORUS)]),
            reading(3788.27),
            exact_failures=[{"solid": "solid #12", "why": "another reason"}],
        )
        code, printed = corpus.check()
        self.assertEqual(code, 1, printed)
        self.assertIn("**Horizon CAD did not read it", printed)

    def test_an_export_whose_faces_change_kind_fails(self) -> None:
        corpus = Corpus()
        original = reading(1000.0, {"Plane": 4, "Cylinder": 2})
        corpus.add(
            "a.step",
            read("a.step", 1000.0),
            exported(original, {"Plane": 4, "BSplineSurface": 2}),
        )
        code, printed = corpus.check()
        self.assertEqual(code, 1, printed)
        self.assertIn("Cylinder 0, not 2", printed)

    def test_an_extrusion_written_as_a_bspline_agrees(self) -> None:
        corpus = Corpus()
        original = reading(1000.0, {"Plane": 4, "SurfaceOfExtrusion": 1})
        corpus.add(
            "a.step",
            read("a.step", 1000.0),
            exported(original, {"Plane": 4, "BSplineSurface": 1}),
        )
        code, printed = corpus.check()
        self.assertEqual(code, 0, printed)

    def test_a_known_gap_that_closes_fails(self) -> None:
        corpus = Corpus()
        original = reading(1000.0, {"Plane": 4, "Cylinder": 2})
        corpus.add(
            "a.step",
            read("a.step", 1000.0),
            exported(original),
            known_gaps={"export-kinds": "a tessellated copy OpenCASCADE alone reads"},
        )
        code, printed = corpus.check()
        self.assertEqual(code, 1, printed)
        self.assertIn("export-kinds is as OpenCASCADE reads it now", printed)

    def test_a_known_gap_is_reported_not_failed(self) -> None:
        corpus = Corpus()
        original = reading(1000.0, {"Plane": 4, "Cylinder": 2, "none": 9})
        corpus.add(
            "a.step",
            read("a.step", 1000.0),
            exported(original, {"Plane": 4, "Cylinder": 2}),
            known_gaps={"export-kinds": "a tessellated copy OpenCASCADE alone reads"},
        )
        code, printed = corpus.check()
        self.assertEqual(code, 0, printed)
        self.assertIn("known gap: the export's faces by kind: none 0, not 9", printed)

    def test_a_reading_older_than_kinds_compares_no_kinds(self) -> None:
        corpus = Corpus()
        original = reading(1000.0)
        original["kinds"] = None
        corpus.add("a.step", read("a.step", 1000.0), exported(original))
        code, printed = corpus.check()
        self.assertEqual(code, 0, printed)

    def test_what_it_keeps_reads_back_the_same(self) -> None:
        corpus = Corpus()
        original = reading(1000.0, {"Plane": 4, "SurfaceOfExtrusion": 1})
        corpus.add(
            "a.step",
            read("a.step", 1000.0),
            exported(original, {"Plane": 4, "BSplineSurface": 1}),
        )
        again = corpus.dir / "again.json"
        code, printed = corpus.check(out=again)
        self.assertEqual(code, 0, printed)
        self.assertEqual(
            json.loads(again.read_text(encoding="utf-8"))["files"], corpus.kept
        )

    def test_it_needs_no_opencascade(self) -> None:
        corpus = Corpus()
        corpus.add("a.step", read("a.step", 1000.0), exported(reading(1000.0)))
        _ = corpus.check()
        self.assertNotIn("OCP", sys.modules)
        # And from the command line, as CI would run it with --kept.
        run = subprocess.run(
            [
                sys.executable,
                str(TOOL),
                "check",
                str(corpus.dir),
                str(corpus.dir / "summary.json"),
                "--kept",
                str(corpus.dir / "occt.json"),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)


class MergeTest(unittest.TestCase):
    def test_it_writes_what_opencascade_found(self) -> None:
        folder = Path(tempfile.mkdtemp())
        manifest = folder / "manifest.json"
        _ = manifest.write_text(
            json.dumps(
                {
                    "files": [
                        {"file": "a.step", "expect": {"skipped": 0}},
                        {
                            "file": "emblem.stp",
                            "expect": {"bodies": 12, "volume": 9657.479022},
                            "occt_reads_surfaces": "surfaces to OpenCASCADE",
                        },
                    ]
                }
            ),
            encoding="utf-8",
        )
        found = folder / "occt.json"
        surfaces = reading(0.0)
        surfaces["solids"] = 0
        surfaces["bounds"] = [[-1.0, -2.0, -3.0], [1.0, 2.0, 3.0]]
        _ = found.write_text(
            json.dumps(
                {"files": {"a.step": reading(1000.1234567), "emblem.stp": surfaces}}
            ),
            encoding="utf-8",
        )
        self.assertEqual(oc.merge(found, manifest), 0)
        files = json.loads(manifest.read_text(encoding="utf-8"))["files"]
        self.assertEqual(
            files[0]["expect"],
            {"skipped": 0, "bodies": 1, "volume": 1000.123457, "bounds": BOX},
        )
        # A surface model keeps Horizon CAD's bodies and volume; its bounds
        # are OpenCASCADE's.
        self.assertEqual(
            files[1]["expect"],
            {"bodies": 12, "volume": 9657.479022, "bounds": surfaces["bounds"]},
        )


if __name__ == "__main__":
    unittest.main()
