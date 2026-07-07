# STEP interoperability fixtures

Part-21 files exercised by `tests/fileio/test_StepFixtures.cpp`.

## Layout

- `import_ok/` — every `*.step` file here must import successfully: at least
  one solid, manifold topology, positive enclosed volume.  The scanning test
  picks up new files automatically.
- `reject/` — every `*.step` file here must be **rejected with a clear
  error** (`StepFormat::lastError()` non-empty).  Used to pin documented
  limitations (e.g. `BREP_WITH_VOIDS`) so a silent-import regression fails
  the suite.

## Adding real third-party exports

The checked-in fixtures are hand-authored in the styles of common exporters
(FreeCAD/Open CASCADE, SolidWorks) because real vendor files cannot be
redistributed here.  When you have real exports from FreeCAD, Onshape,
SolidWorks, Fusion, etc.:

1. Drop the `.step`/`.STEP` file into `import_ok/`.
2. Run `hz_fileio_tests` — the scan test validates it imports cleanly.
3. If it exercises a known limitation instead (voids, assembly transforms,
   trimmed analytic faces), move it to `reject/` or add a dedicated test
   documenting the current behavior.

Keep fixtures small (single parts, few faces) so failures stay debuggable.
