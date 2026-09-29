# STEP interoperability fixtures

Part-21 files exercised by `tests/fileio/test_StepFixtures.cpp`.

## Layout

- `import_ok/` — every `*.step` file here must import successfully: at least
  one solid, manifold topology, positive enclosed volume.  The scanning test
  picks up new files automatically.
- `reject/` — every `*.step` file here must be **rejected with a clear
  error** (`StepFormat::lastError()` non-empty): malformed files (a
  `BREP_WITH_VOIDS` with empty loops) and documented limitations, so a
  silent-import regression fails the suite.

## Real third-party exports

These fixtures are hand-written in the styles of common exporters. Real
exports from twenty other CAD systems, with their licences, are in the
external corpus, `tests/interop/step/`, which OpenCASCADE checks too. Add a
real file there (its README says how), and keep these small and
hand-written, so a failure here stays easy to debug.
