# Changelog

All notable changes to Horizon CAD are recorded here. The project was built
phase-by-phase against the roadmap in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md);
this file summarizes that work by era. Each phase shipped as an honest core
slice — where the roadmap named a heavy third-party dependency, an in-house
implementation was built instead to keep CI lean and the code testable
headless. Those deviations (STEPcode/OCCT, Embree, OpenCAMLib) are documented
in [the era findings note](docs/superpowers/notes/2026-07-03-era2-roadmap-findings.md).

## Unreleased — Production readiness, Milestone 4 (Phases 111–114)

- **Ten common shortcuts did nothing (111).** Ctrl+Z, Ctrl+Y, Ctrl+S,
  Ctrl+O, Ctrl+N, Ctrl+C, Ctrl+V, Ctrl+D, Ctrl+G and Ctrl+Shift+G were each
  bound twice, once to the menu and once to the ribbon, which Qt treats as
  ambiguous and ignores. Each is now one action, and a test checks every
  shortcut in the window.
- **The application forgot everything between sessions (111):**
  - File ▸ Open Recent lists the ten newest files;
  - the window's size and dock layout are kept;
  - Edit ▸ Preferences sets:
    - the autosave interval;
    - the language;
    - the grid snap spacing;
    - the snap reach;
    - the unit coordinates and measurements are shown in.
  - Help ▸ About shows the version, revision and build.
  - `horizon file...` opens files from the command line.
  - View reaches every dock.
- **Every command is exercised (112).** A smoke test triggers each of the
  window's 166 commands on an empty drawing, part and assembly, and on a
  selection, dismissing whatever dialog it opens. The drawing tools are
  tested through the viewport's own mouse handling: draw, select, delete,
  undo.

## Unreleased — Production readiness, Milestone 3 (Phases 107–110)

- **2D snapping and picking depended on zoom, and ignored hidden layers (110).**
  - Snapping reached a fixed 0.5 units: thousands of pixels when zoomed in,
    under one pixel when zoomed out. Picking had a 0.15-unit floor with the
    same effect. Both now reach 10 pixels on screen at any zoom.
  - Hidden and locked layers were snapped to, and Trim cut at them. Both
    now leave them alone.
  - Every snap was called an endpoint, and a line's midpoint was no snap at
    all. Snaps are now typed: endpoint, midpoint, centre, quadrant, and a
    new intersection snap.
  - An object snap in reach now beats the grid.
  - Trim's pieces lost their line type and group; Break, Extend, Chamfer
    and Fillet lost the group. All of these are kept now.
- **STEP files came in at the wrong size, all-or-nothing, with holes filled
  in (109).**
  - The length unit was ignored, so a part in inches came in 25.4 times
    too small and one in metres 1000 times too small.
  - One solid the reader could not rebuild refused the whole file.
  - A face with a hole was drawn over it, counted as solid in mass
    properties, and cut through by Booleans.

  Solids now come in in millimetres, and the conversion is noted. A bad
  solid is reported and the rest come in. Holes are holes.
- **DXF text, colour and units came in wrong (108b).**
  - MTEXT chunks were joined back to front. Its lines ran together into
    one. Formatting codes such as `\pxi-3;` were left in the text.
  - Centred and right-aligned TEXT came in at the wrong point.
  - `%%d`, `%%c` and `\U+00B0` came in as they are written, not as °, ⌀
    and the character.
  - Only ten colours were known, so index 12 came in white; true colour
    was ignored.
  - Pre-2007 files in Windows-1252 or 1251 came in as broken UTF-8.
  - A drawing in inches came in 25.4 times too small.

  All of these are read correctly now. Saving writes true colour, escapes
  text so it reads back, and declares millimetres. Formatting the drawing
  cannot show, such as underline or a height change inside a text, is
  reported.
- **DXF geometry came in wrong in several common cases (108a).**
  - A polyline's arc segments (bulges) came in as straight lines.
  - Mirrored entities (an extrusion of (0, 0, −1)) came in unmirrored.
  - The old POLYLINE / VERTEX form was not read at all.
  - A partial ellipse came in whole.
  - A block inserted with unequal scales was drawn at their average: (2, 1)
    became 1.5. A mirrored insert lost its mirror.
  - A block inside a block was dropped.

  All of these are now read as the file means them. Where the drawing model
  cannot hold something as it was, the import report says what was
  approximated:
  - a polyline with arcs comes in as grouped lines and arcs;
  - a stretched block comes in exploded;
  - a partial ellipse comes in as a polyline.

  Entities out of the drawing's plane, and 3D polylines, are reported rather
  than flattened.
- **Mirroring an arc across an axis that missed its centre put it in the
  wrong place (108a).** The 2D Mirror tool was affected. Fixed.

- **STEP, STL and glTF could not be reached from the window (107).** STEP
  import and export and glTF export existed in code, but no menu reached
  them, and there was no STL export at all. File ▸ Import and File ▸ Export
  now cover all four:
  - **STEP** imports as a new part. Each body is kept inside the part, so the
    part no longer depends on the STEP file.
  - **DXF** imports into the drawing you are working on, as one undoable
    step.
  - **Exports:** STEP, STL (new) and glTF for a part's body, and DXF for a
    drawing.
- **Opening a file silently lost what could not be read.** Since Phase 100 a
  malformed item is skipped instead of failing the whole file, but nothing
  said so, and the next save dropped it for good. A DXF's unread entity types
  vanished the same way. Opening or importing such a file now lists what was
  left out, and why ("entity 7 (spline): …", "3 3DFACE entities not read"),
  before you can save over it. A feature whose sketch is missing now says so.

## Unreleased — Production readiness, Milestone 2 (Phases 102–106b) — complete

- **Every Boolean result was a heap of triangles, which Fillet refused
  (106b).** The CSG triangulates both parts and splits the triangles along
  the other part's planes, then kept the pieces apart. A groove across a block
  came out as more than 20 triangles for 10 faces, and even faces the cut
  never reached came out in pieces. Every corner had extra edges meeting at
  it, so Fillet refused any edge of a part that had been through a Boolean
  ("non box-like corner"). A plate with a hole could not be filleted.

  Each face's pieces are now put back together after the Boolean. A face with
  a hole through it is cut through the hole into two, since a face here has a
  single loop; the cut meets the outer edge part-way along, so the corners are
  unaffected. The grooved block has its 10 faces, and names survive a
  Boolean that doesn't touch them. **A plate with a hole can now be filleted,
  undone, saved and reopened through the window**, which was the goal of
  Milestone 2. Files saved earlier rebuild exactly as before.

- **An edit to a sketch could move a fillet to a different edge (106).** An
  extrusion numbered its side faces in profile order and its edges in storage
  order, and a fillet stores the name of the edge it rounds. Adding a vertex
  anywhere in the sketch renumbered the edges, so the fillet silently rounded
  whichever edge now had its old number.

  New extrusions name their side faces after the sketch entities they come
  from (a line, a side of a rectangle, a facet of an arc) and their edges
  after the two faces they separate. An edit elsewhere in the sketch leaves
  those names alone.

  Boolean results follow the same rule:
  - A face a Boolean splits gets a name for each piece; pieces used to share
    one name.
  - Fillet, Chamfer, Shell and mates that refer to the whole face or edge find
    its pieces.

  Files saved before this keep the old names for every feature and every
  Boolean result, since their references were made against those names. The
  format moves to version 18, so older builds refuse the new files.

  (Boolean results were still left in fragments, which renamed edges beside
  an unrelated cut; 106b, above, puts them back together.)

- **Parts with a hole through them failed validation (105).** The
  Euler–Poincaré check read `V − E + F = 2(S − R)`: the face-hole count was on
  the wrong side, doubled, and the formula had no genus term. So every torus,
  and every part with a hole through it, was "invalid". That was not just a
  label:
  - STEP import refused such parts.
  - Fillet refused to round any edge on one.

  The check is now `V − E + F − R = 2(S − G)`, and `Solid::genus()` reports
  the number of through-holes.

  Every feature's result is now held to the full set of solid checks: every
  edge between two faces, counts Euler allows, flat faces flat, no boundary
  crossing itself, and a closed skin. The same goes for every Join, Cut and
  Intersect result. A malformed solid stops at the feature that made it, with
  a reason ("the result is not a valid solid: …"), instead of corrupting the
  features built on it.

  A profile that crosses or touches itself, such as a figure-eight, is refused
  with the point named. It bounds no single region.

- **Most 3D ribbon commands were demos (104b).** Box, Cylinder, Sphere, Cone,
  Torus, Union, Subtract, Intersect, Fillet and Chamfer each dropped a fixed
  mesh into the viewport, outside the document: the mesh was never saved or
  undone, and it ignored the part entirely. (A fillet was always "one edge of
  a 10 mm box".) Shell, Draft and Pattern had no command at all.

  Each command now asks for its inputs and adds a feature to the part, as one
  undoable step:
  - **Primitives** ask for their sizes and how the body combines with the
    part.
  - **Union / Subtract / Intersect** combine the part's separate bodies.
  - **Fillet and Chamfer** take the edges you choose. **Shell** takes the
    faces to open and a wall thickness. Until the viewport can pick edges and
    faces, they are chosen from lists: edges by their end points, faces by
    which way they face and where their middle is.
  - **Draft** takes a pull direction, a neutral plane and an angle.
  - **Linear and Circular Pattern** take a direction or axis, a spacing or
    angle, and a count.

  A feature that fails on the spot, such as a fillet too big for its edge,
  is refused with its reason rather than added.

  **Loft and Sweep have no command yet.** They need sketches on more than one
  plane, and the window can sketch only on XY; that is the next step
  (Phase 104c).

- **Changes to a part's history could not be undone, and one was not even
  saved (104a).** Undo covered 2D drawing only. Adding, reordering and editing
  features went around the undo stack, and **editing a feature's values
  (double-click) did not mark the document modified**, so closing the window
  discarded the edit without asking. Features could not be deleted or
  suppressed at all. Inserting a component or adding a mate could not be
  undone either, and a mate solve moved components with no way back.

  Every one of these is now a command on the undo stack:
  - adding a feature (with the profile sketch made for it);
  - editing a feature's values and how its body combines with the part, in
    one dialog instead of one prompt per value;
  - reordering, deleting and suppressing a feature, from the feature panel's
    menu and the Delete key;
  - inserting a component, and adding a mate together with the moves its solve
    made.

  Undo rebuilds the model or assembly it changed. The document's modified
  marker follows the undo stack, so undoing back to the saved state clears it.
  Script edits go through the same commands.

  A suppressed feature stays in the history but is left out of the build.
  It is saved as `"featureSuppressed"`, and the file format moves to version
  17: a build from before body operations (102) or suppression would ignore
  both and silently build a different part, so it now refuses the file
  instead.

- **A failed feature said "failed to execute" (103).** Extrude, Revolve,
  Loft, Sweep, Draft, the primitives, patterns and `BooleanOp` returned a bare
  null pointer; Shell, Fillet and Chamfer produced messages that their
  features then dropped; `ProfileValidator` explained a bad profile and
  Extrude threw the explanation away; and the Extrude command guessed "profile
  is not a closed loop" for every failure. `Feature::execute` now takes an
  optional reason, and every feature fills it, so the feature tree and status
  bar say, for example, "the profile is open: its ends at (0, 0) and (0, 10)
  do not meet", "Edge not found: box/no_such_edge", or "the cut removes the
  whole body". Boolean failures now distinguish an empty result (a cut
  through everything, an intersection of bodies that do not touch) from one
  that could not be sewn. Extrude also refuses two inputs it used to turn
  into zero-volume solids without complaint: a zero distance, and a direction
  lying in the sketch plane.

  Writing those messages turned up a larger gap: the profile reader
  understood lines, arcs and a lone circle only, so **a shape drawn with the
  Rectangle or Polyline tool — the usual way to draw one — could not be
  extruded, revolved, lofted or swept**, and was reported as a circle
  problem. Rectangles and polylines (and so imported LWPOLYLINEs) now count as
  the line segments they are drawn with; anything still unsupported is named
  ("an ellipse cannot be used in a profile yet").

- **A part could hold only one feature's geometry (102).** The rebuild
  threaded a single solid through the feature tree, and every feature that
  builds geometry — Extrude, Revolve, Loft, Sweep, the primitives — ignored the
  solid it was handed: a second extrude *replaced* the first, and a hole could
  not be cut. `buildBodies()` knew about separate bodies but only the tests
  called it, and the Boolean feature was a no-op on the product path.

  Each creating feature now has a body operation. *Join*, *Cut* and
  *Intersect* combine its body with the part through `BooleanOp`; *New body*
  keeps it as a separate shell of the part's solid, which rendering, the
  tessellation cache, mates and mass properties already handle. A cut or
  intersect with nothing to act on, a cut that would leave nothing, and a
  Boolean that fails are feature failures with a reason, never an empty part.
  All three build paths share one rule. The Extrude and Revolve commands ask
  for the operation (Join once the part has a body, New body for the first)
  and refuse — rather than add — a feature that fails on the spot. Stored as
  `"bodyOperation"`; files written before load every body as its own, so a
  part that showed only its last extrude now shows all of them.

  Two transforms assumed one body, which a spaced pattern could already
  break and New body now makes routine. Shell rebuilt its cup from one body's
  caps and dropped every other body without a word; it now refuses a part
  with several bodies. Draft oriented every face against the centroid of the
  whole solid, which for two bodies lies between them, outside both — so the
  sides facing each other were drafted the wrong way; each face is now
  oriented against its own body.

## Unreleased — Production readiness (Phases 97–101) — Milestone 1 complete

Work against the [production-readiness roadmap](docs/superpowers/specs/2026-09-23-production-readiness-roadmap.md).

- **The warning and sanitizer gates did not gate (97).** `cmake/CompilerWarnings.cmake`
  and `cmake/Sanitizers.cmake` defined `hz_set_warnings()` and
  `hz_enable_sanitizers()`, and nothing called either. The whole project built
  with no warning flags, and the CI job named "AddressSanitizer" configured
  `-DHZ_ENABLE_SANITIZERS=ON` into a plain Debug build — its log contains no
  `-fsanitize` at all. Both functions are now applied from the root
  `CMakeLists.txt` to every first-party target, so a new module is covered
  without opting in. GCC/Clang build with `-Wall -Wextra -Wpedantic
  -Wconversion` (MSVC `/W4 /permissive-`), and `HZ_WARNINGS_AS_ERRORS` fails
  the Linux CI jobs on any warning. UBSan is built with
  `-fno-sanitize-recover`, since by default it reports and carries on, which
  lets a test pass straight over undefined behaviour. The full suite is clean
  under ASan + UBSan + LeakSanitizer. `-Wsign-conversion` (about 560 size/index
  sites) is left off as a tracked burn-down rather than suppressed site by
  site, and clang-tidy is told the same, because Clang reads GCC's
  `-Wconversion` as including it.

  The warnings found about 25 sites, mostly dead code: unused variables, a
  lambda nothing called, an unused `flipped` flag in the angular dimension's
  arrowheads, `Eigen::Index` narrowed to `int`, a Qt signal deprecated in 6.9,
  and `Solid` forward-declared as a `struct` but defined as a `class` — legal,
  but the tag is part of the MSVC-mangled name.
- **Building on Linux without compiling Qt.** Qt is now a default-on `qt`
  feature of the vcpkg manifest; the new `linux-system-qt` preset turns it off,
  so vcpkg provides only the small libraries and the build uses an installed
  Qt 6. C++20 module scanning is off (no modules are used), which also stops
  CMake ≥ 3.28 with GCC ≥ 14 writing flags into `compile_commands.json` that
  clang-tidy cannot parse.
- **Line endings.** A checkout copied from Windows to Linux showed all 559
  files as modified — CRLF in the working tree, LF in the index.
  `.gitattributes` now pins LF everywhere.
- **Quitting lost unsaved work (98).** There was no `closeEvent`: File ▸ Exit
  and the window's close button discarded every open document without a
  word. And a drawing never became "modified" in the first place — the 2D
  commands edit `DraftDocument`, which had no dirty flag, and
  `Document::m_dirty` was only set by two methods the UI never calls, so the
  tab-close prompt that did exist could not fire for 2D work.

  The modified state now comes from the undo stack: it records which state was
  last saved, so every command marks the document modified, undoing back to
  the saved state clears the mark, and a push that discards the redo history
  holding the saved state leaves it unreachable. Changes made outside the undo
  stack (feature edits, until they become commands) still mark the document
  explicitly. Quitting walks every modified tab and asks Save / Discard /
  Cancel; closing a tab offers Save as well (it offered only Close and
  Cancel); modified tabs show `*` and the title bar Qt's modified marker. A
  new offscreen test binary drives the real `MainWindow` through each of these.
- **Saves could destroy the file they replaced (98).** Every writer opened the
  target with `std::ofstream`, truncating it, *then* serialized. `json::dump()`
  throws on invalid UTF-8 — which DXF-imported text carries whenever its source
  used a legacy code page — so saving such a drawing as `.hcad` left a 0-byte
  file and threw out of the Qt event loop. `file.good()` was also checked
  before `close()`, so a failed final flush (disk full) reported success. All
  writers now build their output in memory and replace the target through
  `io::writeFileAtomically`: temporary file in the same directory, flushed to
  disk, renamed over the target, permissions kept, symlinks followed, and on
  any failure the original untouched and the temporary removed. Invalid UTF-8
  is written as U+FFFD instead of throwing.
- **Non-ASCII paths on Windows (98).** Paths are UTF-8 strings throughout, but
  `std::ifstream(std::string)` reads a narrow path in the Windows code page, so
  a file under `C:\Users\José` could not be opened or saved. Readers and
  writers now open paths as UTF-8, and the executable declares UTF-8 as its
  active code page.
- `~ViewportWidget` dereferenced the current GL context unconditionally and
  crashed when the viewport had never been shown.
- **An error left no trace and no reason (99).** The whole code base made two
  spdlog calls, both to stdout — which a Windows GUI-subsystem executable
  discards — so a problem in the field left nothing behind. Logging now goes
  to a rotating file in the platform's app-data directory (`…/logs/horizon.log`,
  3 × 5 MB), Qt's own warnings are routed into it, warnings flush immediately,
  and start-up records the version, Qt version, OS and OpenGL driver.

  An exception escaping a slot or event handler terminated the application,
  and every open document with it; a kernel op that throws (the NURBS
  constructors do, on invalid input) during a rebuild would do exactly that.
  `hz::ui::Application::notify()` now contains it: the failing command is
  abandoned, the user is told once (not once per repaint), and the session
  survives to save. Feature execution catches too, so a throwing feature is a
  failed feature whose reason appears in the tree. `std::terminate` logs its
  cause before the process goes.

  Files that failed to open or save said "Failed to open file." and nothing
  more. The loaders and savers now return the reason — "the file does not
  exist", "it is a folder, not a file", "parse error at line 3, column 5: …",
  "this is not a Horizon document", the write error — and never throw; a
  wrong-typed field that used to throw `json::type_error` out of `load()` is
  now "the file is damaged: …". A viewport that cannot draw (no OpenGL 3.3,
  shaders that fail to compile, or no context at all) says so instead of
  staying blank.
- **Hostile input could crash, hang or quietly misread the app (100).**
  - *Native files.* The reader indexed `const json` objects with `[]`, which
    for a missing key or an empty array is an assertion in Debug and
    undefined behaviour in Release — no `try` can catch it. Every access in
    the readers is now checked. Integer and enum fields are range-checked (a
    float where an integer belongs went through a plain `static_cast`; a line
    type out of range would index past the dash-pattern table). A file from a
    newer format version is refused with both version numbers instead of
    loading with its unknown content dropped, ready to be destroyed by the
    next save. Counts that cost memory on every rebuild are clamped:
    `segments` at 4096 steps per turn (the cap the tolerance path already
    used), fillet chords at 1024, pattern instances at 10 000 — a pattern
    count of 2e9 in a file tried to allocate that many copies on open, and
    `static_cast<int>` of an infinite parameter was undefined behaviour.
  - *DXF.* A file cut short hung the application: the entity loop re-read its
    last pair forever at end of input. Every section now reads through one
    function that treats the end of input as "the file ends before the end of
    a section (it may be truncated)", a group code without a value and a
    non-numeric code line are errors naming the line, and numbers are parsed
    with `std::from_chars` — `std::stod` follows the C locale, which Qt sets
    from the environment on Unix, so under de_DE "1.5" read as 1.
  - *STEP.* Exceptions from the NURBS constructors (a degree-0 B-spline passes
    the reader's own checks) became the import error instead of escaping;
    `#` entity numbers are overflow-checked; reals are written and read with
    `to_chars`/`from_chars`, which ignore the locale and round-trip exactly
    (`%.15g` did neither), and a malformed number is an error rather than a
    silent prefix.
  - *Expressions* read from files recursed without limit: 200 000 minus signs
    overflowed the stack. Parsing is bounded to 64 levels of nesting and 1024
    nodes (a long flat sum parses in a loop but builds a tree evaluation must
    recurse through), and `Expression::fromJson` now honours its "nullptr on
    any error" contract instead of throwing on a wrong-typed field.
  - *Fuzzing.* libFuzzer targets for the native, DXF and STEP readers and for
    expressions (`HZ_BUILD_FUZZERS=ON`, Clang). The same targets always build
    as corpus replays registered with CTest, so the seed corpus — real
    documents from the real writers, plus every crashing input above — runs
    in every build, including the sanitizer job.
- **A crash lost everything since the last save (101).** Every two minutes
  (`autosave/intervalSeconds` in the settings; 0 turns it off) each modified
  document that changed since its last snapshot is written — atomically, in
  native format even for a DXF drawing — to a recovery directory owned by the
  running session and removed when it exits cleanly. After a crash, the next
  start finds the session whose lock's process is gone and offers its
  documents back: Recover reopens them modified, pointing at where they were
  saved and marked "(recovered)" until saved again; Discard deletes them;
  Later keeps them for the next start. A second instance running at the same
  time is never mistaken for a crashed one, and two instances starting
  together cannot both recover the same documents.

## Unreleased — Post-1.0 kernel work, continued (Phases 89–96)

Continues against the "Not yet addressed" list in the
[post-1.0 findings note](docs/superpowers/notes/2026-09-01-post-1.0-kernel-findings.md).

- **Sweep collapsed on any turning path (89).** `Sweep` carried the profile
  along the path by translation only, which the header described as a
  fidelity limit — "the profile keeps its orientation". For a turning path it
  was a correctness defect: an XY-plane square swept up and then along +X was
  translated edge-on for the second leg, so that leg was a zero-thickness
  sheet. The L-shaped sweep integrated to 40 against 72, with two degenerate
  faces the geometric validator reports and the topology-only test never
  asked about. Separately, `SweepFeature` reduced every arc in a path sketch
  to its chord, so a bent path was swept as one straight segment.

  The profile is now carried by a rotation-minimizing frame: at every interior
  path point the section is the cut of the incoming prism by the miter plane
  bisecting the turn, which makes the cross-section perpendicular to each
  segment the profile turned by the smallest rotation between consecutive
  directions, and the far cap the profile's plane carried through the same
  turns. Every lateral face is exactly planar (its corners lie on two lines
  parallel to the segment), and with the profile's centroid on the path the
  volume is *exactly* normal-section area × path length — which the tests
  assert on a path turning in three planes. A path that doubles back, a
  profile whose plane contains the sweep direction, a turn tight enough that
  the inside of the profile would travel backwards, and any result the
  geometric validator rejects are refused instead of returned.

  Path arcs are sampled at `segments` steps per turn — a `SweepFeature`
  parameter, editable and persisted like the Phase 88 resolutions — so a bent
  sweep converges to area × arc length from below.
- **Accuracy is a distance, not a count (90).** Phase 88 made the facet count
  a feature property, but a count is the wrong unit: a fixed n sags
  r(1 − cos(π/n)), so the default 32 facets are within 0.024 on a radius-5
  cylinder and 0.48 on a radius-100 one. `segmentsForTolerance()` existed on
  `PrimitiveFactory` and `Revolve` and nothing outside the tests called it.
  Curved primitives, revolves, sweeps and fillets now take a `chordTolerance`
  parameter: when positive, the count is re-derived from it and the governing
  radius on every rebuild — the widest circle of a cone or torus, the profile
  vertex farthest from a revolve's axis, each path arc's own radius, and the
  fillet radius over the quarter arc its blend spans (new
  `FilletOp::arcSegmentsForTolerance()`). A radius edit therefore keeps the
  accuracy rather than the count. Setting the count explicitly returns to
  count mode; 0 turns the tolerance off; the tolerance is persisted and
  reloads as a tolerance, not as the count it produced. The feature parameter
  dialog's 0.001 floor would have silently switched a tolerance of 0 on for
  anyone clicking through it, so that field now accepts 0.
- **Rational NURBS surfaces were evaluated off their surface (91).**
  `NurbsSurface::evaluate` runs De Boor in two passes — each row in V, then
  across the rows in U — and gave the second pass unit weights. That discards
  the U-direction rationality, so every point of a cylinder, sphere, torus or
  cone *between* knots sat off the surface: up to 0.30 on a radius-5 cylinder,
  0.18 on a radius-3 sphere, 0.42 on a torus (about 6% of the radius), while
  every knot value was exact. The geometry tests had tolerated this in
  comments ("the two-pass evaluation loses some rational precision", "~5%")
  with tolerances of 0.2–0.5. The second pass now carries each row's weight
  sum Σⱼ Nⱼ(v)·wᵢⱼ, which makes it the exact tensor-product rational surface;
  those tests assert 1e-12, including off-knot samples on all four quadrics
  and the weighted-surface centre against the closed-form homogeneous value.
  Mate frames were unaffected only because they happen to sample at knots.
- **Extrude turned circles into squares (92).** Phase 84 found the curved
  primitives were box topology wearing a curved surface. Extrude, the most
  used feature in the product, had the same disease and was not in that
  sweep: a circle profile was extruded through box topology from four points
  on the circle, with a cylinder surface pasted onto the four flat sides, so a
  radius-5 disc extruded 10 integrated to **500 against 785**. Every arc in a
  line/arc profile was taken as its chord — a slot's round ends vanished (80
  against 111.4). Profile extraction was shared, so Revolve, Sweep and Loft
  chorded arcs the same way, and a circle section made Loft return nothing.

  Profiles are now faceted by one shared sampler: arcs are followed along
  their curve and a circle becomes an N-gon, at `segments` chords per turn or
  at the count each arc's own radius needs under `chordTolerance`, recording
  which chords came from which arc. The extrusion is an exact inscribed prism
  (the tests assert the N-gon volume to 1e-9) that converges to the exact
  solid from below. The lateral facets of an arc record their cylinder on
  `Face::analyticSurface` (for extrusions along the sketch normal) and every
  arc chord its circle on `Edge::analyticCurve`, so mates and radial
  dimensions still resolve from a single pick. A half disc now revolves to a
  sphere, two circles loft to the inscribed frustum, and a circle sweeps a
  pipe. `ExtrudeFeature` gains `segments` and `chordTolerance`, reported only
  when the profile has an arc or circle; Sweep's `segments` now also facets
  its profile. Documents that extrude a circle rebuild as the faceted cylinder
  on open, which changes their edge numbering: a fillet or chamfer that
  referenced one of the old square's edges by TopologyID no longer finds it.
- **Patterns stacked overlapping instances; Booleans and patterns dropped
  the ideals (93).** A pattern cloned every instance into its own shell
  whatever the spacing — the header called the merge "deferred" — so three
  10mm boxes 5 apart integrated to 3000 against the 2000 they occupy, and the
  six copies of a unit square patterned about its own corner were six
  interpenetrating shells that a document test asserted as correct. Every
  structural and geometric check passed. Instances whose bounds touch or
  overlap are now merged with `BooleanOp::Union` (a merge that fails refuses
  the pattern instead of returning interpenetrating shells); instances that
  stay apart remain separate bodies with no Boolean.

  Separately, the ideal geometry Phases 84–92 record was lost by the next
  operation. The pattern clone copied carriers but not `analyticSurface` or
  `analyticCurve`, and Boolean fragments were sewn without them, so a bored
  cylinder's bore — or any instance of a patterned boss — no longer resolved
  to a cylinder for a concentric mate or radial dimension. Pattern now moves
  the ideals with each instance; `BoundaryMesh` and `SolidSewer` carry
  `analyticSurface`, Boolean fragments recover it from their source face
  (looked up per operand, since two primitives of one kind share face IDs),
  and result edges lying along a source edge inherit its ideal curve.
- **Filleting a cylinder rim (94).** The open item Phases 85–86 left: a
  faceted cylinder's rim is a closed chain of chords, every vertex of which
  has two of them, and FilletOp refused any vertex with two selected edges
  ("exactly three are required for a corner blend"). So did two adjacent top
  edges of a box. Where two selected edges meet at a three-edge vertex, share
  one face, and the unselected third edge joins their other faces, the blends
  now meet on the plane bisecting the turn in the shared face: each blend's
  end section is carried along its edge onto that plane, the construction
  Phase 89 used for sweeps. The bands stay planar, the cap receives the inset
  polygon and the side edge is shortened by one radius, and no patch is
  needed. The removed material is exactly the blend cross-section times the
  length of its centroid path, which the tests assert to 1e-9 for a corner
  pair, an open chain, a square rim, a miter turned in a side face, and a
  32-chord cylinder rim. A turn too tight for the radius is refused.
- **Twisted lofts integrated the wrong solid (95).** A loft between a square
  and the same square turned 0.6 rad has lateral bands whose four corners are
  not coplanar. Such a loop encloses no well-defined volume, so each path in
  the kernel picked its own: mass properties and Booleans fanned it along one
  diagonal and got **180.8**, while the renderer drew the ruled patch, which
  encloses **150.7** — 20% apart. A level with any non-planar band is now cut
  along its rulings into strips, each non-planar strip into two triangles
  with the diagonal alternating from strip to strip. Every facet is flat, so
  the display and the computation see the same solid; and because the volume
  a bilinear patch bounds is exactly the mean of its two triangulations,
  alternating diagonals make the faceted volume equal the ruled loft's
  exactly — asserted to 1e-9 against Simpson's rule, which is exact for the
  quadratic section area. `twistSegments` (default 8, rounded up to even)
  only sets how closely the facets follow the curved patch, which each facet
  records on `analyticSurface`. Planar bands stay single quads, so aligned
  and similar sections build exactly as before.
- **Interference checking was unreachable (96).** `InterferenceChecker`
  (Phase 48) had no caller outside its tests: no assembly API, no command.
  It also reported only *whether* two solids clash — its header deferred the
  volume because the intersection Boolean was "not yet robust enough", which
  predates the kernel hardening. Each interfering pair now carries the
  volume of material the two share, from the Boolean intersection (flagged
  if that cannot be resolved); `AssemblyDocument::findInterference()` places
  every resolved, unsuppressed component by its transform and reports
  interfering pairs by component id, listing unresolved components as
  unchecked rather than passing them silently; and assemblies gain a
  **Check Interference** command. Mated faces and tangent cylinders touch
  without interfering. `Pattern::transformed()` exposes the placement copy,
  which moves carriers and ideals with the solid.

## Unreleased — Geometric validation, faceted geometry, working blends (Phases 81–88)

Post-1.0 kernel work, continuing from the review response below. Where kernel
hardening fixed what the Booleans *did*, this pass fixes what the kernel could
not *see*.

- **Geometric B-Rep validation (81).** `Solid::isValid()` walks twin/next/prev
  linkage and counts entities — it never reads a coordinate, so a solid can
  pass every structural check while its loops are self-intersecting,
  non-planar, or spanning positions its twin half-edges disagree about. The
  new `hz::topo::GeometryValidator` checks vertex-chain consistency
  (`he->next->origin == he->twin->origin`), twin positional coincidence,
  degenerate edges and faces, loop planarity against the face's own carrier
  (curved carriers are skipped rather than guessed at), loop
  self-intersection, and closed-shell area-vector balance — with advisory,
  non-failing reports for edge curves that miss their vertices and for
  distinct vertices at the same position.
- **Sharp cones were degenerate (81).** `makeCone` built box topology from two
  4-point rings; with `topRadius = 0` that ring collapses to a point, giving
  four zero-length edges around a zero-area cap. The Cone command asks for
  exactly that shape, so every cone inserted from the UI passed Euler and
  manifold while being geometric nonsense. Sharp cones now build apex topology
  (5V/8E/5F) with the analytic conical carrier; a cone with both radii or zero
  height is refused rather than returning a degenerate solid.
- **ChamferOp rebuilt on `SolidSewer` (82).** This closes the defect pinned in
  `test_AdversarialModels`: a 20mm box with two 2mm edge chamfers integrated
  to 3200 instead of 7920. Two things were wrong — loop construction pushed
  the original vertex back into the loop at corners the chamfer had already
  replaced, and reconstruction replayed the soup through Euler operators with
  a convergence loop that terminated on valid linkage that was not the linkage
  the polygons described. Loops are now vertex-driven and the soup is sewn by
  the same pipeline `BooleanOp` uses, so original faces keep their carriers
  and TopologyIDs. FilletOp needed no rebuild — its Phase-61 strict half-edge
  assembly already produced consistent loops — and both ops now refuse
  geometrically invalid output instead of returning it.
- **Chamfer vertex blends (83).** Two or three selected edges meeting at a
  corner used to be refused outright. The material a chamfer removes is the
  half-space beyond its chamfer plane, so a corner is just that corner clipped
  by the planes of every chamfer meeting there — blends fall out of the
  single-chamfer code path, with no invented corner patch, which is the
  standard planar-chamfer result. Validated against inclusion–exclusion
  volumes of the union of cutting prisms; chamfering all twelve edges of a
  cube yields the chamfered cube (18 faces, 32 vertices) at its exact volume.

Remaining known limits in this area are documented in the headers: chamfers
are for straight edges of planar-faced solids with orthogonal, convex corners
(oblique corners are refused by the geometric gate, not silently mis-built),
and inner face loops are not carried through the chamfer rewrite.

### Faceted curved primitives (84)

The kernel evaluates a solid from its face loops everywhere that matters —
Boolean classification, interference, mass properties, drawing projection,
export. The curved primitives leaned on that being invisible: `makeCylinder`
built box topology (8V/12E/6F) and bound a cylindrical NURBS patch to its four
lateral faces, so the solid *was* a square prism to every computation and only
the renderer disagreed. A cylinder's volume came out 500 against π·r²·h = 785,
a sphere's 192 against 524, and a torus — genus 0 with eight vertices —
enclosed no volume at all. Subtracting one cylinder from another gave 320
where the answer is 503.

- **Curved primitives are now faceted at construction**, built on `SolidSewer`
  from a polygon soup with a tunable `segments` count;
  `PrimitiveFactory::segmentsForTolerance()` inverts a chord-sag budget when
  you want to pick it from a tolerance instead. Facets carry planar patches
  that match their loops, so the B-Rep is exactly what the rest of the kernel
  treats it as. Volumes converge from below — at the default 32 segments a
  cylinder is within 0.65% and at 128 within 0.04% — and Booleans on curved
  solids work: boring a cylinder yields a real tube, correct to the facet
  error rather than 36% light. The torus is now a genuine genus-1 shell.
- **Facets remember what they approximate.** `topo::Face::analyticSurface` and
  `topo::Edge::analyticCurve` record the ideal geometry a facet stands in for,
  distinct from the carrier that actually bounds it. That is what keeps a
  cylindrical mate frame and a radial dimension resolvable from a single pick
  on one facet; binding the cylinder to a planar quad instead would put
  display and computation straight back out of step.
- **A rendering defect fell out of the same diagnosis.** `SolidTessellator`
  emits a face's whole *untrimmed* carrier whenever that carrier is curved, so
  faces sharing a surface each re-emitted all of it: a sphere drew six
  overlapping spheres at 480,000 triangles, and the resulting mesh enclosed
  three times the solid's volume. Faceted primitives take the loop path, so
  each face is emitted once — 960 triangles for that sphere, 124 for a
  cylinder — and the display mesh and the mass-properties integrator now
  agree. Both bounds are pinned by tests.

Two consequences are deliberate and pinned rather than papered over. A torus
is genus 1, so `V - E + F` is 0 and `Solid::checkEulerFormula()` — which has
no genus term — rejects it; `checkManifold()` and the geometric validator both
pass, and the tests assert exactly that. And a faceted cylinder exports to
STEP as planar B-spline faces rather than a rational cylindrical surface: the
written file is exactly the model in memory. Emitting analytic faces would
mean un-faceting on export, which is a separate feature, not a property of the
round trip.

### Blends that work on faceted geometry (85–86)

Faceting the primitives exposed that neither blend operation could act on
them, and checking why turned up defects rather than missing features.

- **Chamfer capacity measured the wrong thing.** It capped the distance at
  half the shortest edge of the adjacent face — a proxy that holds for a box
  and means nothing once faces are faceted. On a 32-sided cylinder the
  shortest edge is the facet chord, so every chamfer over 0.49 was refused
  where the geometry is exact past 4.9; it also rejected a 6mm chamfer on a
  10mm cube that builds correctly. Capacity is now how far the face reaches
  along the offset direction — a necessary condition that never rejects a
  distance that would have worked, with the geometric gate doing the real
  guaranteeing. A cylinder rim now chamfers to the exact truncated cone it
  removes, at any distance up to the cap's inradius: ten times the old range,
  bounded by geometry rather than a heuristic.
- **Vertex identity was inconsistent across the pipeline.** The clip
  deduplicated at 1e-9 × scale, the sewer welds at an absolute 1e-7, and the
  geometric validator judges a loop degenerate relative to its own extent. A
  1.075e-7 segment therefore sewed as legal and validated as degenerate, which
  is what the "self-intersecting loops" refusals actually were. Clipping now
  deduplicates against the loop's extent so all three agree; intersections are
  snapped to the endpoint they land on, since a cut through an existing vertex
  recomputes it with cancellation; and a segment nearly parallel to the clip
  plane no longer has an intersection computed at all, the denominator there
  being the difference of two nearly equal distances. `SolidSewer`'s weld
  tolerance is a named constant so the two cannot drift apart again.
- **A fillet's boundary was the chord, not the arc.** The blend was one flat
  quad joining the two tangent lines, carrying the correct rational-quadratic
  arc surface — the Phase 84 disease in a second place. Every loop-based path
  integrated a chamfer while the renderer drew a fillet: `(1/2)r²L` removed
  instead of `r²(1 − π/4)L`, 2.33 times too much, at every radius. Blends are
  faceted across the arc now, sampled by spherical interpolation between the
  two tangent radii so the points are exact on the arc, with the arc patch
  kept on `Face::analyticSurface`. Corner blends follow the same samples, so
  the spherical patch matches the arcs it joins instead of spanning them
  flat. Volume converges quadratically — inside 0.001% at 16 chords — and
  `analyticSurface` now survives a later operation, which it did not before.
- **Revolve enclosed zero volume (87).** The builder handled exactly one
  input: a four-vertex profile turned a full 360°. For it, it rotated the four
  profile corners to 0° and 180° and built an eight-vertex box from the two
  quads — which are mirror images of each other through the axis, so the box
  is inside-out against itself and encloses nothing. It passed every check the
  suite made (Euler, manifold, `isValid()`, a NURBS surface on all six faces),
  because no test asked for a volume. A torus surface was pasted on all six
  faces, including the two that were the profile itself. Every other input
  returned `nullptr`: any partial angle, any profile that was not a
  quadrilateral. The UI's Revolve command offers 1–360°, so 359 of its 360
  settings silently produced nothing.

  Revolve is now a swept ring stack sewn by `SolidSewer`, the same pipeline as
  the Phase 84 primitives: any closed profile, any angle in (0, 2π], angular
  resolution tunable per call and derivable from a chord-sag budget with
  `Revolve::segmentsForTolerance()`. Partial turns are capped at both ends
  (genus 0); a full turn clear of the axis closes on itself as a genus-1
  torus, which is manifold but which the genus-free Euler check rejects — the
  same documented caveat as `makeTorus`. Profile vertices sitting *on* the
  axis do not move, so their bands collapse to triangles and a profile
  touching the axis sweeps a proper cone. A profile crossing the axis, or one
  whose plane the axis does not lie in, is refused rather than swept through
  itself — one test on the radial vectors catches both.

  Volume converges quadratically to the Pappus value: −9.97% at 8 steps,
  −2.55% at 16, −0.64% at 32, −0.16% at 64, −0.04% at 128, and a partial turn
  carries the same relative error as a full one at equal angular resolution.
  Every band is *exactly* planar — rotating two points about a shared axis
  leaves all four corners on the plane whose normal combines the angular
  bisector with the axis — so unlike the fillet blends, no face here needs an
  approximate carrier. Only the ideals go on the side: the cylinder or cone
  each curved band approximates on `Face::analyticSurface`, the circle each
  profile vertex traces on `Edge::analyticCurve`. A band sweeping a flat
  annulus records neither, because its planar carrier is already exact.
- **Faceting resolution was unreachable from the document (88).** Phases 84–87
  made resolution the knob that decides how close a faceted solid's volume
  gets to the exact one — a cylinder at 32 segments is 0.64% under, at 128
  it is 0.04% — and then left it as a kernel call argument.
  `PrimitiveFeature`, `RevolveFeature` and `FilletFeature` each hard-coded the
  default, so no model could ask for a tighter one, no parameter edit could
  change it, and every reopened file replayed at whatever the build-time
  default happened to be. Accuracy was, in effect, a compile-time constant.

  The count is now a parameter of the feature that owns it: `segments` on
  curved primitives and on revolves, `arcSegments` on fillets, reported by
  `parameters()` and validated by `setParameter()` (three steps is the fewest
  that bounds a volume; one chord is the degenerate blend that removes a
  chamfer's worth of material). That is the path the UI's parameter editor and
  the save format already went through, so it became editable and persistent
  in one move rather than two. A box is exact, so it reports no resolution and
  refuses one.

  It is written into the JSON envelope and rides the FlatBuffers container
  with it. Files written before the field existed simply lack the key and load
  at the feature's default, which is what they were built with.

## Unreleased — Kernel hardening (post-1.0 review response)

Response to the external senior review: fix the Boolean/kernel reality gap
first, then the architecture boundary, interop pinning, and CI enforcement.

- **Boolean rework (the headline).** `BooleanOp` is now a BSP-tree CSG
  pipeline: the trimmed boundary is extracted from face loops
  (`BoundaryMesh`, with ear-clip triangulation of non-convex faces and
  global outward orientation), faces are split along the other solid's face
  planes with exact fragment classification and coplanar-face resolution
  (`MeshCsg`), and selected fragments are welded, T-junction-freed, and sewn
  into a manifold half-edge solid (`SolidSewer`).  Results carry TopologyID
  provenance, chain into further Booleans, and round-trip through STEP.
  Volumes are closed-form-exact for planar-faced solids (unions, subtracts,
  intersects, cavities, through-holes, multi-shell operands).  Curved faces
  participate as their loop polyhedra — analytic surface–surface
  intersection remains future work.
- **Faithful boundary evaluation everywhere.**
  `ExactPredicates::tessellateSolid` and `SolidTessellator` no longer
  tessellate the over-covering bounding-rectangle surface patches for planar
  faces (or re-emit shared curved surfaces once per face) — classification,
  display, interference, and export now agree with the actual solid.
- **CSG-exact interference.** `InterferenceChecker`'s narrow phase is now
  "does the Boolean intersection enclose volume", replacing edge-crossing
  heuristics that missed exactly-grazing symmetric configurations; surface
  touching no longer counts as interference, and cavities are respected.
- **Modeling kernel decoupled from render.** The mesh POD moved to
  `hz::geo::MeshData`; `render::MeshData` is an alias.  `hz_modeling` and
  `hz_document` no longer link `Horizon::Render`.
- **STEP interop pinned by fixtures.** Hand-authored third-party-style
  Part-21 fixtures (FreeCAD/OCC and SolidWorks formatting, analytic
  PLANE/LINE geometry, assembly product structure, `BREP_WITH_VOIDS`) with a
  drop-in directory contract for real vendor exports; documented
  limitations are enforced as tests, and restyled-reimport tests pin parser
  robustness (comments, reflow, entity reordering).
- **Adversarial model suite.** Hole patterns against multi-shell operands,
  nested pockets, non-convex bracket extrudes, shelled containers, Boolean
  volume-conservation fuzzing, and long feature chains — all with
  closed-form expected volumes.  The suite exposed and pins a real
  `ChamferOp` defect (combinatorially valid topology with geometrically
  inconsistent loops; volume integrator undercounts) for a future rebuild on
  `SolidSewer`.
- **clang-tidy is now a gate.** CI fails on any `bugprone-*`/`performance-*`
  finding outside a 10-check known-dirty backlog (which stays visible as
  advisory warnings).
- **Honest maturity documentation.** README gained a per-module Feature
  Maturity table (stable / experimental / prototype) and no longer implies
  1.0 production readiness.

Adversarial-review follow-ups (self-verified from code after the review's
automated verify pass was cut short):

- Fixed a self-inflicted regression: `ExactPredicates::tessellateSolid` (the
  `DrawingProjection` hidden-line occluder) had been switched to pure
  loop triangulation, which collapses curved solids — a torus's eight ring
  corners are coplanar — to a flat, zero-volume mesh.  It now delegates to
  `SolidTessellator`, keeping smooth surface tessellation for curved faces
  and restoring the `tessTol` control.
- The `checkManifold()` output contract now applies on **every** `BooleanOp`
  path, including the disjoint fast paths (previously only the CSG path was
  gated).
- Tolerance stack made consistent: the CSG on-plane epsilon is exported as
  `kCsgPlaneEps`, and `BooleanOp` welds CSG fragments at that tolerance so
  seams the splitter is allowed to open are always reconcilable during
  sewing.
- `SolidSewer`'s degenerate-face area test is now computed relative to the
  loop's first vertex, so a far-from-origin loop's true zero area no longer
  drowns in `R²` cancellation noise.
- Documented (in headers) the remaining known limitations the review
  surfaced: unbalanced BSP recursion depth, discarded face inner-loops on
  inputs, greedy twin pairing at non-manifold edges, and Booleans against
  coarse-box-topology curved primitives (torus/revolve).

## 1.0.0 — Production readiness

All 80 roadmap phases (plus the 61b sheet-metal insert) delivered. ~900
automated regression tests pass across the Windows, Ubuntu, and
AddressSanitizer CI gates, with clang-format and clang-tidy checks. GPU paths
were verified on an RTX 5070 Ti via headless Vulkan.

### Era 0 — Foundation (Phases 1–30)

2D drafting application: math library, OpenGL 3.3 renderer, camera/grid/Qt
shell; document model with undo/redo, selection, snapping, native `.hcad` JSON
format and DXF I/O. Full drawing toolset (line, arc, circle, rectangle,
polyline, ellipse, spline, text, hatch), editing tools (offset, trim, fillet,
chamfer, break, extend, stretch, mirror, rotate, scale, arrays), dimensions and
annotations, a Newton–Raphson/LM constraint solver, blocks/components, layers
with ByLayer inheritance, line types via GPU shader, box selection, grouping,
UI modernization (dark theme, ribbon), an R\*-tree spatial index, parametric
sketch solving, an expression engine, and Linux CI.

### Era 1 — Geometry kernel (Phases 31–40)

NURBS curves and surfaces with adaptive tessellation; a half-edge B-Rep with
TopologyID genealogy and Euler operators; primitives; extrude and revolve;
Boolean operations; fillet/chamfer; feature-tree UI; viewport polish; kernel
hardening.

### Era 2 — Assemblies, interop & scripting (Phases 41–52)

Multi-document architecture with FlatBuffers `.hzpart`/`.hzasm` formats and
lightweight/resolved assembly loading; assembly mates (8 types, 6-DOF solver);
loft and sweep; shell and draft; linear/circular patterns; reference geometry;
Python scripting (embedded CPython via pybind11); collision detection;
measurement and mass properties (Eberly integrals); STEP AP242 import/export
(in-house ISO 10303-21); native binary format with zero-copy tessellation
cache; stabilization (sparse assembly solve, Boolean robustness, memory
guards).

### Era 3 — Professional workflow (Phases 53–64)

2D drawing generation (hidden-line projection, standard/section/detail views,
`.hzdwg`); GD&T feature control frames; BOM and balloons; sheets and title
blocks; in-house FEA (linear static and steady-state thermal); PDM local
version control and multi-user vault locking; advanced fillets (variable-radius
+ spherical corner blends) and drawing section views (61); sheet-metal core
(bend allowance/K-factor/flat pattern, 62) and 3D flange bodies (61b); Python
API phase 2; end-to-end stabilization.

### Era 4 — Cloud, rendering & market parity (Phases 65–80)

Rendering abstraction layer (`RenderBackend`) with OpenGL and staged Vulkan
backends; GPU compute NURBS tessellation (SPIR-V, verified GPU≡CPU); PBR
material library with IBL-lite ambient; an in-house CPU Monte Carlo path tracer;
local-first cloud sync of vault revisions; live-collaboration sessions with
feature-level token locking; CAM (2.5-axis toolpaths + G-code); kinematics
(forward + CCD inverse); advanced simulation (modal + stress-life fatigue);
configuration management (design tables); surfacing (Coons patches, 75); glTF
2.0 GLB export (76); localization infrastructure with starter catalogs (77);
large-assembly instancing + frustum culling (78); a zero-code-execution plugin
registry with a fail-closed permission model (79); and 1.0 release prep (80).

### Notable deviations from the roadmap (in-house instead of a dependency)

- **STEP** — in-house ISO 10303-21 Part-21 reader/writer rather than
  STEPcode/OCCT.
- **Ray tracing** — in-house Monte Carlo path tracer rather than Embree.
- **CAM** — closed-form 2.5-axis toolpaths rather than OpenCAMLib (general
  free-form pocketing/waterline staged behind that integration).

### Deferred beyond the 1.0 code kernel

Phase 73's CFD (deferred by the roadmap itself), and the productization tail of
Phase 80 — signed installers (MSI/AppImage/DMG), the hosted plugin marketplace,
and published SolidWorks/FreeCAD benchmarks — are future work, not code slices.
