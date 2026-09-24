# Milestone 7: No Crash, No Hang, No Lie (Phases 122–126): Implementation Plan

Roadmap: [2026-09-24-product-completeness-roadmap.md](../specs/2026-09-24-product-completeness-roadmap.md)

## Goal

Everything the audit found that crashes, hangs, or gives a wrong answer
without saying so is fixed or refused. Each fix comes with a test that fails
without it.

## Phase 122: 2D crash and hang fixes

### What the audit found

- **Insert Block started twice crashed.** `onInsertBlock` registered a new
  `InsertBlockTool` under the same name. `ToolManager::registerTool`
  replaced, and so destroyed, the tool that was active. `setActiveTool` then
  called `deactivate()` on it, and so did the viewport, which held its own
  pointer to it. Under AddressSanitizer, the new test fails on the old code
  with a heap-use-after-free at `ToolManager.cpp:24`.
- **Removing an entity rebuilt the whole R-tree.** `RTree::remove` collected
  every entry, cleared the tree and inserted them all again. The following
  each removed entities one at a time, O(n log n) per entity:
  - Delete, Cut and Create Block;
  - undo of a DXF import, a duplicate, a mirror, a rotate, a scale or an
    explode.

  Undoing a 100,000-entity import was about 10¹⁰ insertions.
- **Undo moved deleted entities to the end.** `RemoveEntityCommand::undo`
  used `addEntity`, so a restored entity was drawn over everything else.
- **Finding an entity by id scanned the drawing:**
  - once per selected entity in box selection, group expansion and the
    grip hit test;
  - every frame in `renderGrips`, and in the constraint labels;
  - 49 times in the commands, and on every mouse move in snapping.
- **The spatial index went stale.**
  - Moves and block edits rebuilt the whole index after each change.
  - Eight property commands never updated it: text content, height,
    rotation and alignment; spline closed; ellipse semi-axes and rotation.
    Picking and box selection then worked from the old outline.

### As built

- **R-tree deletion** (`RTree::remove(value, where)`):
  - The entry is found by searching only the nodes that intersect the box
    it was inserted with, and taken out of its leaf.
  - Underfull nodes on the way up are dissolved and their entries inserted
    again (Guttman's CondenseTree), and the root shrinks.
  - Freed nodes are reused.
  - O(log n). A wrong box costs a full search, never a wrong result.
  - `remove(value)` still removes every copy.
  - A randomized test checks 6,000 inserts, removals (with right, wrong and
    no box) and queries against brute force.
- **`SpatialIndex`** remembers the box each entity is indexed under. It can
  remove or update an entity that has moved since it was indexed, and it
  keeps one entry per id.
- **`DraftDocument`:**
  - an id→entity map: `findEntity` / `sharedEntity`, O(1);
  - `removeEntity` returns the entity's position, and `insertEntity`
    inserts at one;
  - `removeEntities` / `restoreEntities` remove or restore a batch in one
    pass, with positions;
  - `replaceEntity` swaps in a new object for grip edits;
  - `updateEntityBounds` re-indexes one entity.

  `rebuildSpatialIndex()` rebuilds both indexes, so it stays a full resync
  point.
- **Commands:**
  - All 49 id scans are lookups.
  - `RemoveEntityCommand::undo` restores the position.
  - A new `RemoveEntitiesCommand` backs Delete and Cut.
  - Moves, block edits and grip edits update only the entities they change.
  - The eight property commands update the index.
- **Selection, grips and snapping:**
  - Box selection, group expansion, the grip hit test, `renderGrips`, the
    constraint labels and the array commands use the lookup.
  - Snapping has an overload that takes the drawing: index, then lookup, so
    it no longer scans the entity list on every mouse move.
- **Insert Block:**
  - `ToolManager::registerTool` stops pointing at a tool it is about to
    destroy.
  - `onInsertBlock` releases the viewport's tool before replacing it.
- **Tests:** 15 new.
  - R-tree:
    - random churn against brute force;
    - node reuse;
    - duplicate values.
  - `DraftDocument`:
    - lookups;
    - order through remove and insert;
    - batch round trip;
    - replace;
    - bounds update;
    - 20,000 removals in reverse order within a time budget.
  - Commands:
    - positional undo;
    - batch undo;
    - a move re-indexes;
    - a text-height change re-indexes.
  - Window:
    - Insert Block three times;
    - delete a box selection, then undo and redo, in order.
- **Not done:**
  - Removing an entity from the middle of the list is still an O(n) shift.
    A batch pays it once; one at a time, from the end, as undo removes, it
    is O(1).
  - The constraint solver's full index rebuild after a solve is left to
    Phase 138.

## Phase 123: Kernel honesty

### What the audit found

- **Shell built a different part.** It made a new cup from two faces: the
  removed cap and an opposite cap with the same number of corners.
  - Everything else on the part was dropped: holes, bosses, pockets.
  - Only the first of several open faces was used.
  - "Inward" was the side facing the profile's centroid. On an L-shaped cap
    that turned the inner-corner edges outward, and the cavity broke through
    the walls.
  - The cavity's walls were always straight down, so a tapered part got the
    wrong wall thickness.
  - The thickness limit was a distance from the centroid, which refused
    walls the profile could take.
- **Draft and Chamfer compared with a centroid too.** Draft judged each face
  against its body's centroid, so hole walls and the inner faces of an L
  leaned the wrong way. Chamfer judged each face against its own centroid,
  which is wrong on non-convex faces.
- **A failing feature emptied the part.** The build returned no solid, the
  viewport went blank, and `rebuildScene` then rebuilt the failing model on
  the GUI thread at every redraw, tab switch and save.
- **Surface area was too high on non-convex faces.** A face without holes
  was fanned from its first corner and the triangles' areas added unsigned.
  An extruded U counted 116 for its 52-square cap.
- **Each three-edge fillet corner carried a whole sphere.** The tessellator
  drew all of it, so every STL and glTF export had seven eighths of a ball
  inside each blended corner.

### As built

- **Shell:**
  - It opens one face. More are refused.
  - It hollows a right prism only: N+2 faces, 2N vertices, quad sides, no
    holes, the base straight below the top. Anything else is refused with
    one message saying why.
  - The inner profile is offset by winding, `axis × edge` on the
    counter-clockwise cap.
  - The profile is then checked: every edge keeps its direction, the polygon
    stays simple, and it stays inside the outer profile. This replaces the
    centroid "inradius". An L with 3-wide arms now takes a 1.4 wall and
    refuses 1.5, where its arms close.
  - The header now describes what Shell does.
- **Draft** takes each face's outward normal from its loop's winding, turned
  over for a body wound inside out. That is judged by the body's signed
  volume, now `topo::signedVolume`, which Pattern shares.
- **Chamfer** uses FilletOp's rule: walking a loop with its normal up, the
  face lies to the left (`topo::loopNormal`). This holds however the body is
  wound.
- **A failed build keeps the part as it stood.**
  `FeatureTree::buildWithDiagnostics` replays the features up to the failing
  one, so a failed build pays twice and a successful one nothing.
  `Document::needsBuild()` counts a failed build as built until the tree
  changes. `rebuildScene` builds only a model that has no solid and has not
  been built.
- **Save caches the part as it is.** Save builds any model that is behind its
  features before writing the tessellation cache. It used to build only when
  there was no solid, so during a rebuild on a worker it cached the part as
  it was before the edit, and lightweight assembly loads showed that. (Found
  in review; the gap predates this phase.)
- **Mass properties** triangulate every face of more than three corners
  (`BoundaryMesh::triangulatePolygon`), not a fan. Volume is unchanged.
- **Fillet corners** carry `NurbsSurface::makeSphereOctant`, an exact
  rational biquadratic eighth of the ball facing the vertex. The whole
  sphere remains the face's analytic surface, which mates and dimensions
  read.
- **The Boolean header** no longer says holes are ignored and split faces are
  never merged back.
- **Tests:** 11 new.
  - Shell:
    - an asymmetric L, with its exact cup volume;
    - the thickness limit, 1.4 accepted and 1.5 refused;
    - a drilled plate refused;
    - two open faces refused.
  - A failing feature:
    - leaves the part before it;
    - leaves nothing when it is the first feature;
    - is not rebuilt until the tree changes.
  - Mass properties: a U-channel's area.
  - A corner blend's mesh area matches the part's.
  - The octant patch itself.
  - Save during a rebuild on a worker caches the part as it is.
- **Not done** (Milestone 11):
  - a real offset-based shell;
  - Draft on selected faces;
  - curved STEP faces are still drawn untrimmed;
  - the validator still cannot see faces crossing each other.

## Phase 124: Hostile files, round 2

### What the audit found

- **Hatch boundaries were garbled.** `parseHatch` took every 10/20 pair in
  the entity after the first as one polygon. That merged the outer boundary,
  its islands, and the seed points after them into one outline, and turned
  edge-defined boundaries (lines, arcs, ellipses, splines) into their
  endpoints. It said nothing, and no test read a hatch.
- **Nested blocks multiplied without bound.** Blocks inserted into blocks are
  flattened. The depth was capped at 16, but not the breadth: ten inserts of
  a block of ten inserts, eight levels deep, is 10⁸ entities from a few
  kilobytes, made on the GUI thread.
- **Lineweights were not DXF lineweights.** Group 370 was written as
  `width × 100` (1.5 as 150), not one of the 24 values DXF allows, and the
  layer table wrote none, though the reader takes it.
- **A duplicate ID hid an entity** (from the Phase 122 review). A native file
  with two entities under one ID gave the second the first's place in the id
  lookup. The first could then be neither selected nor deleted.

### As built

- **Hatch boundaries are read path by path**, in the file's order:
  - 91 paths, each starting at 92;
  - a polyline path's 72/73/93 and vertices, with their bulges followed as
    arcs;
  - an edge path's 93 edges, each by its 72 type: lines; circular and
    elliptic arcs, including clockwise ones, sampled every π/16; splines,
    by their control polygon, with their fit data (97, then 11/21, 12/22,
    13/23) read past.

  The hatch keeps the path the file marks as outer (else the largest). The
  report says "islands inside it left out" when there were more, and
  "curved boundary brought in as segments" when a curve was sampled.
- **Flattening has a budget** of 2,000,000 placements per import. Every
  placement counts, a block within a block as well as an entity: a block
  already in the drawing can be made only of inserts, and walking it
  multiplies as much. It is `DxfFormat::setMaxFlattenedEntities`, for tests
  and embedding applications. Past it, flattening stops, and the report
  lists what was left out.
- **Lineweights:** `dxfLineweight` writes the nearest allowed value other
  than 0. The hairline 0 reads back as no width, ByLayer on an entity, so a
  thin width is written as 5. Layers write theirs: a default-width layer, or
  one with no width, is written as -3 (default), which reads back as the
  default.
- **A duplicate native ID** gives the later entity a new one
  (`DraftEntity::newId()`), and the report says so. Every ID in the file is
  reserved before the entities are read, so the new ID is not one that a
  later entity holds.
- **The other readers' output is proportional to their input.** The native
  format's counts are capped (Phase 100), and the STEP reader has no
  instancing. So the DXF flattening budget is the entity budget that was
  missing.
- **Tests:** 9 new.
  - DXF:
    - a hatch with an island and a seed point;
    - an edge boundary with a half circle;
    - a spline edge with fit data, followed by lines;
    - nested blocks cut at a budget of 1,000;
    - a billion nested inserts with nothing in them, cut at the same budget;
    - every 370 written is valid, and the layer widths round-trip;
    - a thin width is not written as no width.
  - Native:
    - a duplicate ID;
    - a renumbered duplicate does not take a later entity's ID.

  Two new fuzz seeds cover every hatch path and edge type, and nested
  inserts.
- **Not done:**
  - A hatch still has one boundary. Holding islands needs `DraftHatch` to
    take more than one loop.

## Phase 125: Background work that stops

### What the audit found

- **Cancel did not cancel.** A STEP import and an interference check ran on
  workers whose work ignored the flag they were given (C8). Cancel dropped
  the result, but the work ran to its end, and quitting waited for it.
- **A worker that could not start wedged its job.** `BackgroundTask::start`
  and `RebuildJob::start` let `std::thread`'s `std::system_error` escape.
  The job was never finished:
  - an import said "already running" until restart;
  - every rebuild queued behind the first one.
- **The trial build could leave a feature behind.** `addModelFeature` puts
  the new feature into the tree to try it, then takes it out. An exception
  in between left the feature in the tree but not in the undo history, where
  it could never be undone. Combining a feature's body with the part (`combine`)
  was not contained.
- **Autosave failed silently (C11).** An unwritable recovery folder, or a
  full disk, turned autosave off, and only the log said so.
- **Recovery could loop (C11).** A recovered document that crashed the
  application was offered again at every start, with Recover as the
  default. Recovered documents were not snapshotted until the next autosave,
  but the crashed session's copies were deleted at once, so a crash in
  between lost them.
- **An old OpenGL context got OpenGL 3 calls (C10).** Below 3.3 the
  viewport recorded the problem, then built its renderer and drew every
  frame with functions the context may not have.
- **Seven empty `catch (...)` blocks (N7):**
  - Fillet and Chamfer typed input: `std::stod` read "1.2.3" as 1.2, and
    dropped "." without a word.
  - Four in the constraint tool's preview, which could not otherwise tell a
    ref that no longer fits its entity.
  - One around the constraint labels, dead since Phase 122's lookup.

### As built

- **Cooperative cancel.**
  - `StepFormat::load`/`fromString` take the flag. It is checked every
    1,024 instances while parsing, per face while building, and per solid.
    A cancelled read returns nothing, with `lastError()` "cancelled".
  - `InterferenceChecker::check` and
    `AssemblyDocument::measureInterference` take it too, checked before
    each candidate pair.
  - The window passes its task's flag to both.
- **`startWorker`** (`WorkerThread.h`) starts a job's thread. When none can
  be had, it runs the work on the caller's thread and logs it. Both job
  types use it.
- **The trial build** restores the tree however it ends. `applyFeature`
  contains everything, so no build path can throw.
- **Autosave state in the status bar.** `RecoveryManager::problem()` says
  why a session could not start or why the last write failed. A status-bar
  label shows "Autosave off" or "Autosave failed", with the reason in its
  tooltip, until a write succeeds. Applying the preferences keeps a failure
  standing (found in review: it used to hide it). It shows nothing when
  autosave is off in the preferences.
- **Recovery counts.**
  - Each snapshot's sidecar carries `recoveries`.
  - Before opening anything, `noteRecoveryAttempt()` counts one more in
    each claimed sidecar.
  - A recovered tab keeps the count in its own snapshots until it is saved.
  - When any offered document has a count, Later is the default and the
    message names it.
  - Recovered documents are snapshotted in the new session before the old
    copies are deleted.
- **The GL guard.** Below 3.3, or when the shaders fail, `initializeGL`
  stops. `paintGL` then only clears (GL 1.0), and `resizeGL` leaves the
  renderer alone. Checked by hand on a Mesa context forced to 2.1: the
  viewport reports the problem and draws the background. CI's offscreen
  platform has no OpenGL at all.
- **The empty catches.**
  - `TypedLength` reads a typed length whole or refuses it, and says so in
    the prompt. Fillet now shows its radius in its prompt, as Chamfer
    showed its distance.
  - `cstr::pointOf`/`lineOf` return nothing for a ref that does not fit.
    The preview and constraint creation use them.
  - The dead block is removed.
  - `executeMultiContained` keeps its catch-all, now with a comment:
    `buildBodies()`, which only tests call, has no way to report a reason.
- **Tests:** 12 new.
  - `TypedLength`:
    - what is a length;
    - Enter takes or refuses;
    - Chamfer and Fillet refuse "1.2.3" and ".".
  - Refs: one that does not fit gives nothing.
  - STEP read and interference check: each stops when cancelled.
  - With `RLIMIT_NPROC` at 0, `startWorker`, a `BackgroundTask` and a
    `RebuildJob` all finish on the calling thread.
  - Recovery:
    - an unfinished recovery is counted next time;
    - a session that cannot start says why;
    - a failed autosave is shown, survives applying the preferences, then
      clears;
    - a document recovered before is not recovered by default;
    - recovered documents are snapshotted at once, counted.
- **Not done:**
  - A STEP import still reads the whole file into memory before parsing.
  - The trial build still runs on the GUI thread (S2, Phase 137).

## Phase 126: A safety net that catches

### What the audit found

- **No ThreadSanitizer (N1).** A worker rebuilds models while the window
  edits, and imports and interference checks run on workers too, but no job
  looked for data races.
- **Fuzzing only replayed seeds (N3).** libFuzzer never ran, and CI never
  compiled with Clang. The binary format, plugin manifests and the PDM's
  files had no fuzz target at all.
- **No coverage measurement (N4).**
- **The Windows warnings were not counted (N5).** MSVC builds at `/W4`
  without `/WX`, and nothing said how many warnings there were.
- **Packages probably shipped no translations (N6).** Catalogs were compiled
  only when Qt's LinguistTools package was found. vcpkg's Qt has no tools,
  so the release build never compiled them. The executable's `POST_BUILD`
  copy also missed an edited catalog when the executable did not relink.
- **CI hygiene:**
  - a failed job threw away the vcpkg cache it had just built (Qt);
  - no job had a time limit;
  - tests ran one at a time;
  - the workflow had the default token permissions.

### As built

- **Build options:**
  - `HZ_ENABLE_TSAN` (`-fsanitize=thread`, refused together with ASan);
  - `HZ_ENABLE_COVERAGE` (`--coverage`);
  - `HZ_REQUIRE_TRANSLATIONS`.

  `enable_testing()` now comes before `src/`, so a test can be declared
  beside what it checks.
- **Translations:**
  - With no LinguistTools package, any `lrelease` compiles the catalogs:
    `lrelease-qt6`, `lrelease` or `pyside6-lrelease`.
  - A `horizon_translations` target places them next to the executable in
    every build.
  - The `TranslationsAreNextToTheExecutable` test checks each one is there.
  - `HZ_REQUIRE_TRANSLATIONS` fails the configure when no `lrelease` exists.
- **CI (`ci.yml`):**
  - `permissions: contents: read`, and a time limit on every job.
  - The vcpkg cache is restored, then saved even when a later step failed,
    once Configure has succeeded.
  - `ctest --parallel 4`.
  - The Windows and Linux Debug builds require translations: `lrelease`
    comes from PySide6 on Windows and `qt6-l10n-tools` on Linux.
  - The Windows build counts its warnings by code into the run summary.
  - **ThreadSanitizer:** every test but the window tests (label `window`),
    with `tests/tsan.supp`. The job sets `vm.mmap_rnd_bits=28`, which the
    runner's kernel needs for GCC 11's TSan.
    - Qt is not instrumented, so the window tests report Qt's own threads.
      Locally all 46 reports were inside Qt: its thread pool drawing icons,
      a wait condition torn down at exit.
    - `called_from_lib` for QtCore hides Qt's thread starts from TSan's
      registry, which then aborts on a reused thread ID.
    - So the worker tests moved out of the window tests, into
      `test_Workers.cpp` in `hz_ui_tests`, which has no Qt threads.
    - A new test edits a document while its rebuild runs on a worker, and
      another checks a `BackgroundTask` sees its Cancel.
    - Timing tests skip under TSan.
  - **Coverage:** the whole suite, then gcovr. The summary gets a table by
    module, and the HTML report is kept with the run.
  - **Fuzz:** Clang 15 builds the seven targets, which are fuzzed a minute
    each from the committed seeds. Failing inputs are kept with the run.
- **Release (`release.yml`):**
  - both packages require translations;
  - the Linux tarball is checked to hold every catalog;
  - every job has a time limit.
- **New fuzz targets:**
  - `hz_fuzz_binary`: part, assembly and mesh-only reads;
  - `hz_fuzz_plugin_manifest`: the manifest and semver parsers;
  - `hz_fuzz_pdm`: an archive manifest over no blobs, and a lock file.

  Each has seeds from the real writers (`hz_fuzz_seedgen`). They share
  `FuzzTempDir.h` for the readers that take a path.
- **Checked locally:**
  - The three new targets ran a minute each under libFuzzer with ASan and
    UBSan: 67,000 binary inputs, 900,000 manifests and 208,000 PDM inputs,
    with no failure.
  - The suite under ThreadSanitizer (GCC 16, a distribution's Qt): all
    1,307 tests outside the window tests pass.
  - Both translation paths: Qt's LinguistTools and a plain `lrelease`.
- **Not done:**
  - `/WX` on Windows waits for the count to reach zero.
  - Coverage is reported, not gated.
  - A Clang build of the whole project (not only the fuzz targets).
  - Windows Release on pull requests.
  - The OpenGL viewport in CI (N2).
