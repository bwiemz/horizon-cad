# Milestone 4: A Product Shell That Responds (Phases 111–114): Implementation Plan

Roadmap: [2026-09-23-production-readiness-roadmap.md](../specs/2026-09-23-production-readiness-roadmap.md)

## Goal

The window behaves like a finished desktop application:
- its shortcuts work;
- it remembers what was open and how it was laid out;
- its settings can be changed without editing a file;
- it says what version is running;
- a long rebuild does not freeze it.

## Phase 111: Application essentials

### What the audit found

- **Ten common shortcuts did nothing.** Ctrl+Z, Ctrl+Y, Ctrl+S, Ctrl+O,
  Ctrl+N, Ctrl+C, Ctrl+V, Ctrl+D, Ctrl+G and Ctrl+Shift+G were each bound to
  a menu action and again to a separate ribbon action. Qt treats a key bound
  to two enabled actions as ambiguous and runs neither.
- **No persistence or preferences:**
  - no recent files;
  - no saved window or dock layout;
  - no Preferences dialog: the autosave interval could only be changed by
    editing the settings file, and the grid snap spacing was fixed at 1;
  - no About box;
  - no way to open a file from the command line.
- **The View menu** reached two of the three docks; the feature tree was
  missing.

### As built

- **Shortcuts.** The ribbon shows the menu's own action for any command the
  menus already have (the same key sequence), so each shortcut has one
  action. `AppEssentialsTest.EveryShortcutRunsOneCommand` checks every key
  in the window.
- **`MainWindow::openPath()`** opens a file in a tab of its own, or shows
  its tab if it is already open. The file dialog, Open Recent and the
  command line all use it.
- **Recent files** (`RecentFiles`, setting `files/recent`):
  - the ten newest files opened or saved, newest first, without duplicates;
  - File ▸ Open Recent is rebuilt each time it opens;
  - a file that is gone is taken off the list, and the user is told;
  - Clear Recent Files empties it.
- **Window layout.** On close, `saveGeometry()` and a versioned
  `saveState()` go to `window/geometry` and `window/state`; they are
  restored at start. The property and layer docks now have object names;
  the feature tree already had one.
- **Preferences** (`Preferences`, `PreferencesDialog`; Edit ▸ Preferences…,
  Ctrl+,):
  - autosave interval (0 turns it off);
  - language, which applies on restart; "System default" stores nothing, so
    `main.cpp` keeps following the system;
  - grid snap spacing;
  - snap reach in pixels;
  - the unit lengths are shown in (mm, cm, m, in, ft), with the number of
    decimals.

  The model stays in millimetres. The display unit is used for the
  status-bar coordinates and the Distance and Area measurements.
  `MainWindow::applyPreferences()` puts the rest into effect at once.
- **Help ▸ About Horizon CAD:**
  - the version and the source revision, taken by CMake at configure time
    (`HZ_GIT_REVISION`);
  - the build type;
  - the compiler, the Qt version it was built against and the one it runs
    on, and the OS;
  - the licence, pointing to the distributed LICENSE file. That file is GPL
    v3 while the README says MIT: an open owner decision.

  Help ▸ About Qt sits next to it.
- **Command line.** `horizon [files...]`, through `QCommandLineParser`
  (with `--help` and `--version`), opens each file once recovery has been
  offered.
- **Tests.** The window tests now write their settings to a temporary
  directory, where they used to write the user's.
- **Not done:** a units preference for the model itself. Documents carry no
  unit, and the model unit is the millimetre; DXF and STEP import convert
  into it.

## Phase 112: Offscreen UI test harness

Under `QT_QPA_PLATFORM=offscreen`:
- a MainWindow smoke test;
- tool-driven edits against a `Document`;
- modified-state and close-prompt tests;
- the shortcut-uniqueness test, which landed in 111.

The harness exists already (`hz_ui_window_tests`, `UiTestSupport.h`: form
fillers, dialog responders, file pickers). This phase fills the gaps.

**As built.**
- **`UiTestSupport.h`** gains two helpers:
  - `ToolDriver` presses, moves and releases at world points through the
    viewport's own mouse handling (`worldToScreen`, then the viewport's
    event handlers), so a tool sees what a real click gives it. The window is
    never shown, so no OpenGL is needed.
  - `ModalCloser` dismisses any modal that opens, and records its title.
- **Ribbon tool actions** now have object names (`tool_line`, `tool_select`…)
  that tests can find.
- **`test_ToolEdits.cpp`** drives the tools:
  - Line: draw a line, then undo and redo it;
  - Circle and Rectangle: the geometry clicked;
  - Select: click a line, press Delete, undo.
- **`test_Smoke.cpp`** triggers every command (166 menu and ribbon actions,
  all but Exit) on:
  - an empty drawing;
  - an empty part;
  - an empty assembly;
  - a drawing with a selection.

  Every dialog they open is dismissed, and none may crash. It passes under
  ASan as well.
- The modified-state and close-prompt tests were already there
  (`MainWindowDocumentsTest`), and the shortcut test landed in 111.

## Phase 113: Render efficiency and high DPI

- Constraint analysis only when the sketch changes.
- Remove the per-frame picking pass nobody reads.
- Evict the GL mesh cache.
- Overlay and picking viewport at device pixels.
- An OpenGL capability check with a clear message.

**As built.**
- **Constraint analysis.** The DOF analysis ran the constraint solver on
  every frame, and a frame is drawn on every mouse move. It now runs only
  when the document or its undo revision changes (`recomputeDOF` keeps
  both), or when the viewport is given a document (`invalidateDOF`). The
  second case matters because a new document can reuse the address of one
  just closed, with the same revision.
- **Picking pass.** The per-frame picking pass is gone: nothing called
  `pickAtPixel`. A pick that needs it can render one on demand.
- **Mesh cache.** A scene is rebuilt with new node IDs on every model
  change, and the GL mesh cache, keyed by node ID, never let go of the old
  ones, so every edit leaked the model's GPU buffers. `renderNodes()` now
  drops the entries whose node has left the scene (`eraseStaleEntries`,
  `SceneGraph::nodeIds`), while the context is current.
- **Device pixels.** `resizeGL` gets logical pixels; the renderer's
  viewport and picking buffer are now sized in device pixels. The text
  overlay is drawn into an image at the device pixel ratio, so dimension
  text is sharp on a high-DPI screen instead of scaled up.
- **OpenGL check.** It was already there from Phase 99: a context below
  3.3, shaders that fail to compile, or no OpenGL at all is reported in
  words.
- **Tests.** Tests cover what does not need a GL context:
  - the analysis runs once per change, not once per frame;
  - `nodeIds` reaches every depth;
  - stale cache entries are dropped.

  The GL paths run only under the opt-in GL runtime tests.

## Phase 114: Off-thread regeneration

- Rebuild, import and interference run on a worker, with progress and
  cancel.
- Sketches are snapshotted into the job.
- Atomic ID counters.

**As built.**
- **Atomic ID counters.** `math::IdCounter` (`next()` is a fetch_add;
  `reserveThrough()` uses compare-exchange) replaces the 17 plain static
  counters: DraftEntity, Constraint, Sketch, SceneNode, and the 13 feature
  classes. A worker can then make objects while the window does.
- **Rebuild.**
  - `FeatureTree::buildWithDiagnostics(BuildControl*)` reports features done
    and total, and checks a cancel flag between features.
  - `Document::applyBuild` takes a build made elsewhere; a cancelled one
    leaves the model as it was.
  - `RebuildJob` snapshots the whole document, sketches included, as its
    native JSON on the GUI thread. The worker loads the snapshot into a
    document of its own and builds that, so nothing it reads can change
    under it. The job carries the document's undo and feature revisions.
  - `MainWindow` sends a rebuild to the worker when the last one took over
    300 ms (`RebuildMode::Auto`; also `Always` and `Never`).
  - A progress bar and Cancel sit in the status bar meanwhile.
  - The result is applied only if the document is where it was; otherwise
    the document as it is now is rebuilt.
  - One job runs at a time. A new request for the same document supersedes
    the running job; a job for another document finishes, and that tab is
    rebuilt when shown if its result was dropped.
- **Import and interference.** `BackgroundTask<Result>` runs any work on a
  worker, and its cancel means "stop waiting".
  - A STEP file of 1 MB or more is parsed on a worker; the part is made on
    the GUI thread when it is done.
  - Interference is split into `interferenceInput()` (copies of the placed
    solids, taken on the GUI thread) and `measureInterference()` (the CSG,
    on the worker, from 2000 faces up). The assembly can be edited, or its
    tab closed, meanwhile.
- The window's destructor waits for its workers.
- **Not done:** a feature being added still trial-builds on the GUI thread,
  to refuse one that fails itself. Only the rebuild that follows moves to
  the worker.
