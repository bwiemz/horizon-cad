# Milestone 1 — Trust (Phases 97–101): Implementation Plan

Roadmap: [2026-09-23-production-readiness-roadmap.md](../specs/2026-09-23-production-readiness-roadmap.md)

## Goal

No known path loses user work; malformed files produce error messages instead
of crashes, hangs or silent partial loads; the warning and sanitizer gates
actually gate; a crash leaves a log.

## Phase 97 — Build integrity

**Files:** `cmake/CompilerWarnings.cmake`, `cmake/Sanitizers.cmake`,
`CMakeLists.txt`, `CMakePresets.json`, `vcpkg.json`, `.gitattributes`,
`.gitignore`, `.github/workflows/ci.yml`, plus every source file the new
warnings flag.

1. Apply `hz_set_warnings()` and (when `HZ_ENABLE_SANITIZERS`)
   `hz_enable_sanitizers()` to every first-party target from the root
   `CMakeLists.txt`, after all subdirectories are added, by walking
   `BUILDSYSTEM_TARGETS` recursively — new modules are covered without opting in.
2. Warning set: GCC/Clang `-Wall -Wextra -Wpedantic -Wconversion`; Clang also
   gets `-Wno-sign-conversion -Wno-shorten-64-to-32` to hold the same policy
   as GCC. MSVC `/W4 /permissive-`. `HZ_WARNINGS_AS_ERRORS` adds
   `-Werror`/`/WX`.
3. Sanitizers: `-fsanitize=address,undefined -fno-sanitize-recover=undefined
   -fno-omit-frame-pointer`, linked with `-fsanitize=address,undefined`.
4. Fix every warning surfaced (≈25 unique sites: unused variables/parameters,
   a dead lambda, `Eigen::Index` narrowing, a deprecated Qt signal, dangling
   `else` around gtest macros, `class`/`struct` tag mismatch on `Solid`).
5. CI: `HZ_WARNINGS_AS_ERRORS=ON` for the Linux build and ASan jobs (MSVC
   reports without failing until its backlog is measured); `UBSAN_OPTIONS`
   prints stacks; clang-tidy gets `-Wno-sign-conversion
   -Wno-shorten-64-to-32` so it does not re-report the GCC backlog under
   clang's broader `-Wconversion`.
6. `CMAKE_CXX_SCAN_FOR_MODULES OFF` (no modules are used; scanning adds flags
   clang-tidy cannot parse on CMake ≥ 3.28 + GCC ≥ 14).
7. `.gitattributes`: `* text=auto eol=lf`. `.gitignore`: `.claude/worktrees/`.
8. vcpkg: Qt moves into a default-on `qt` manifest feature; a
   `linux-system-qt` preset turns it off so Linux developers build against an
   installed Qt 6 instead of compiling Qt from source.

**Acceptance:** full suite passes with `-Werror` (GCC 16) and under
ASan+UBSan+LSan locally; `compile_commands.json` shows the sanitizer flags on
every translation unit; clang-tidy with CI's check set reports nothing new.

**Follow-up (tracked, not in this phase):** `-Wsign-conversion` burn-down
(≈560 sites); MSVC `/W4` backlog measured in CI, then `/WX`.

## Phase 98 — Never lose work

**Files:** `src/document/include/horizon/document/UndoStack.h`,
`src/document/src/UndoStack.cpp`, `Document.h/.cpp`,
`src/fileio/include/horizon/fileio/AtomicFile.h` (new),
`src/fileio/src/AtomicFile.cpp` (new), `NativeFormat.cpp`, `DxfFormat.cpp`,
`StepFormat.cpp`, `DrawingDocumentIO.cpp`, `GltfExport.cpp`,
`src/ui/include/horizon/ui/MainWindow.h`, `src/ui/src/MainWindow.cpp`.

1. **Clean state on the undo stack.** `UndoStack::setClean()`,
   `isClean()`, `revision()` (monotonic, bumped by push/undo/redo — used by
   autosave in Phase 101), and a change callback. The clean index is the undo
   depth at the last save; a push that discards the redo stack while the clean
   index lies inside it makes the clean state unreachable.
2. **Document modified state.** `Document::isDirty()` =
   *explicitly marked* ∨ ¬`undoStack().isClean()`. `setDirty(false)` also
   marks the stack clean. Operations that are not yet commands (feature
   add/edit, until Phase 104) keep calling `setDirty(true)`.
3. **Atomic writes.** `io::writeFileAtomically(path, bytes, error*)`: write to
   a uniquely named temp file in the target's directory, flush and close with
   error checks (fsync on POSIX), then rename over the target. On any failure
   the temp file is removed and the original is untouched. Every writer builds
   its output in memory first; JSON is dumped with
   `error_handler_t::replace`, so invalid UTF-8 from imported text cannot
   throw mid-save.
4. **Prompts.** `closeEvent` walks all tabs and, for each modified one,
   focuses it and asks Save / Discard / Cancel; Cancel aborts the quit. Tab
   close offers the same three choices (it offered only Close/Cancel). Save
   failures keep the tab open.
5. **Indicators.** Window title uses `[*]` + `setWindowModified`; modified tabs
   show a trailing `*`. Tabs keep their base title so markers never
   accumulate. `Document::setChangeCallback` fires for undo-stack changes
   *and* explicit `setDirty()` (tools such as ConstraintTool mark documents
   directly), and refreshes both.
6. **UTF-8 paths.** Paths travel as UTF-8 `std::string`; every reader and
   writer opens them through `io::pathFromUtf8`, and the Windows executable
   declares UTF-8 as its active code page in a manifest, so a file under
   `C:\Users\José` can be opened and saved.
7. **MainWindow tests.** A `QApplication`-based test binary on Qt's offscreen
   platform (the first piece of the Phase 112 harness) drives the real window:
   markers follow edits and undo, quit and tab close prompt, and Cancel /
   Discard / Save do what they say. `~ViewportWidget` no longer dereferences a
   null GL context when the widget was never shown.

**Tests:** `tests/document/test_UndoStack.cpp` (new) — fresh stack clean;
push dirty; undo back clean; redo dirty; clean point unreachable after
branching; clear resets. `test_Document` cases through real commands.
`tests/fileio/test_AtomicFile.cpp` (new) — writes and replaces; failure
leaves the original byte-identical and no temp file (read-only directory,
skipped as root/Windows); missing directory fails cleanly.
`test_PartFormat` — a drawing with invalid UTF-8 text saves and reloads.

**Acceptance:** drawing a line marks the tab `*`; undoing it clears the
mark; quitting with modified tabs prompts per tab; a failed save never
damages the existing file.

## Phase 99 — Crash containment & diagnostics

**Files:** `src/app/main.cpp`, `src/ui/include/horizon/ui/Application.h`
(new), `src/ui/src/Application.cpp` (new), `src/ui/src/Logging.cpp` (new),
`src/document/src/FeatureTree.cpp`, `NativeFormat.h/.cpp`, `DxfFormat.h/.cpp`,
`MainWindow.cpp`, `ViewportWidget.cpp`, `GLRenderer.cpp`.

1. **Logging.** Rotating spdlog file sink (`<AppLocalData>/logs/horizon.log`,
   5 MB × 3) plus stderr; `qInstallMessageHandler` routes Qt messages into
   it; startup banner with version, Qt version, OS; flush on warning.
2. **Exception boundary.** `hz::ui::Application : QApplication` overrides
   `notify()`: a `std::exception` escaping an event handler is logged and
   reported once in a dialog, and the event loop continues. A
   `std::set_terminate` handler logs and flushes before aborting.
3. **Feature execution.** `buildWithDiagnostics` catches exceptions from
   `execute()` and records `e.what()` as the feature's failure.
4. **Reasons.** `NativeFormat::load/save/loadAssembly/saveAssembly`,
   `documentFromJson/assemblyFromJson` and `DxfFormat::load/save` take an
   optional `std::string* error` and never throw: parse and type errors become
   the reason ("parse error at line 3, column 5: ..."), and a path that is
   missing or a folder is checked before opening. The UI shows "Could not open
   “name”. <Reason>." and logs it; DocumentManager's loader hooks keep their
   signature (MainWindow's loaders capture the reason).
5. **GL.** `initializeGL` logs the driver and flags a context older than 3.3 or
   shaders that failed to compile; a check shortly after the first show catches
   a context Qt could not create at all (paintGL then never runs). Either way
   the user is told once instead of facing a blank viewport.

**Tests:** feature that throws → build reports the message and no build path
throws; loaders give a reason for a missing file, a folder, bad JSON (with line
and column), foreign JSON, a wrong-typed field (previously an uncaught
`type_error`), a wrong assembly type, a non-DXF file, and a failed save;
`hz::ui::Application` contains a throwing event handler and reports it once;
a viewport on a platform without OpenGL raises the warning. (The version gate
moved to Phase 100 with the rest of the envelope checks.)

## Phase 100 — Hostile-input hardening

**Files:** `NativeFormat.cpp`, `DxfFormat.cpp`, `StepFormat.cpp`,
`Expression.cpp`, `FeatureTree.cpp`, `tests/fileio/test_MalformedInput.cpp`
(new), `tests/fuzz/` (new).

1. `documentFromJson` / `assemblyFromJson`: every `const json` access that can
   miss goes through `.at()` or checked helpers (`readVec2/readVec3`) and the
   whole load sits in one `try` that turns any exception into an error.
2. Version gate: a file newer than `kCurrentVersion` is refused with a message
   naming both versions (it used to load with unknown content dropped).
3. Parameter bounds: `segments`, `arcSegments`, pattern `count` clamped to
   documented maxima at `setParameter`, constructors and load;
   `static_cast<int>` of doubles goes through a finite/range-checked helper.
4. Expression parser: recursion depth limit with a parse error.
5. DXF: every section loop stops at EOF; a truncated file is an error, not a
   hang. `std::stod` → `std::from_chars` (locale-independent); `%g` output →
   `std::to_chars`.
6. STEP: constructor exceptions caught per entity into `lastError`; `#id`
   parsing overflow-checked; same `from_chars`/`to_chars` change.
7. Fuzz harnesses (`LLVMFuzzerTestOneInput`) for native JSON, DXF, STEP and
   expressions. Built as libFuzzer targets with Clang when `HZ_BUILD_FUZZERS`
   is on; always built as corpus-replay tests registered with CTest, so the
   seed corpus and every crasher found runs in every CI job.

**Tests:** one regression test per crashing input from the audit (empty
`pullDir`, integer `currentLayer`, truncated DXF at two points, degree-0
B-spline, 200 000-deep expression, `count: 2e9`, `segments: 1e9`,
newer version).

## Phase 101 — Autosave & recovery

**Files:** `src/ui/include/horizon/ui/RecoveryManager.h` (new),
`src/ui/src/RecoveryManager.cpp` (new), `MainWindow.cpp`, `main.cpp`,
`tests/ui/test_RecoveryManager.cpp` (new).

1. Each session owns `<AppLocalData>/recovery/<session-id>/`, held by a
   `QLockFile`.
2. Every 2 minutes, each modified tab whose undo revision changed since its
   last autosave is written (atomically, native format — DXF drawings recover
   as `.hcad`) with a small JSON sidecar: original path, type, title, time.
3. Clean save or close of a document deletes its recovery file; clean exit
   deletes the session directory.
4. On start, session directories whose lock is stale belong to a crashed
   session: the user is offered to restore them as modified, untitled
   documents ("name (recovered)") or discard them.

**Tests:** round trip through the store; stale-session detection; revision
gating (no write when unchanged); cleanup on save.

## Requirement → task map

| Roadmap requirement (Milestone 1) | Task |
|---|---|
| Warnings on every target, `-Werror` in CI | 97.1, 97.2, 97.4, 97.5 |
| Sanitizers real, UBSan fatal | 97.1, 97.3, 97.5 |
| `.gitattributes`, worktrees ignored | 97.7 |
| System-Qt Linux preset | 97.8 |
| clang-tidy not double-reporting | 97.5, 97.6 |
| `-Wsign-conversion` tracked, not suppressed | 97 follow-up |
| Modified state from undo clean index | 98.1, 98.2 |
| Close prompts across tabs; tab close offers Save | 98.4 |
| `*` markers | 98.5 |
| Non-ASCII paths open and save on Windows | 98.6 |
| Prompts verified against the real window | 98.7 |
| Atomic writes for every writer | 98.3 |
| JSON serialized before open; invalid UTF-8 replaced | 98.3 |
| Rotating file log | 99.1 |
| `notify` override, terminate handler | 99.2 |
| Feature exceptions become feature errors | 99.3 |
| Load/save reasons shown | 99.4 |
| GL failure shown; destructor safe | 99.5 |
| Checked JSON + catch-all | 100.1 |
| Version gate | 100.2 |
| Parameter clamps | 100.3 |
| Expression depth limit | 100.4 |
| DXF EOF / truncation | 100.5 |
| STEP exceptions and overflow | 100.6 |
| Locale-independent numbers | 100.5, 100.6 |
| Regression tests for audited crashes | 100 tests |
| libFuzzer harnesses + corpus replay in CI | 100.7 |
| Autosave to recovery dir | 101.1, 101.2 |
| Recovery offer after unclean exit | 101.4 |
| Recovery files removed on save/close | 101.3 |
