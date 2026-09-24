# Horizon CAD — Production-Readiness Roadmap

**Date:** 2026-09-23
**Status:** Active. Milestone 1 in progress.
**Baseline:** `master` @ `28b4de4` (after PR #57, Phases 81–96)
**Scope:** What stands between the current codebase and a release real users
can trust with their work. This roadmap continues the phase numbering of the
80-phase roadmap and the post-1.0 kernel work, starting at Phase 97.

---

## 1. Where the project actually is

Horizon has a broad, well-tested *kernel*: 1033 tests pass on Linux (GCC 16,
Qt 6.11) as well as on CI, the closed-form geometric tests on Booleans, sweeps,
fillets and mass properties are unusually strong, and the Phase 81–96 notes are
candid about their limits. What it does not yet have is the *product* layer
around that kernel. An audit across five areas (application/UI, file I/O,
geometry kernel, build/release infrastructure, secondary modules/security)
found the gaps below. Each item was verified in the code; file:line references
point at the `28b4de4` tree.

### 1.1 Findings that put user data at risk

| # | Finding | Evidence |
|---|---------|----------|
| D1 | Quitting discards every open document without a prompt. There is no `closeEvent`; File ▸ Exit is wired straight to `QWidget::close`. | no `closeEvent`/`aboutToQuit` in `src/`; `MainWindow.cpp:245` |
| D2 | 2D edits never mark a document modified. Commands mutate `DraftDocument`, which has no dirty flag; `Document::m_dirty` is set only by `addEntity`/`removeEntity`, which the UI never calls. The tab-close prompt therefore never fires for drawings. | `Document.cpp:21,35`; `MainWindow.cpp:685` |
| D3 | Saves truncate the target before serializing. `json::dump()` throws on invalid UTF-8 (reachable from DXF text), leaving a 0-byte file and an exception escaping into the Qt event loop. `file.good()` is checked before `close()`, so a failed final flush reports success. | `NativeFormat.cpp:603,1510`; `DxfFormat.cpp:941`; `StepFormat.cpp:1285` |
| D4 | A truncated DXF hangs the application: `parseEntitiesSection` loops on `while (true)` and re-parses the last pair forever once `readPair` hits EOF. | `DxfFormat.cpp:883-932` |
| D5 | Malformed `.hcad`/`.hzpart` input crashes the app: unchecked `const json` indexing (`pObj["origin"][0]`), type errors outside any `try`, no catch in `load()` or its callers. | `NativeFormat.cpp:802-836, 951-1005, 1059-1066, 1402-1414` |
| D6 | Values in a file can exhaust memory or the stack on open: unbounded `segments`/`count`, `static_cast<int>(double)` on out-of-range values (UB), unbounded expression recursion. | `FeatureTree.cpp:83,154,333`; `Expression.cpp:495-556` |
| D7 | A newer-version file loads silently with its unknown content dropped; the next save destroys it. | `NativeFormat.cpp:786` |
| D8 | No autosave, no crash recovery, no backup-on-save. | — |

### 1.2 Findings that make quality claims untrue

| # | Finding | Evidence |
|---|---------|----------|
| Q1 | Compiler warnings were never enabled: `hz_set_warnings()` existed and nothing called it. | `cmake/CompilerWarnings.cmake` |
| Q2 | The "AddressSanitizer" CI gate never sanitized anything: `hz_enable_sanitizers()` was never called either. | `cmake/Sanitizers.cmake`; CI log has no `-fsanitize` |
| Q3 | The product rebuilds a single body. `Document::rebuildModel` uses `buildWithDiagnostics`, and every solid-creating feature ignores its input ("Boolean combination with inputSolid comes in a future phase"), so a second Extrude *replaces* the first. `buildBodies()` exists but only tests call it. A plate with a hole cannot be modelled in the app. | `FeatureTree.cpp:115-121, 586-592, 1010-1036` |
| Q4 | The 3D ribbon's Boolean/Fillet/Chamfer/primitive commands are hard-coded demos added to the scene only — not in the document, not saved, not undoable, gone on the next rebuild. | `MainWindow.cpp:2115-2251` |
| Q5 | `Solid::checkEulerFormula` tests `V−E+F = 2(S−H)`; Euler–Poincaré is `V−E+F−R = 2(S−G)`. Every face with an inner loop fails, and the check still gates FilletOp output and STEP import. | `Solid.cpp:88-103` |
| Q6 | The README rates "ui / app" *stable*; `src/ui` is 17.7k lines with 6 tests (all for `LocaleManager`). STEP, glTF, STL, CAM, FEA, PDM, scripting, plugins and kinematics are not linked into the application at all. | `src/ui/CMakeLists.txt:115-129` |
| Q7 | `LICENSE` is a GPLv3 preamble fragment with no terms; the README says MIT. | `LICENSE`, `README.md:294` |

### 1.3 What is solid (do not redo)

- Kernel closed-form tests: exact Boolean volumes, convergence asserted as a
  property for primitives/revolve/sweep/loft/fillet/chamfer.
- 2D undo: ~68 undo-stack pushes, `CompositeCommand` for multi-entity edits.
- BinaryFormat runs `flatbuffers::Verifier` before access; the STEP parser caps
  nesting and knot values; the plugin manifest reader is fail-closed.
- Kernel ops take `const Solid&` and hold no mutable globals — safe to call on
  separate inputs concurrently.
- The sketch solver's `SolveResult` (status enum + diagnostics) is the error
  contract the rest of the kernel should copy.
- CI shape: Windows + Linux matrix, clang-format and clang-tidy gates.

---

## 2. Principles

1. **User data first.** Nothing that can lose or corrupt a user's work ships
   behind a feature. Milestone 1 comes before any new capability.
2. **Say why.** Every failure a user can hit carries a reason they can act on
   ("profile has 2 open ends at (3.0, 4.0)"), not "Failed to open file."
3. **Untrusted input is hostile.** Every reader is bounded, checked and fuzzed;
   a bad file is an error message, never a crash, hang or silent partial load.
4. **Gates must gate.** A CI job named for a check must run that check and fail
   on it. No quality claim in the README without a test or gate behind it.
5. **Reachable or hidden.** A capability is either wired into the product with
   tests, or marked library-only — never advertised and unreachable.
6. **Honest-core slices, as before.** Each phase lands implemented, tested and
   documented, with its limits written down, one PR per phase or small group.

---

## 3. Milestones

Effort: S < 1 day, M 1–3 days, L > 3 days.

### Milestone 1 — Trust (Phases 97–101)

*Exit criteria:* no known path loses user work; malformed files produce error
messages; the warning and sanitizer gates are real; crashes leave a log.

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 97 | Build integrity | Apply `hz_set_warnings` / `hz_enable_sanitizers` to every first-party target (GCC/Clang `-Wall -Wextra -Wpedantic -Wconversion`, MSVC `/W4 /permissive-`); `HZ_WARNINGS_AS_ERRORS` ON in the Linux CI jobs; UBSan made fatal (`-fno-sanitize-recover`); fix everything the flags and sanitizers surface; `.gitattributes` (LF policy — the Windows→Linux move produced CRLF churn on all 559 files); ignore `.claude/worktrees/`; a vcpkg manifest feature so Linux developers can build against system Qt; clang-tidy stops double-reporting compiler diagnostics. `-Wsign-conversion` (≈560 sites) is a tracked burn-down, not suppressed per-site. | M |
| 98 | Never lose work | Modified state derived from the undo stack's clean index (covers every command, 2D and 3D, and undo back to the saved state); `closeEvent` prompts Save / Discard / Cancel across all tabs; tab close offers Save; `*` markers in title and tabs; one atomic-write helper (temp file in the target directory, verified flush and close, rename) used by every writer; JSON serialized *before* the file is opened, with invalid UTF-8 replaced. | M |
| 99 | Crash containment & diagnostics | Rotating spdlog file sink in the platform app-data directory; `QApplication::notify` override and `std::set_terminate` handler that log and report instead of aborting silently; feature execution wrapped so a throwing op becomes a feature error; load/save return a result carrying a user-readable reason; GL initialization failure shown to the user; viewport destructor safe without a context. | M |
| 100 | Hostile-input hardening | Checked JSON accessors and a catch-all around document loading; version gate (refuse a newer format with a clear message); clamps on `segments`/`count`/resolution parameters; depth limit in the expression parser; DXF EOF/truncation handling; STEP constructor exceptions and `#id` overflow; locale-independent number parsing/formatting (`from_chars`/`to_chars`); regression tests for every crashing input found by the audit; libFuzzer harnesses for NativeFormat, DXF, STEP and expressions with a seed-corpus smoke run in CI. | M–L |
| 101 | Autosave & recovery | Periodic autosave of modified documents to a recovery directory; recovery offer on the next start after an unclean exit; recovery files removed on clean save/close. | M |

### Milestone 2 — Real parametric parts (Phases 102–106)

*Exit criteria:* a user can model a plate with a hole, fillet an edge, undo it,
save and reopen it — through the UI, with every failure explained.

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 102 | Multi-body regeneration | Product rebuild moves to `buildBodies` semantics; Extrude/Revolve gain an operation mode (New body / Join / Cut / Intersect) executed through `BooleanOp`; features after a failure are flagged, not silently dropped; the document exposes bodies, not a single solid. | L |
| 103 | One result contract for kernel ops | An `OpResult<T>` (value + status + reason) modelled on `SolveResult`, adopted by Extrude, Revolve, Sweep, Loft, Shell, Draft, BooleanOp, Pattern, Fillet, Chamfer; the reasons reach the feature tree and the status bar instead of "Feature 'X' failed to execute". | M |
| 104 | Real 3D commands | Primitives, Boolean, Fillet, Chamfer, Shell, Pattern, Loft, Sweep ribbon commands create document features via dialogs and selection; feature add/edit/delete/reorder/suppress and assembly component/mate edits become undoable commands. | L |
| 105 | Topology correctness gates | Euler–Poincaré with ring and genus terms (valid iff `V−E+F−R` is even and ≤ 2S); `GeometryValidator` gates every solid-producing op, not just Fillet/Chamfer/Sweep; profile validation rejects self-intersection and zero-length extrudes. | M |
| 106 | Persistent naming, first cut | Replace positional `featureID/edgeN` naming with names derived from generating geometry (source profile segment + side), so upstream edits that add a vertex do not silently retarget downstream fillets; wire `TopologyID::resolve` into feature execution; unique face IDs after Booleans. | L |

### Milestone 3 — Interoperability & 2D fidelity (Phases 107–110)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 107 | Import/Export reachable | File ▸ Import/Export submenus for STEP (import/export), STL, glTF, DXF; each import — and each native load — produces a report of what was skipped or approximated. (Since Phase 100 a malformed entity or feature inside an otherwise readable `.hcad` is skipped rather than crashing, but still silently; the report is where the user learns of it before a save drops it for good.) | M |
| 108 | DXF fidelity | LWPOLYLINE bulges (import and export), OCS extrusion, partial ELLIPSE, non-uniform/mirrored INSERT scale, nested INSERTs, POLYLINE/VERTEX, POINT, MTEXT chunk order and `\P`/`%%` codes, full ACI colour table, `$INSUNITS`, `$DWGCODEPAGE` → UTF-8, escaped string output; DXF fixtures authored to the spec for each entity; an import report instead of silent skipping. | L |
| 109 | STEP fidelity | `LENGTH_UNIT` conversion, faces with inner loops (needs Phase 105), per-solid partial import instead of all-or-nothing, round-trip-exact real formatting. | M |
| 110 | 2D correctness pass | Snap tolerance in screen pixels; hidden/locked layers excluded from snapping and trim cutting edges; correct Midpoint/Center snap types; intersection snap; Trim preserves line type and group; zoom-independent pick tolerances. | M |

### Milestone 4 — A product shell that responds (Phases 111–114)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 111 | Application essentials | Resolve duplicate menu/ribbon shortcuts; recent files; window and dock state persistence; Preferences (units, autosave interval, snap); Help ▸ About with version, build and licence information; open files from the command line; every dock reachable from View. | M |
| 112 | Offscreen UI test harness | QTest under `QT_QPA_PLATFORM=offscreen`: MainWindow smoke test, tool-driven edits against a `Document`, modified-state and close-prompt tests, a shortcut-uniqueness test. | M |
| 113 | Render efficiency & high-DPI | Constraint analysis only when the sketch changes; remove the per-frame picking pass nobody reads; evict the GL mesh cache; overlay and picking viewport at device pixels; OpenGL capability check with a clear message. | M |
| 114 | Off-thread regeneration | Rebuild, import and interference run on a worker with progress and cancel; sketches snapshotted into the job; atomic ID counters. | L |

### Milestone 5 — Shippable (Phases 115–118)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 115 | One version | Version from `project()` into a generated header, the About dialog, `--version`, CPack and the installer; CHANGELOG and README reconciled (the CHANGELOG's "1.0.0" vs `0.1.0` everywhere else). | S |
| 116 | Packaging | Fix `include(CPack)` ordering; install rules for the Qt runtime (`qt_generate_deploy_app_script`), translations and licences; icon, `.desktop` and AppStream metadata; Windows NSIS/WiX; Linux AppImage via linuxdeploy; third-party notices. | L |
| 117 | Release pipeline | Tag-triggered workflow producing Release artifacts with checksums; Release-configuration CI jobs; a working vcpkg binary cache (the `x-gha` backend restores nothing — Qt is rebuilt from source in every job); Dependabot for actions. | M |
| 118 | Governance | Licence resolved (**owner decision**); SECURITY.md, CODE_OF_CONDUCT, issue/PR templates, CONTRIBUTING refresh; branch protection on `master` (**owner action** — PR #53 merged red and broke `master`). | S |

### Milestone 6 — Experimental modules made safe (Phases 119–121)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 119 | PDM integrity | Atomic lock acquisition (exclusive-create lock file or temp + rename with a generation counter); corrupt manifest fails closed; SHA-256 content hashes verified on read and push. | M |
| 120 | Scripting & plugin safety | Clear the `doc` global after each run (use-after-free); `HZ_ENABLE_SCRIPTING` off by default until sandboxed; plugin entry re-validated at load time. | S–M |
| 121 | CAM & FEA honesty | G-code preamble with modal resets, spindle/tool, retract-first rapids, parameter validation; FEA refuses non-box solids until real meshing exists; README maturity table marks library-only modules as such. | M |

---

## 4. Sequencing and risk

- **Milestone 1 before everything.** Its phases are independent of the kernel
  work and remove the risks that would make any later feature dangerous to use.
  Phase 97 goes first because warnings and sanitizers find bugs the other
  phases would otherwise inherit.
- **Phase 102 is the largest product risk.** Regeneration semantics change what
  every saved part rebuilds to. Files written before it contain single-body
  trees; the default operation mode for existing Extrude/Revolve features on load
  is *New body*, which reproduces what the old rebuild displayed for the last
  feature while keeping earlier bodies visible rather than discarding them. The
  change is covered by round-trip fixtures of pre-102 files.
- **Phase 106 is open-ended.** Persistent naming is a research problem in every
  CAD kernel; the first cut targets the dominant failure (a profile gaining a
  vertex) and measures the rest.
- **Windows.** Local development has moved to Linux; MSVC coverage comes from
  CI. Anything Windows-specific (UTF-8 paths via a manifest, NSIS) is verified
  there.

## 5. Decisions needed from the owner

1. **Licence.** GPLv3 (what `LICENSE` begins) or MIT (what the README says).
   This decides the Qt obligations the installer must meet (Qt is linked
   statically on the vcpkg Linux triplet). Blocks Phase 116/118, nothing earlier.
2. **Branch protection** on `master` requiring the CI checks. Needs repository
   admin rights.

## 6. Tracking

Each phase lands as its own PR (or a small group), with an entry in the
README's post-1.0 table and CHANGELOG, following the existing per-phase
workflow: implement with tests → full suite → independent review → format →
PR → CI green → merge. Implementation plans live in
`docs/superpowers/plans/`; the plan for Milestone 1 is
[2026-09-23-m1-trust-foundation.md](../plans/2026-09-23-m1-trust-foundation.md).
