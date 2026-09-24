# Horizon CAD — Product Completeness Roadmap

**Date:** 2026-09-24
**Status:** Active. Milestone 7 in progress — plan:
[2026-09-24-m7-no-crash-no-hang-no-lie.md](../plans/2026-09-24-m7-no-crash-no-hang-no-lie.md).
**Baseline:** `master` @ `a9b2421` (after PR #85; Phases 97–121 of the
[production-readiness roadmap](2026-09-23-production-readiness-roadmap.md) are all merged).
**Scope:** What stands between a trustworthy core and a product people can
do real work in. Phase numbering continues at 122.

---

## 1. Where the project is

The production-readiness roadmap made the application safe with a user's
data: saves are atomic, input is bounded and fuzzed, crashes are contained,
long rebuilds run on a worker, and it installs and releases like an
application. A fresh audit across five areas found what remains. It was run
on `625fd62`, the tree #85 merged; each item was checked in the code.

### 1.1 It can still crash, hang or give a wrong answer

| # | Finding | Evidence |
|---|---------|----------|
| C1 | Starting Insert Block while it is active frees the active tool, then calls it. | `MainWindow.cpp:2650`; `ToolManager.cpp:13,23`; `ViewportWidget.cpp:70` |
| C2 | Removing an entity rebuilds the whole R-tree. Undoing a 100k-entity DXF import is 100k rebuilds; deleting 1k entities from a large drawing takes minutes. | `RTree.h:107-121`; `DraftDocument.cpp:14-19` |
| C3 | Finding an entity by id is a linear scan, done per selected entity in box select, grips (every frame), arrays and most commands. | `SelectTool.cpp:214-235`; `ViewportRenderer.cpp:541-543` |
| C4 | Shell rebuilds a new cup from two faces: every hole or boss on the part is dropped, silently. Shell and Draft pick "inward" from a vertex average, which is wrong on non-convex parts. The validator cannot see it: it has no face–face crossing test. | `Shell.cpp:62-78,230-290`; `Draft.cpp:18-40`; `GeometryValidator.cpp:245-405` |
| C5 | One failing feature empties the whole part: the rebuild returns no solid, the viewport goes blank, and the scene then rebuilds the failing model again on the GUI thread. | `FeatureTree.cpp:1326-1334`; `MainWindow.cpp:985` |
| C6 | A few-KB DXF of nested INSERTs expands geometrically: the depth cap bounds depth, not breadth. | `DxfFormat.cpp:1126` |
| C7 | DXF HATCH import merges every boundary loop into one polygon, so islands and edge-defined boundaries are garbled, silently. No HATCH test exists. | `DxfFormat.cpp:606-625` |
| C8 | Cancel does not cancel a STEP import or an interference check; exit then waits for them. | `MainWindow.cpp:1585,1901` |
| C9 | Surface area is overcounted on non-convex faces (a U-shape reads 11 for 7). Every three-edge fillet corner exports a whole stray sphere to STL and glTF. | `MassProperties.cpp:55-56`; `FilletOp.cpp:849`; `SolidTessellator.cpp:76` |
| C10 | A context older than OpenGL 3.3 records the problem and then calls GL 3 functions anyway. | `ViewportRenderer.cpp:89-133,917` |
| C11 | An unwritable recovery folder or a full disk turns autosave off without telling the user; a recovered document that crashes the kernel is offered again, as the default, on every start. | `RecoveryManager.cpp:47-110`; `MainWindow.cpp:1135` |
| C12 | DXF export writes lineweight 370 as `width*100`, which is not a DXF lineweight value. | `DxfFormat.cpp:102-104` |

### 1.2 It cannot yet do the job

| # | Finding | Evidence |
|---|---------|----------|
| P1 | A drawing cannot be printed or exported to PDF, SVG or an image; there is no paper space. DXF is the only way out. | no `QPrinter`/`QPdfWriter` in `src/`; `MainWindow.cpp:406-420` |
| P2 | No typed input while drawing: no coordinates, lengths or angles, no ortho or polar tracking. Line does not chain. Snaps cannot be switched off. | `LineTool.cpp:43-83`; `ViewportWidget.cpp:148-158` |
| P3 | Every sketch is the whole top-level drawing on XY. A profile is one loop: no hole in it, and any text breaks it. Nothing can be sketched on another plane or a face. | `MainWindow.cpp:2854-2881`; `ProfileValidator.cpp:210-275` |
| P4 | No 3D picking. Edges and faces are chosen from coordinate lists; a curved part lists every facet. | `GLRenderer.cpp:712-776` unused; `MainWindow.cpp:132-184` |
| P5 | Loft, Sweep and datums have no command. Revolve's axis is always Y, Extrude always +Z and blind, primitives always at the origin; none of these can be changed after creation, and angles are edited in radians. | `FeatureTree.h:227-285,571`; `MainWindow.cpp:2899,2941,3205` |
| P6 | 3D commands are missing from the command palette and have no shortcuts; Fit All ignores solids; there is no mass-properties command. | `MainWindow.cpp:339-355,2209-2223` |
| P7 | Assembly components land at the origin and cannot be moved, listed or removed; mates take raw topology tags. | `AssemblyDocument.h:34,100,115`; `MainWindow.cpp:1984-1995` |

### 1.3 It will not scale

| # | Finding | Evidence |
|---|---------|----------|
| S1 | The 2D view rebuilds every vertex every frame, one draw call per circle and arc; the text overlay reallocates and re-uploads a full-window image every frame. | `ViewportRenderer.cpp:180-460,893-903` |
| S2 | Adding a feature builds the model two or three times, the first always on the GUI thread; opening a part always builds on the GUI thread; tessellation is never cached. | `MainWindow.cpp:2904,2983-3002,985`; `MainWindow.h:237` |
| S3 | The constraint DOF analysis is a dense SVD on the GUI thread after every edit. | `SketchSolver.cpp:156-157` |
| S4 | The undo stack has no limit; move commands clone every constrained entity; resolved assembly parts are never released. | `UndoStack.cpp`; `ConstraintSolveHelper.cpp:84-108`; `DocumentManager.cpp:48` |

### 1.4 The safety net has holes

| # | Finding | Evidence |
|---|---------|----------|
| N1 | No ThreadSanitizer, though a worker now rebuilds while the GUI edits and 17 process-global ID counters are shared. | `cmake/Sanitizers.cmake`; `NativeFormat.cpp:895,1089,1149` |
| N2 | The OpenGL viewport never runs in CI; `GLRenderer`, `ViewportRenderer` and `Camera` have no direct tests. | `test_OpenGLBackend.cpp:28-30` |
| N3 | Fuzzing only replays 29 seeds; libFuzzer never runs, and CI never compiles with Clang. | `tests/fuzz/CMakeLists.txt:16-18` |
| N4 | No coverage measurement; `ui` has 4.1 tests per 1000 lines, `drafting` 8.4. | — |
| N5 | Windows `/W4` warnings are tolerated; PRs never build Windows Release. | `ci.yml:21-26` |
| N6 | Translations are compiled only if Qt Linguist is found, which CI never installs, so packages likely ship none. | `src/app/CMakeLists.txt:77-97` |
| N7 | Seven empty `catch (...)` blocks swallow errors. | `ChamferTool.cpp:229` and six more |

### 1.5 The kernel's limits (known, documented, still limits)

Fillet and chamfer accept only 90° convex edges between planar faces. Names
of edges after a fillet, and of Revolve/Loft/Sweep/Chamfer faces, follow
storage order. Curved faces from STEP are measured and cut as vertex
polygons. Tolerances are fixed absolute values. These are Milestone 11.

## 2. Principles

The production-readiness principles still hold (user data first, say why,
untrusted input is hostile, gates must gate, reachable or hidden, honest-core
slices). Two more:

7. **Wrong is worse than refused.** An operation that cannot do the job
   correctly refuses with a reason. Silent approximations are bugs.
8. **The user's clock is the GUI thread.** Nothing that grows with the size
   of the model runs on it without a bound.

## 3. Milestones

### Milestone 7 — No crash, no hang, no lie (Phases 122–126)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 122 | 2D crash and hang fixes | C1; R-tree deletion without a rebuild and an id→entity index in `DraftDocument` (C2, C3); stale spatial index after property edits | M |
| 123 | Kernel honesty | Shell refuses what it cannot shell and offsets by winding; Draft and Chamfer take direction from winding (C4); keep the last good solid and mark the failing feature (C5); area and stray-sphere fixes (C9) | M |
| 124 | Hostile files, round 2 | DXF expansion budget (C6); multi-loop HATCH import with tests (C7); valid lineweights (C12); an entity budget on every reader | S–M |
| 125 | Background work that stops | Cooperative cancel for STEP import and interference (C8); wedged-job and trial-build exception safety; autosave failures surfaced; recovery crash-loop guard (C11); GL version guard (C10); swallowed errors (N7) | M |
| 126 | A safety net that catches | TSan job over the threaded paths (N1); coverage report (N4); a Clang job running libFuzzer briefly, plus fuzz targets for the binary format, plugin manifests and PDM files (N3); `/W4` measured (N5); translations shipped or the release fails (N6); CI cache and timeouts | M |

### Milestone 8 — A drawing you can hand over (Phases 127–130)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 127 | Print and PDF | Print and PDF/SVG export of the drawing at a chosen scale and paper size, with real plot line weights and text to scale (P1) | L |
| 128 | Precise input | Typed absolute/relative/polar coordinates and lengths in every draw tool; line chaining; ortho and polar tracking; snaps that can be switched; property-panel geometry editing (P2) | L |
| 129 | Dimensions and text | Dimension style editor; display units in dimensions; baseline and continue dimensions; multi-line text | M |
| 130 | Layers and blocks | Layer rename and line weight; block base point; README claims matched to the product | S–M |

### Milestone 9 — Real part modelling (Phases 131–135)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 131 | Sketches on planes | A sketch mode on XY/XZ/YZ, a datum or a planar face; sketch entities owned by the sketch; profiles with inner loops (P3) | L |
| 132 | 3D picking | Face and edge ids through tessellation; ray picking with highlight; fillet, chamfer, shell and mates chosen by clicking (P4) | L |
| 133 | The missing commands | Loft, Sweep and datum commands; editable definitions (directions, axes, positions) with angles in degrees; rollback bar (P5) | M–L |
| 134 | Extrude and pattern options | Through-all, reverse and symmetric extrude; pattern selected features; placed primitives | M |
| 135 | Finding and seeing | 3D commands in menus, palette and shortcuts; Fit All for solids; orthographic view and display modes; mass-properties dialog; section plane (P6) | M |

### Milestone 10 — Scales to real models (Phases 136–138)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 136 | A 2D view that scales | Cached vertex buffers rebuilt on change, view culling, batched arcs, a text overlay redrawn only when it changes (S1) | L |
| 137 | The GUI thread stays free | One build per added feature, on the worker; open and save serialisation off the GUI thread; cached tessellation (S2) | M–L |
| 138 | Bounded memory | Undo limit; sparse constraint analysis off the paint path; shared meshes between assembly instances; released parts (S3, S4) | M–L |

### Milestone 11 — Kernel depth (Phases 139–142)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 139 | Stable names | Feature-scoped primitive names; fillet keeps untouched edge names; geometry naming for Revolve, Loft, Sweep, Chamfer; a faceted curve as one logical edge | M–L |
| 140 | Fillet and chamfer at any angle | Any dihedral angle between planar faces, concave edges, multi-body parts | M–L |
| 141 | Curved faces measured as curved | Mass properties integrated over the analytic surface; "as modelled" and "ideal" reported separately; STEP curved faces trimmed in (u,v) | L |
| 142 | Boolean robustness | Iterative, balanced BSP; a relative tolerance model; strict robustness tests | L |

### Milestone 12 — Assemblies you can build (Phases 143–144)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 143 | Placing components | Move and rotate components; a component and mate tree; remove; mates by clicking faces (needs 132) (P7) | M–L |
| 144 | Living assemblies | Refresh when a part changes; open a part from the assembly; bill of materials | M |

## 4. Sequencing and risk

- **Milestone 7 first.** Its phases are small, independent, and remove every
  crash, hang and silently wrong answer the audit found.
- **131 and 132 are the largest risks.** Sketch mode touches every 2D tool,
  which today assumes the top-level drawing at z = 0; picking needs face ids
  carried through tessellation. Both change how every 3D command takes input.
- **Milestone 8 and Milestone 9 are independent** and can interleave.
- **Milestone 11 is open-ended.** Each phase measures before and after, and
  refuses what it cannot yet do (Principle 7).

## 5. Decisions needed from the owner

1. **Printing needs Qt PrintSupport** (a vcpkg `qtbase` feature) and, for SVG,
   Qt Svg. Both add to the build and the package.
2. **DWG.** Reading DWG needs a library. LibreDWG is GPL v3, now compatible
   with Horizon's licence; the alternative is to stay DXF-only.
3. **Branch protection, private vulnerability reporting, the release dry run
   and the next version number** are still open from the previous roadmap.

## 6. Tracking

As before: each phase lands as its own PR (or a small group), with an entry
in the README's roadmap table and the CHANGELOG, following the per-phase
workflow — implement with tests → full suite → independent review → format →
PR → CI green → merge. Implementation plans live in `docs/superpowers/plans/`.
