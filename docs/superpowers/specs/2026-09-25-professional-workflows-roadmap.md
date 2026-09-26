# Horizon CAD — Professional Workflows Roadmap

**Date:** 2026-09-25
**Status:** Proposed. It starts when Milestone 12 of the
[product-completeness roadmap](2026-09-24-product-completeness-roadmap.md)
(Phases 122–144) is merged.
**Baseline:** `prod/phase-144-living` @ `d2abe30`.
**Scope:** The work between a product that can do a job and one that a
mechanical designer could use every day: drawings made from the model,
curved geometry that other CAD tools can read, units and design intent,
real assemblies, a wider kernel, and an application that ships everywhere.
Phase numbering continues at 145.

---

## 1. Where the project is

The product-completeness roadmap removed the crashes and hangs. It added
precise 2D input and printing to PDF, sketches on planes, 3D picking, stable
names, and assemblies that can be placed and kept up to date. A fresh audit
was run on `d2abe30`, and each item below was checked in the code.

### 1.1 A drawing cannot be made from a model

| # | Finding | Evidence |
|---|---------|----------|
| D1 | Hidden-line projection, section views, auxiliary and detail views, sheets, title blocks, model dimensions and balloons all exist, but only as a library. Nothing in the application reaches them. The code that turns a drawing into a 2D document is local to one file. | `DrawingExport.cpp:25,109`; `DrawingView.h:58-90`; `SectionView.h` |
| D2 | Projection draws every B-Rep edge, including the seams between facets inside one curved face. A cylinder draws its facet lines, and it has no silhouette. | `DrawingProjection.cpp:159`; `DrawingProjection.h:48` |
| D3 | Visibility ray-casts 24 samples of every edge against every triangle, with no acceleration structure. | `DrawingProjection.cpp:64-81,133,141` |
| D4 | A `.hzdwg` stores only the part's path and the gap between views, and always regenerates the four standard views. Scale, sheet, views, sections and dimensions are lost. | `DrawingDocumentIO.cpp:19-20,38-50` |

### 1.2 Curved geometry does not leave the application as curves

| # | Finding | Evidence |
|---|---------|----------|
| G1 | STEP export writes each facet as its own face, on its own flat surface, even though every facet records the analytic surface it approximates. Other CAD tools receive a polyhedron. | `StepFormat.cpp:238,331` |
| G2 | STEP import builds a curved face as a grid only when it is bounded by a rectangle of its (u, v). Any other curved face becomes one flat facet, its outline. | M11 plan, Phase 141 "Not done" |
| G3 | STEP assembly structure is not mapped either way: an imported assembly is one part. | `StepFormat.h:37-38` |

### 1.3 Design intent is not captured

| # | Finding | Evidence |
|---|---------|----------|
| I1 | The display unit reaches only the cursor readout and the measure tools. Feature dialogs, typed points, the property panel and mass properties are in millimetres, and files store no unit. | `MainWindow.cpp:4099`; `MeasureDistanceTool.cpp:51`; `MeasureAreaTool.cpp:110` |
| I2 | Feature parameters are plain numbers. Design variables are saved, but nothing lets the user create or edit them, and a parameter cannot be an expression. | `FeatureTree.h:128-131`; `NativeFormat.cpp:621-628` |
| I3 | Configurations (`ConfigurationTable`) are neither saved nor reachable. | `src/fileio` has no reference |
| I4 | A sketch on a face does not follow the face when the part changes, and part edges cannot be projected into a sketch. Extrude has no "up to face". | M9 plan, Phases 131 and 134 "Not done" |

### 1.4 Assemblies stop at one level

| # | Finding | Evidence |
|---|---------|----------|
| A1 | A component can only be a part: no subassemblies, so the BOM has one level. | `MainWindow.cpp` Insert Component filter; `AssemblyDocument.h:32-43` |
| A2 | Components move only by typed values. There is no drag with the mates solved live. | `MainWindow.cpp` Move/Rotate Component |
| A3 | A mate references faces only, planar or cylindrical. Edges, axes, points, datums, cones and spheres cannot be mated, and there are no limit mates. | `AssemblyDocument.h:50-53`; `MateGeometry.h:28-31` |
| A4 | No exploded views, component patterns or mirrored components. | (searched: none) |

### 1.5 The kernel's reach

| # | Finding | Evidence |
|---|---------|----------|
| K1 | Shell hollows only a right prism with one open face. | `Shell.h:18-27`; `Shell.cpp:220-232` |
| K2 | Fillet and chamfer work only on straight edges between planar faces. | `FilletOp.h:30-35`; `FilletOp.cpp:488,588,695,803` |
| K3 | There is no Hole, Mirror, Rib, Thread or Split feature. Patterns are linear or circular only. | `FeatureTree.h:509` |
| K4 | Building a convex solid's BSP is quadratic: 2.6 s for a 2,048-facet cylinder, minutes at 100k triangles. | M11 plan, Phase 142 "Not done" |

### 1.6 The application is not ready to ship everywhere

| # | Finding | Evidence |
|---|---------|----------|
| W1 | Six translation catalogs hold 18 messages each, against about 690 strings in the UI. | `translations/*.ts` |
| W2 | No accessible names anywhere in the UI. | `src/ui` (searched) |
| W3 | Exceptions are contained, but a crash in native code (a signal) leaves no report. | `Application.cpp:25,57` |
| W4 | The Help menu has About and About Qt only; there is no user guide or sample files. | `MainWindow.cpp:996-1002` |
| W5 | No macOS build; installers are unsigned. | `cmake/HorizonInstall.cmake:35`; `release.yml` |

### 1.7 The code and tests that everything above builds on

| # | Finding | Evidence |
|---|---------|----------|
| T1 | `MainWindow.cpp` is 5,488 lines with 191 member functions. Drawings, assemblies and every other workbench would grow it further. | `src/ui/src/MainWindow.cpp` |
| T2 | The OpenGL viewport is never tested in CI: its tests need a display and CI provides none. | `test_ViewportGraphics.cpp:84` |
| T3 | Offset, Break, Extend, Polyline Edit, Mirror, Rotate, Scale, Hatch, Spline and Leader have no test that checks their geometry. The smoke test only runs them. | `tests/ui` (searched) |
| T4 | The UI has 7 tests per 1,000 lines, against 25 in modeling and fileio. | line and test counts per module |

### 1.8 Library modules nobody can reach

Sheet metal, PDM, the Python scripting console and plugin execution, the
path tracer and CAM are tested libraries with no command in the
application. `hz_ui` does not even link simulation, PDM, kinematics, CAM,
plugins or scripting (`src/ui/CMakeLists.txt:147-162`). FEA needs a tetra
mesher for arbitrary solids before a command would be useful; kinematics
needs mates mapped to joints.

---

## 2. Principles

The earlier principles still hold: user data first, say why, untrusted input
is hostile, gates must gate, reachable or hidden, honest-core slices, wrong
is worse than refused, and the user's clock is the GUI thread. Two more:

9. **A model is the source of truth.** Drawings, BOMs and exports are
   derived from it, update when it changes, and never hold geometry of their
   own that can drift from it.
10. **What leaves the application is what was designed.** Exports carry the
    ideal geometry (a cylinder as a cylinder) wherever the format can.
    Faceted output is a named, deliberate fallback.

---

## 3. Milestones

### Milestone 13 — Foundations for new workbenches (Phases 145–146)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 145 | A safety net for the viewport and 2D tools | Run the GL viewport tests in CI under Mesa llvmpipe with Xvfb (T2); geometric window tests for every 2D edit tool (T3); a per-module coverage floor that fails CI when it drops (T4) | M |
| 146 | Workbench controllers | Move the assembly commands out of `MainWindow` into an `AssemblyWorkbench` controller, and the 3D part commands into a `PartWorkbench`, with the window keeping only the shell: tabs, menus, docks and file commands. No behaviour changes; the window tests, with 145's, are the net (T1) | L |

### Milestone 14 — Drawings from the model (Phases 147–150)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 147 | Clean projection | Project logical edges only (no seams inside a face), silhouettes of curved faces, tangent edges shown or hidden, and a BVH for visibility (D2, D3) | L |
| 148 | A drawing sheet | New Drawing from a part: a sheet with a title block, views placed and scaled, printed and exported to PDF, SVG and DXF through the existing 2D paths. Everything saved in `.hzdwg` (D1, D4) | L |
| 149 | Section, detail and dimensions | Section and detail views placed on the sheet; model dimensions and centre lines added to views | M–L |
| 150 | Drawings that follow the model | A drawing regenerates when its part changes (Principle 9), keeping the user's annotations attached; assembly drawings with balloons and a BOM table | M–L |

### Milestone 15 — Curved geometry that travels (Phases 151–153)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 151 | STEP export as designed | Export each logical face once, on its analytic surface (plane, cylinder, cone, sphere, torus, B-spline), bounded by its logical edges as lines, circles and B-spline curves. Faceted output only where no ideal is recorded, and reported (G1, Principle 10) | L |
| 152 | STEP import of trimmed surfaces | A curved face with any boundary is built on its surface, trimmed in (u, v) and faceted within the trim (G2) | L |
| 153 | STEP assemblies | Product structure read into an assembly of parts, and written from one (G3) | M |

### Milestone 16 — Units and design intent (Phases 154–157)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 154 | Document units | A unit per document, saved in the file; every length and angle field shows and takes it, and typed values accept units ("2 in", "30 deg") (I1) | M–L |
| 155 | Variables and equations | A design-variables dialog; any feature parameter can be an expression of variables, re-evaluated on rebuild, with cycles refused (I2) | M |
| 156 | Configurations | A design table of variable values per configuration, saved, with the active configuration chosen in the feature tree (I3) | M |
| 157 | Sketches that follow | A sketch on a face follows the face by its stable name; project part edges into a sketch; extrude up to a face (I4) | L |

### Milestone 17 — Assemblies, part 2 (Phases 158–161)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 158 | Drag with the mates | Drag a component in the view, or with a triad, with the mates solved live and held components refused (A2) | M–L |
| 159 | Subassemblies | An assembly as a component, resolved recursively with cycles refused; a multi-level BOM (A1) | L |
| 160 | More to mate | Mates on edges, axes, points and datums; cones and spheres; distance and angle limits (A3) | M–L |
| 161 | Exploded views and patterns | Named exploded views with steps; linear and circular component patterns (A4). Mirrored components moved to after Phase 162: they need a mirrored body | M |

### Milestone 18 — Kernel reach (Phases 162–165)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 162 | Hole and Mirror | A Hole feature (simple, counterbore, countersink; through, blind, up to face) placed on a face; Mirror of features and bodies (K3); with the mirrored body, mirrored components in an assembly (A4, from 161) | M |
| 163 | Shell, part 2 | Shell with several open faces, on bodies with holes and bosses, by offsetting each face, refusing what it cannot offset (K1) | L |
| 164 | Fillets on curved faces | Edges where a planar face meets a cylinder or cone, including a revolve's rim chain (K2) | L |
| 165 | Booleans at scale | Clip only the polygons near the other operand, found through a bounding-volume tree, so the cost follows the intersection and not the facet count (K4) | M–L |

### Milestone 19 — Ready for the world (Phases 166–169)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 166 | Every string translated | Full catalogs for the six languages, a CI check that fails when a shipped catalog falls below a floor, and accessible names on every control (W1, W2) | M |
| 167 | Crash reports | A native crash handler writing a minidump and the log beside the recovery files, offered for the user to attach on the next start. Nothing is uploaded (W3) | M |
| 168 | Help and samples | A user guide opened from Help, getting-started tours, and sample parts, assemblies and drawings installed with the application (W4) | M |
| 169 | macOS and signed installers | A macOS build in CI and a DMG; signed Windows and macOS installers once the owner provides certificates (W5) | M–L |

### Milestone 20 — The other workbenches (Phases 170–173)

| Phase | Title | Scope | Effort |
|------:|-------|-------|:------:|
| 170 | Sheet metal | A sheet-metal feature (thickness, radius, K-factor, flanges) in the tree, with a flat pattern exported to DXF with bend lines | M–L |
| 171 | Vault | Check out, check in with a message, history with diff and restore, over the PDM library | M |
| 172 | Scripting and plugins | A Python console and Run Script, and plugins run through it, in builds with scripting on, with a warning that scripts are not sandboxed | M |
| 173 | Render Image | A path-traced image of the current view, rendered on a worker and saved as PNG | S–M |

---

## 4. Sequencing and risk

- **Milestone 13 first, net before split.** Drawings and the other
  workbenches would each add over a thousand lines to `MainWindow`. The
  split is mechanical but wide. The window tests, with 145's tests of the
  2D tools and the viewport, make it safe to do. The split waits until no
  open PR touches `MainWindow`, since every one would conflict with it.
- **Milestone 14 is the highest-value product work.** Most of it is
  wiring an existing library, but 147 is real geometry work. A drawing with
  facet lines is not one anyone can hand over.
- **Milestone 15 carries the largest risk.** Writing a logical face as one
  STEP face needs its boundary as curves, which the stable names of Phase
  139 now make possible. Test against files read back by our own importer
  and against reference files from other systems in `tests/fileio/fixtures`.
- **Milestones 16 and 17 are independent** and can interleave with 14.
- **Milestone 18 is open-ended.** As in Milestone 11, each phase measures
  before and after, and refuses what it cannot yet do.
- **Milestone 20 is optional.** Each phase stands alone, and any of them can
  be left out if the owner wants a narrower product.

## 5. Decisions needed from the owner

1. **Printing to a printer** still needs Qt PrintSupport (open since
   Phase 127). Drawings (148) print through PDF until then.
2. **Code-signing certificates** for Windows and an Apple developer account
   for macOS (169).
3. **Crash reports stay local** in this plan: nothing is uploaded. A
   reporting service would be a separate decision.
4. **The Vulkan backend** is compute-only and not on the product path. Keep
   it as a library, or remove it to cut build time.
5. **Milestone 20's scope**: which of sheet metal, vault, scripting and
   rendering belong in the product.
6. Still open from before: DWG, branch protection, private vulnerability
   reporting, a release dry run and the next version number.

## 6. Tracking

As before: each phase lands as its own PR, stacked when they depend on each
other, with an entry in the README's roadmap table and the CHANGELOG. The
per-phase workflow is: implement with tests, run the full suite, get an
independent review, format, open the PR, get CI green, merge. Implementation
plans live in `docs/superpowers/plans/`, one per milestone, written when the
milestone starts.
