# Horizon CAD

An open-source 2D drafting application built from scratch in C++20. Horizon provides a familiar CAD workflow with drawing tools, dimension annotations, constraints, layers, blocks, undo/redo, and file I/O — all rendered with OpenGL and wrapped in a modern Qt6 desktop interface.

## Features

### Drawing Tools
- **Line** (chaining), **Circle** (center-radius), **Arc** (3-click), **Rectangle** (2-corner), **Polyline** (multi-click; closed with Polyline Edit)
- **Ellipse** (center + semi-axes), **Spline** (cubic B-spline with control points)
- **Text** (standalone text entities of one line or several, with height, rotation, alignment)
- **Hatch** (boundary fill: solid, parallel lines, or cross-hatch)
- Snapping to endpoints, midpoints, centers, quadrants, intersections and the grid, each switchable (F3, F9); ortho (F8) and polar tracking (F10)
- Points typed while drawing and dimensioning: `x,y`, `@dx,dy`, `@length<angle`, or a length toward the cursor

### Editing Tools
- Select with **window/crossing box selection** (left-to-right = enclosed only, right-to-left = overlapping), click, and Shift multi-select
- **Entity grouping** (Ctrl+G / Ctrl+Shift+G) — lightweight selection groups without block overhead
- Move, Duplicate, Offset, Trim, Fillet, Chamfer, Break, Extend, Stretch, Mirror, Rotate, Scale
- Copy, cut and paste within Horizon CAD (Ctrl+C/X/V; not the system clipboard)
- Rectangular and Polar array operations
- Polyline editing (add/remove vertices, toggle closed, join polylines)
- Grip editing for direct point manipulation

### Dimensions & Annotations
- **Linear** dimensions (horizontal, vertical, aligned) with auto-orientation detection
- **Radial** dimensions (radius or diameter) placed on circles and arcs
- **Angular** dimensions measuring the angle between two lines
- **Continue** and **baseline** dimensions: a chain on from the last dimension, or each from its first point, stepped out
- **Leader** annotations with custom text
- Text override support on all dimension types
- A dimension style editor (text height, arrows, extension lines, decimal places, and the unit: mm, cm, m, in or ft) kept with the drawing

### Measurement Tools
- **Distance** measurement between two points
- **Angle** measurement between two lines
- **Area** measurement of closed polygons

### Geometric Constraints
- Coincident, Horizontal, Vertical, Perpendicular, Parallel, Tangent
- Equal length, Fixed position, Distance, and Angle constraints
- Constraint solver with real-time visual indicators

### Blocks & Components
- Create reusable block definitions from selected entities, at a base point you choose
- Insert block references with position, rotation, and scale
- Explode block references back to individual entities

### Layers & Properties
- Layer management with visibility, lock, color, line width, and line type
- **Line types**: Continuous, Dashed, Dotted, DashDot, Center, Hidden, Phantom — rendered via GPU shader
- ByLayer property inheritance — entities can inherit color, line width, and line type from their layer
- Property panel for inspecting and editing selected entities, including a line's, circle's or arc's geometry
- Layer panel with add, remove, rename, current layer, and per-layer visibility, lock, color, line weight and line type

### Document System
- Full undo/redo with composite command support
- Native JSON file format (`.hcad`/`.hzpart`, format v18) with backward-compatible versioning
- DXF import/export (LINE, CIRCLE, ARC, LWPOLYLINE, TEXT, MTEXT, SPLINE, HATCH, INSERT)
- New, Open, Save, Save As workflow
- PDF and SVG export of a drawing, on a chosen paper, fitted or to scale, with line weights and text to scale

### Modern UI
- **Dark theme** with Fusion style, custom palette, and QSS stylesheet
- **Ribbon toolbar** with tabbed categories (Home, Draw, Modify, Annotate, Constrain, Block, 3D)
- **Programmatic icons** — 45+ vector-style icons generated via QPainter (no external assets)
- **Keyboard shortcuts** for the common tools (single-key access: L=Line, C=Circle, etc.)
- **Enhanced status bar** showing coordinates, active tool, the drafting aids (as toggles), selection count, tool prompts and what is typed
- **Viewport overlays** — crosshair cursor, snap markers, axis indicator (all GL-rendered)
- **Styled panels** — consistent dark theme across property and layer panels

### Rendering
- OpenGL 3.3 Core Profile with batched line rendering
- Pan, orbit, and zoom camera controls
- Grid overlay with fit-all view
- Real-time snap indicators and tool previews
- GL texture-based text overlay for dimension and entity text
- Selection highlighting and color-aware rendering

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Language | C++20 |
| GUI | Qt 6 (Widgets, OpenGL) |
| Graphics | OpenGL 3.3 Core Profile |
| Build | CMake 3.28+ with vcpkg |
| Serialization | nlohmann/json + FlatBuffers |
| Logging | spdlog |
| Testing | Google Test |

## Building

**Prerequisites:** A C++20 compiler (MSVC, GCC, or Clang), CMake 3.28+, Ninja
(Linux), and vcpkg with `VCPKG_ROOT` set.

**Windows (Visual Studio 2022):**

```bash
cmake --preset debug
cmake --build build/debug --config Debug
ctest --test-dir build/debug -C Debug            # ~1030 tests
./build/debug/src/app/Debug/horizon.exe
```

**Linux:**

```bash
# vcpkg builds every dependency, including Qt (slow the first time; needs
# autoconf, autoconf-archive, automake and libtool for Qt's dependencies).
cmake --preset linux-debug

# ...or use an installed Qt 6 (e.g. the qt6-base / qt6-base-dev package) and
# let vcpkg provide only the small libraries.
cmake --preset linux-system-qt

cmake --build build/linux-system-qt
QT_QPA_PLATFORM=offscreen ctest --test-dir build/linux-system-qt
./build/linux-system-qt/src/app/horizon
```

**Build options:**

| Option | Default | Effect |
|--------|---------|--------|
| `HZ_BUILD_TESTS` | `ON` | Build the Google Test suites |
| `HZ_WARNINGS_AS_ERRORS` | `OFF` | `-Werror` / `/WX` (CI turns it on for Linux) |
| `HZ_ENABLE_SANITIZERS` | `OFF` | AddressSanitizer + UndefinedBehaviorSanitizer (GCC/Clang) |
| `HZ_ENABLE_SCRIPTING` | `OFF` | Embedded Python, when Python 3 and pybind11 are found. Off until scripts are sandboxed: a script can do anything the user can |

## Architecture

Horizon is organized into modular libraries with clean dependency boundaries. Each module is a separate CMake target.

```
src/
  math/          Linear algebra (Vec2, Vec3, Mat4, BoundingBox, Quaternion, Transform),
                 R*-tree spatial index, expression engine
  drafting/      Entity model, layers, snap engine, dimension styles, sketch planes
  document/      Document ownership, undo/redo, sketches, feature tree,
                 multi-document manager, assembly documents, collaboration sessions
  render/        OpenGL/Vulkan backends, camera, grid, shaders, selection, scene graph,
                 PBR materials, CPU path tracer, GPU tessellation, instancing/culling
  constraint/    Geometric constraint solver (Newton-Raphson + LM damping)
  fileio/        Native formats (.hcad, .hzpart, .hzasm), DXF, STEP AP242, glTF/GLB
  geometry/      NURBS curves and surfaces, adaptive tessellation, Coons surfacing
  topology/      Half-edge B-Rep with TopologyID genealogy and Euler operators
  modeling/      Extrude, revolve, Booleans, fillet/chamfer, primitives, loft/sweep,
                 shell/draft, patterns, drawings/GD&T/BOM, sheet metal, mass properties
  simulation/    In-house FEA — linear static, steady-state thermal, modal, fatigue
  pdm/           .hzarchive revision store, multi-user vault locks, local-first cloud sync
  kinematics/    Serial-chain forward kinematics + CCD inverse kinematics
  cam/           2.5-axis toolpaths (contour/drill/pocket), feeds & speeds, RS-274 G-code
  plugin/        plugin.json registry: discovery, validation, fail-closed permissions
  scripting/     Embedded CPython (pybind11) `horizon` module (optional feature)
  ui/            Qt widgets, ribbon toolbar, tools, panels, document tabs, i18n
  app/           Application entry point, dark theme, resources, locale loading
tests/           One suite per module + cross-module integration tests (~1030 tests)
```

## Feature Maturity

"Phase done" in the roadmap below means an **honest core slice** exists —
implemented, tested, and documented — not that the area is
production-hardened. This table is the truthful per-module picture. "Library-only" means the module is built and tested but not linked into
the application, so users cannot reach it yet. Ratings:

- **stable** — exercised broadly by the app and test suite; APIs settled;
  suitable as a foundation for new work.
- **experimental** — a working core with known, documented gaps; expect
  edge-case failures and API movement.
- **prototype** — demonstrates the workflow end to end; known defects or
  major missing pieces are called out in tests/headers; do not rely on its
  output without checking it.

| Module | Maturity | Notes |
|--------|----------|-------|
| math | stable | Closed-form-validated linear algebra, R*-tree, expressions |
| drafting (2D) | stable | The original core of the application |
| document / undo | stable | Feature tree + multi-document are newer but well-tested. Faceting resolution is a parameter of the feature that owns it (`segments` on curved primitives, revolves and sweep path arcs, `arcSegments` on fillets), or a `chordTolerance` from which the count is re-derived at the governing radius on every rebuild, so accuracy is editable and persists across save/reload |
| constraint | stable | Newton-Raphson + LM solver, exercised by sketch tests |
| geometry (NURBS) | stable | Curves/surfaces/tessellation, rational surfaces exact between knots; Coons patches experimental and library-only |
| topology (B-Rep) | stable | `Solid::isValid()` is combinatorial (twin/Euler structure); `GeometryValidator` adds the geometric checks — vertex-chain and twin coincidence, degenerate edges/faces, loop planarity against the face's own carrier, self-intersecting loops, shell closure |
| modeling — Booleans | experimental | BSP-CSG with face splitting, exact fragment classification, coplanar handling, manifold sewing; volumes closed-form-tested. Operates on the faceted boundary, which since Phase 84 is the whole solid rather than a 4-sided caricature of it — a bored cylinder now comes out within the facet error instead of 36% light. Tolerances are relative to the operands, and results are sound (valid and volume-conserving, or refused with a reason) far from the origin, at very small and large scales, and with faces in exact contact (142). A convex solid's BSP is a chain, so very large faceted solids (thousands of facets) are slow. Analytic surface–surface intersection is still future work |
| modeling — extrude/revolve/primitives/patterns | stable | Exact volumes verified. Profile arcs and circles are faceted (`segments` / `chordTolerance`) with their ideal cylinder and circles recorded, so an extruded circle is a true inscribed prism rather than a square. Curved primitives (cylinder/cone/sphere/torus) and revolves are faceted at construction — `segments` is tunable, `segmentsForTolerance()` derives it from a chord-sag budget — so their volumes converge to the analytic value rather than matching it exactly; each facet records the analytic surface it approximates. Revolve takes any closed profile through any angle in (0, 2π], and refuses a profile that crosses the axis or a plane the axis does not lie in |
| modeling — fillet/chamfer | experimental | Chamfer rebuilt on `SolidSewer` (82) with vertex blends (83), and both ops now work on faceted geometry (85–86): a cylinder rim chamfers to the exact truncated cone up to the cap's inradius, and fillet blends are faceted across the arc so their volume converges to `r²(1−π/4)L` instead of integrating as a chamfer. Capacity is the adjacent face's width, not a proxy. Both refuse geometrically invalid output. Straight edges of planar-faced solids at any angle, convex or concave (140), on parts of several bodies; oblique or concave three-edge corners are refused, not mis-built. Fillet chains (two edges per vertex, e.g. a cylinder rim) are mitered |
| modeling — loft/sweep/shell/draft | experimental | Core paths tested; complex inputs unverified. Twisted loft bands are faceted on alternating diagonals, so a loft's volume is the ruled solid's exactly. Sweep carries the profile by a rotation-minimizing (mitered) frame, so its volume is exactly section area × path length; path arcs are sampled at a `segments` resolution. A path that crosses itself far from any single turn is not caught: the geometric gate checks each face on its own, not faces crossing each other. Shell hollows a plain prism only (two caps, straight sides) and refuses anything else, which it would have stripped; Draft tilts every side face, outward by each body's winding |
| modeling — sheet metal | experimental, library-only | Validated against analytic bend formulas; no command in the application yet |
| fileio — native (.hcad/.hzpart/.hzasm) | stable | JSON + FlatBuffers binary, backward compatible v1-v9. Feature faceting resolution round-trips; files written before it existed load at the feature defaults |
| fileio — DXF | stable | Entity subset documented. A hatch keeps one boundary (its islands are reported); blocks nested in blocks are flattened up to 2,000,000 entities, and the rest reported |
| fileio — STEP AP242 | experimental | File ▸ Import ▸ STEP as a New Part, and File ▸ Export ▸ STEP, curved faces written as designed where they can be (cylinders, bosses, holes, extrusions; not yet spheres or apexes, which go out as facets and are listed). Assemblies (153): a file's product structure places its parts, nested assemblies and their units compounded, into a part (a body per placement) or, with File ▸ Import ▸ STEP as an Assembly, kept as an assembly of part files; an assembly tab exports as a STEP assembly (placements by MAPPED_ITEM are not read). Core B-Rep subset with documented limitations (no BREP_WITH_VOIDS — pinned by fixture tests in `tests/fileio/fixtures/step/`); what an import skips or approximates is reported. Plane, cylinder, cone, sphere, torus and B-spline faces are read; an imported part is built in facets that record their surface (141), a face bounded by a rectangle of its surface's (u, v) as a grid, any other curved face as one facet, its outline, and reported. Export writes the facets |
| fileio — glTF/STL | experimental | Export-only, from File ▸ Export |
| fileio — drawings (`.hzdwg`, drawing DXF) | experimental | Drawing ▸ New Drawing from Part makes a sheet tab: the standard views at an ISO scale, with border and title block, saved as `.hzdwg`, drawn again when the part changes, exported to DXF, PDF and SVG at 1:1. Sections and details added in the Drawing menu, captioned and marked on their views; views moved by clicks and removed; dimensions picked on edges, measured from the part and kept by the edges' names; centre lines on holes and bosses. Assemblies are drawn with a balloon on each part and a parts list; what is drawn on a sheet by hand is saved and moves with its view |
| render — OpenGL path | stable | The shipping viewport |
| render — Vulkan / GPU tessellation / path tracer | experimental, library-only | Staged bring-up; the application draws with OpenGL only |
| simulation (FEA) | prototype, library-only | Educational/basic analysis: structured box meshing, linear-static/thermal/modal on tets, validated against analytic bars — not a general-purpose FEA workbench. There is no mesher for arbitrary solids: the analyses mesh a solid's bounding box, so they refuse any solid that does not fill it (anything but an axis-aligned box), and say why |
| pdm / sync / collaboration | experimental, library-only | Deliberately conservative: append-only, pessimistic locks, no merges. A check-out is one exclusive file creation, so two users cannot both win it; content is verified against its SHA-256 on every read and push; an archive or lock that cannot be read fails closed instead of reading as empty or free |
| kinematics | prototype, library-only | Serial chains only |
| cam | prototype, library-only | Contour/drill/rect-pocket slices; no cutter offsetting, gouge or collision checking, stock model or post-processors. A program starts from a known state (modal resets, tool with its length offset, spindle), rapids climb before they cross, and paths or programs that could run a rapid through the cut, cut with the spindle stopped, or carry a non-finite number are refused. Check any program in a simulator before running it on a machine |
| plugin registry | experimental, library-only | Fail-closed validation without code execution; the execution bridge is future work |
| scripting (Python) | experimental, library-only | Embedded CPython, off by default (`HZ_ENABLE_SCRIPTING`) because it is not sandboxed. A script's `doc` is valid only during its run; a copy it keeps raises if used later |
| ui / app | experimental | Qt ribbon shell, i18n catalogs, 2D drafting tools with full undo. The 3D ribbon's commands add undoable features to the part; Extrude and Revolve join, cut, intersect or start a body. Offscreen window tests run every command and drive the drafting tools through the viewport |

## Roadmap

Horizon is under active development. Completed and planned work:

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | Done | Math library, OpenGL renderer, camera, grid, Qt shell |
| 2 | Done | Document model, undo/redo, selection, snapping, file I/O |
| 3 | Done | Arc, rectangle, polyline entities and tools |
| 4 | Done | Duplicate, offset, trim, fillet, mirror |
| 5 | Done | Rotate, scale, copy/paste, arrays |
| 6 | Done | Layers, properties, color-aware rendering |
| 7 | Done | Dimensions and annotations |
| 8 | Done | Constraint solver and geometric constraints |
| 9 | Done | Blocks and components |
| 10 | Done | Text entities |
| 11 | Done | Spline entities (cubic B-spline) |
| 12 | Done | Hatch patterns |
| 13 | Done | DXF import/export |
| 14 | Done | Ellipse entities |
| 15 | Done | Grip editing |
| 16 | Done | Measurement tools |
| 17 | Done | UI modernization (dark theme, ribbon toolbar, icons, shortcuts) |
| 18 | Done | Box/window selection (drag-rectangle with window and crossing modes) |
| 19 | Done | Line types (dashed, dotted, center, hidden, phantom) with GPU shader rendering |
| 20 | Done | Break and Extend tools |
| 21 | Done | Stretch tool (crossing window vertex selection with partial entity deformation) |
| 22 | Done | Chamfer tool, offset ellipse support, fillet/offset lineType fixes |
| 23 | Done | Polyline edit tool (add/remove vertices, toggle closed, join) |
| 24 | Done | Entity grouping (Ctrl+G / Ctrl+Shift+G) with group-aware selection |
| 25-30 | Done | Era 0 — Foundation: R*-tree spatial indexing, parametric sketch solving, expression engine, sketch planes, Linux CI, stabilization |
| 31-40 | Done | Era 1 — Geometry kernel: NURBS curves/surfaces, half-edge B-Rep, primitives, extrude/revolve, Booleans, fillet/chamfer, feature tree UI, viewport polish, kernel hardening |
| 41 | Done | Era 2 — Multi-document architecture: .hzpart/.hzasm formats, DocumentManager, lightweight/resolved assembly loading, document tabs |
| 42 | Done | Era 2 — Assembly mates: 8 mate types, 6-DOF solver with kinematic pre-analysis, TopologyID mate references |
| 43 | Done | Era 2 — Loft & Sweep: ring-stack B-Rep, profile interpolation with twist minimization, translation-transport sweep |
| 44 | Done | Era 2 — Shell & Draft: thin-wall hollowing (ring-stack cup) with inradius guard, mitered-offset face taper |
| 45 | Done | Era 2 — Linear & Circular Patterns: geometry pattern (deep B-Rep clone + transform), instance suppression, genealogy TopologyIDs |
| 46 | Done | Era 2 — Reference geometry: datum planes/axes/points (offset, 3-point, angle, midplane, plane-intersection, line-intersection), non-geometric tree features |
| 47 | Done | Era 2 — Python Scripting API (Phase 1): embedded CPython via pybind11, `horizon` module (math, reference geometry, document authoring), captured stdout & error reporting |
| 48 | Done | Era 2 — Collision detection: R*-tree broad phase, robust mesh-overlap narrow phase (edge/face crossing + containment) with per-solid triangle R*-tree acceleration |
| 49 | Done | Era 2 — Measurement & mass properties: B-Rep volume/area/centroid/inertia (Eberly integrals), material presets, point/angle/segment measurements |
| 52 | Done | Era 2 stabilization — assembly solver sparse-Jacobian + sparse Cholesky (100-part / ~200-mate solve well under 1 s); Boolean robustness guarded (Union/Subtract/Intersect stay valid under perturbation); multi-body feature tree, Boolean-as-parametric-feature, ~5.5× faster Boolean face classification; 50-feature rebuild guard; STEP round-trip idempotence guard; 100-part lightweight-assembly memory guard (RSS-instrumented, < 2 GB bound, no full-load fallback) |
| 53 | Done | Era 3 — 2D Drawing Generation: hidden-line projection (visible/hidden, TopologyID-associated), partial-visibility splitting, standard/auxiliary/detail views + sheet layout, DXF export, Python scripting access, `.hzdwg` part-referencing drawing document, model-driven dimensions + draft/DXF rendering |
| 54 | Done | Era 3 — GD&T: feature control frames (12 characteristics, datum refs, MMC/LMC material-condition modifiers), datum feature symbols, TopologyID-anchored frames/datums rendered onto drawing views + DXF |
| 55 | Done | Era 3 — BOM & Balloons: assembly roll-up (group-by-part, quantities, suppressed-aware) with RFC 4180 CSV export, TopologyID-anchored numbered balloons (leader + circle + item) on drawing views + DXF |
| 56 | Done | Era 3 — Sheets & title blocks: ISO A0-A4 / ANSI A-D paper sizes, ISO 128 line weights, parametric title block, border + populated title-block panel framed into DXF export |
| 57 | Done | Era 3 — FEA (linear static): in-house Eigen solver — constant-strain tetrahedra, structured box mesher, global sparse assembly + fixed-DOF BCs + nodal loads, displacement/von-Mises recovery; solids meshed via bounding-box bridge with face-node selection; validated vs analytical bar-in-tension |
| 58 | Done | Era 3 — Thermal FEA: steady-state heat conduction on the same tet mesh (element Laplacian, non-zero Dirichlet BCs, nodal sources), nodal temperatures + element flux; validated vs analytical 1D profile |
| 59 | Done | Era 3 — PDM local version control: file-system `.hzarchive` revision store (commit/history/checkout, stable FNV-1a content hash, unchanged-content no-op) + semantic JSON diff of feature tree/parameters (added/removed/modified with JSON-pointer paths) |
| 60 | Done | Era 3 — PDM multi-user vault: shared JSON lock manifest, per-document pessimistic check-out/check-in/break-lock with read-modify-write shared-folder concurrency |
| 62 | Done | Era 3 — Sheet metal (core): bend allowance / K-factor, bend deduction, and flat-pattern developed-length unfold; validated vs analytical formulas |
| 63 | Done | Era 3 — Python API (phase 2): sheet-metal, fatigue (`horizon.SNCurve`), CAM (`horizon.cam_contour`/`cam_drill`/`cam_pocket_rect`/`cam_gcode`/`spindle_rpm`), and FEA static + modal + steady-state thermal analysis on the current solid (`doc.static_analysis`, `doc.modal_analysis`, `doc.thermal_analysis`), bound into the embedded `horizon` module |
| 64 | Done | Era 3 — Stabilization: end-to-end integration test composing annotated/framed drawing → DXF round-trip, model → FEA, and PDM commit → semantic diff on one model |
| 71 | Done | Era 4 — CAM (core): 2.5-axis contour + drilling + rectangular pocket-clearing (tool-radius inset zig-zag raster) toolpaths with cutting/rapid length accounting, RS-274 G-code output (G0/G1, modal feed), feeds & speeds (spindle RPM, chip-load feed) |
| 72 | Done | Era 4 — Kinematics: serial-chain forward kinematics (revolute/prismatic joints) + CCD inverse kinematics; validated vs analytical arm poses |
| 73 | Done | Era 4 — Advanced simulation: (a) modal analysis — consistent-mass linear-tet element + generalized eigensolve `K φ = ω² M φ` for natural frequencies and mode shapes, validated by exact scaling invariances (f ∝ √E, 1/√ρ, 1/size), free-free rigid-body modes, and ordering; (b) stress-life fatigue — Basquin S-N curve, cycles-to-failure, Goodman/Soderberg mean-stress correction, and endurance safety factor. Exposed as `doc.modal_analysis`, `horizon.SNCurve`, `horizon.fatigue_safety_factor` |
| 74 | Done | Era 4 — Configuration management: named design-table configurations of parameter overrides driving part-family variants, applied onto the parameter registry |
| 50 | Done | Era 2 — STEP AP242 import/export: in-house ISO 10303-21 writer/reader, lossless (rational) B-spline mapping of the NURBS B-Rep, MANIFOLD_SOLID_BREP reconstruction with manifold validation, PLANE/LINE/CIRCLE/CYLINDRICAL_SURFACE interop for external files, round-trip idempotence guard |
| 51 | Done | Era 2 — Native binary format: FlatBuffers `.hzpart`/`.hzasm` container (file id `HZBF`) wrapping the canonical JSON envelope + typed zero-copy tessellation cache; lightweight mesh loads skip the JSON parse entirely; verifier-gated against corrupt/truncated files; JSON/binary sniffing shares the extensions |
| 61 | Done | Era 3 — Advanced fillets & drawings: variable-radius fillets (radius-stop table → exact ruled conic patches), 3-edge spherical corner vertex blends; fillet reconstruction rewritten to strict half-edge assembly (proper chord-cut ends, perpendicular-convex gate refuses oblique/reentrant edges instead of emitting wrong geometry, Newell-normal side selection); drawing section views (tessellation cut → closed profiles + 45° hatching + retained outline, DXF `Section`/`Hatch` layers); model-driven radial/diameter dimensions (circle-fit measurement, R/⌀ leaders in DXF); cylinder rims now carry true arc curves |
| 61b | Done | Era 3 — Sheet-metal 3D flange bodies: folded solids from the Phase-62 strip model (signed bend angles, concentric inside/outside arc boundaries offset by thickness, arc-polyline bends swept to width via Extrude), flat-pattern outline via developedLength; volumes validated against analytic flats+bend-annulus formulas; en route fixed a pre-existing MassProperties bug — per-triangle outward orientation broke non-convex solids (L-prism reported 4 instead of 12); signed fans + global sign normalization are exact for any simple-polygon B-Rep |
| 65 | Done | Era 4 — Rendering abstraction layer: `RenderBackend` interface (buffers/textures/shaders/passes/draws/compute per roadmap §7.1), `OpenGLBackend` over the existing Qt GL path (incl. GL 4.3 compute dispatch), `VulkanBackend` staged bring-up behind quiet SDK detection (instance + discrete-GPU device + queue + host-visible buffer allocation, verified on RTX 5070 Ti; textures/draws staged pending SPIR-V pipeline); RAL contract tests + opt-in GL runtime tests |
| 66 | Done | Era 4 — GPU compute (core): Vulkan compute pipeline (SPIR-V one-shot dispatch with storage-buffer descriptor sets, fence-synchronized, host readback), NURBS surface evaluation kernel (Cox–de Boor in GLSL, compiled at build time via glslangValidator, matching `NurbsSurface::evaluate`'s two-pass rational semantics exactly), `GpuTessellator` grid evaluator — verified against the CPU reference on plane/cylinder/sphere/torus grids on an RTX 5070 Ti; compute shaders only per roadmap §7.2 (MoltenVK-portable) |
| 67 | Done | Era 4 — PBR materials: named metallic-roughness preset library (brushed aluminum, polished steel, matte plastic, rubber, glass, carbon fiber, wood per roadmap §7.3) on top of the existing Cook–Torrance viewport shader; IBL-lite ambient (hemisphere irradiance + roughness-aware Fresnel environment term, texture-free); HDR environment maps and per-face assignment staged |
| 68 | Done | Era 4 — Ray tracing (core): in-house CPU Monte Carlo path tracer (median-split BVH + Möller–Trumbore, cosine/GGX importance sampling of the viewport material model, sun next-event estimation + hemisphere environment, Russian roulette), deterministic per-pixel seeding (multithreaded output bit-identical), Reinhard+gamma PPM export; Embree deviation documented in the findings note |
| 69 | Done | Era 4 — Cloud sync (core): local-first vault replication (roadmap §7.5) — SyncEngine replicates whole .hzarchive revision histories between the local vault and a SyncEndpoint (file-system transport first, HTTP staged); append-only + hash-verified (divergent histories conflict and stay untouched — sync never merges), pessimistic locks extend to sync (pushes to documents checked out by others are refused); fully offline-capable |
| 70 | Done | Era 4 — Live collaboration (core): transport-agnostic CollaborationSession with feature-level pessimistic token locking per roadmap §7.6 (deliberately NOT OT/CRDT) — tokens cover a feature plus its downstream chain, release on confirm+rebuild, leave releases everything; participant presence (cursor/selection/color); JSON snapshots ready for a WebSocket transport (staged) |
| 76 | Done | Era 4 — Import/export ecosystem (glTF slice): self-contained GLB 2.0 export of tessellated solids/scenes — one shared binary buffer, spec-compliant accessors with position bounds, metallic-roughness material passthrough (BLEND for transparent presets), Z-up→Y-up root transform; STL export existed; IGES/3MF/DWG staged |
| 75 | Done | Era 4 — Surfacing workbench (core): Coons boundary patches from four NURBS curves (discrete Coons control-net formula — boundary curves reproduced exactly, corners interpolated; compatibility validation for degree/knots/corner meeting; rational boundaries + G1/G2 continuity and knit/thicken staged) |
| 78 | Done | Era 4 — Large-assembly optimization (data path): content-hash instance batching (identical parts collapse to one batch — per-instance transforms/materials, deterministic order, collision-guarded FNV identity) + Gribb–Hartmann view-frustum culling of instanced batches (conservative positive-vertex AABB test, world-space boxes per instance, verified on a 10,000-part grid); occlusion culling/progressive loading staged behind the GPU instanced draw wiring |
| 77 | Done | Era 4 — Localization (i18n slice): UI strings already `tr()`-wrapped project-wide; LocaleManager loads `horizon_<locale>.qm` catalogs with BCP-47 fallback (de_DE→de), replace/uninstall semantics, and system-locale/QSettings startup wiring; starter `.ts` catalogs shipped for DE/FR/ES/JA/ZH/KO (roadmap §7.13), compiled via Qt Linguist tools when present (quiet CMake gating); loading tested against a spec-built `.qm` so no qttools dependency; accessibility/high-contrast/F1 help staged |
| 79 | Done | Era 4 — Plugin system (registry slice): `hz::plugin` manifest discovery/validation with zero code execution — `plugin.json` schema (name/semver/entry/permissions), fail-closed explicit permission model per roadmap §7.15 sandboxing (unknown permission = invalid manifest), entry-script containment (relative-only; absolute/drive-relative/root-relative/`..`/symlink escapes rejected via canonicalization), duplicate rejection, disabled-by-default enablement, minAppVersion gating; Python-free so CI always tests it — the hz::scripting execution bridge + marketplace staged |
| 80 | Done | Era 4 — release prep: full roadmap swept to Done, [CHANGELOG.md](CHANGELOG.md) authored era-by-era, ~900 automated regression tests green across Windows/Ubuntu/AddressSanitizer CI, honest per-phase scope + documented deviations (OpenCAMLib, Embree, STEPcode) in the [findings note](docs/superpowers/notes/2026-07-03-era2-roadmap-findings.md); installers/marketplace/published-benchmarks are later productization, not code slices |

All 80 roadmap phases plus the 61b sheet-metal insert have delivered their
core slices — see [Feature Maturity](#feature-maturity) above for what that
does and does not mean per module; Horizon is not a production-ready 1.0 CAD
system. Deferred-by-design items (Phase 73 CFD, and the productization tail
of Phase 80 — signed installers, the hosted plugin marketplace,
SolidWorks/FreeCAD benchmark publication) are called out in the per-phase
notes and remain future work beyond the code kernel.

### Post-roadmap kernel work

With the roadmap's 80 phases delivered, work continues against the gaps the
Feature Maturity table names rather than a fixed phase list. Landed so far:

| Phase | Status | Description |
|-------|--------|-------------|
| 81 | Done | Geometric B-Rep validation: `hz::topo::GeometryValidator` closes the "structural checks never read a coordinate" gap — vertex-chain (`he->next->origin == he->twin->origin`) and twin positional coincidence, degenerate edges/faces, loop planarity against the face's own carrier (curved carriers skipped, not guessed at), self-intersecting loops, closed-shell area-vector balance, plus advisory edge-curve and coincident-vertex reports. Pointed at the kernel it immediately found the sharp-cone primitive to be degenerate — `makeCone(r, 0, h)`, which is what the Cone command asks for, collapsed a 4-point ring to a point — now built with apex topology (5V/8E/5F) |
| 82 | Done | ChamferOp rebuilt on `SolidSewer`: fixes the pinned known defect (a 20mm box with two 2mm chamfers integrated to 3200 instead of 7920) — loop construction is now vertex-driven instead of dropping replaced corners back into the loop, and reconstruction uses the same sewing pipeline as `BooleanOp` instead of an Euler-operator convergence loop that terminated on linkage the polygons never described. Original faces keep their carriers and TopologyIDs. Both ChamferOp and FilletOp now refuse geometrically invalid output |
| 83 | Done | Chamfer vertex blends: two or three selected edges meeting at a corner. A corner is clipped by the planes of every chamfer meeting there, so blends fall out of the single-chamfer code path with no invented corner patch — the standard planar-chamfer result. Validated against inclusion–exclusion volumes for the union of the cutting prisms; chamfering all twelve edges of a cube yields the chamfered cube (18F/32V) at its exact volume |
| 84 | Done | Faceted curved primitives: cylinder/cone/sphere/torus were box topology (8V/12E/6F) wearing a curved NURBS surface, so every loop-based path in the kernel — Booleans, classification, mass properties, export — saw a square prism. A cylinder's volume read 500 against 785, a sphere's 192 against 524, and a torus enclosed nothing at all. They are now tessellated at construction with a tunable `segments` (`segmentsForTolerance()` inverts a chord-sag budget), so volumes converge from below and Booleans on curved solids work — boring a cylinder gives a real tube within the facet error. Each facet records the analytic surface it approximates (`topo::Face::analyticSurface`, `topo::Edge::analyticCurve`), which is what keeps cylindrical mate frames and radial dimensions resolvable from a single pick. En route this removed a rendering defect: `SolidTessellator` emits a curved carrier's whole untrimmed patch per face, so a sphere drew six overlapping spheres — 480,000 triangles, and a mesh enclosing three times the solid's volume — where the faceted sphere needs 960 |
| 85 | Done | Blends on faceted geometry: chamfering a cylinder rim was impossible, for two reasons rather than a missing feature. The capacity check capped the distance at half the shortest adjacent face edge — the facet chord on a faceted solid — refusing anything over 0.49 on a 32-sided cylinder where the geometry is exact past 4.9, and wrongly rejecting a 6mm chamfer on a 10mm cube. Capacity is now how far the face reaches along the offset direction. Separately, vertex identity was inconsistent across the pipeline: the clip deduplicated at 1e-9×scale, the sewer welds at an absolute 1e-7, and the geometric validator judges degeneracy relative to a loop's extent, so a 1.075e-7 segment sewed as legal and validated as degenerate. Clipping now deduplicates against the loop's extent, snaps intersections to the endpoint they land on, and skips intersections on segments nearly parallel to the clip plane |
| 86 | Done | Faceted fillet blends: a blend was one flat quad joining the two tangent lines, carrying a correct rational-quadratic arc surface. The loop was the chord, so every loop-based path integrated a chamfer while the renderer drew a fillet — `(1/2)r²L` removed instead of `r²(1−π/4)L`, 2.33× too much, at every radius. Blends are faceted across the arc (`kDefaultArcSegments`, tunable) with the arc kept on `Face::analyticSurface`; corner blends follow the same samples so the spherical patch matches the arcs it joins. Volume converges quadratically — within 0.001% at 16 chords — and `analyticSurface` now survives a later operation |
| 87 | Done | Revolve rebuilt as a swept ring: the old builder handled exactly one case — a four-vertex profile turned a full 360° — and built an eight-vertex box for it from the profile at 0° and at 180°. That box passed every check the test suite made (Euler, manifold, `isValid()`, surfaces bound on all six faces) while enclosing **zero volume**, because the two quads are mirror images through the axis. Every other input — any partial angle, any profile that was not a quadrilateral — returned `nullptr`, so the UI's Revolve command, which offers 1–360°, produced nothing at all for 359 of its 360 settings. Revolve now sweeps any closed profile through any angle in (0, 2π] at a tunable angular resolution (`segmentsForTolerance()` inverts the same chord-sag budget as the primitives), caps partial turns at both ends, collapses the bands of profile vertices that sit on the axis, and refuses a profile that crosses the axis or a plane the axis does not lie in. Volume converges quadratically to Pappus — −9.97% at 8 steps to −0.04% at 128 — and a partial turn carries the same relative error as a full one at equal resolution. Every band is *exactly* planar (rotating two points about a shared axis leaves all four corners coplanar), so no face needs an approximate carrier; the cylinder or cone each curved band approximates is recorded on `Face::analyticSurface` and the circle each profile vertex traces on `Edge::analyticCurve`, while a band sweeping a flat annulus records nothing because its planar carrier is already exact |
| 88 | Done | Faceting resolution made a feature property: Phases 84–87 turned resolution into the knob that decides how close a faceted solid gets to the exact one — a cylinder at 32 segments is 0.64% under volume, at 128 it is 0.04% — and then left it unreachable from the document layer. `PrimitiveFeature`, `RevolveFeature` and `FilletFeature` each hard-coded the kernel default, so no model could ask for a tighter one, no parameter edit could change it, and every reopened file replayed at whatever the build-time default happened to be. The count is now a real parameter (`segments` on curved primitives and revolves, `arcSegments` on fillets) reported by `parameters()` and validated by `setParameter()`, which is the path both the UI's parameter editor and the save format already go through — so it became editable and persistent in one move. It is written to the JSON envelope and rides the FlatBuffers container with it; files written without the field load at the feature's default, and a box, being exact, reports no resolution and refuses one |
| 89 | Done | Sweeps that follow their path: `Sweep` translated the profile along the path without turning it, so on any turning path the profile went edge-on — an L-shaped sweep's second leg was a zero-thickness sheet, integrating to 40 against 72 with two degenerate faces, while the only test checked topology. `SweepFeature` also swept every path arc across its chord. The profile is now carried by a rotation-minimizing frame — each interior section is the miter cut bisecting the turn — so every lateral face is exactly planar and the volume is exactly section area × path length, asserted on a path turning in three planes. Path doubling back, a profile containing the sweep direction, a turn tighter than the profile, and geometrically invalid output are refused. Path arcs are sampled at a `segments` parameter that is editable and persisted |
| 90 | Done | Accuracy as a distance: a fixed facet count sags r(1 − cos(π/n)), so the default count was twenty times less accurate on a radius-100 cylinder than on a radius-5 one, and the `segmentsForTolerance()` helpers had no caller outside the tests. Curved primitives, revolves, sweeps and fillets take a `chordTolerance` parameter; when positive, the count is derived from it and the governing radius (widest circle, farthest profile vertex, each path arc's own radius, the fillet radius) on every rebuild, so a radius edit keeps the accuracy rather than the count. Persisted as a tolerance; setting the count explicitly returns to count mode |
| 91 | Done | Exact rational surface evaluation: `NurbsSurface::evaluate`'s second De Boor pass used unit weights, dropping the U-direction rationality, so points of a cylinder, sphere, torus or cone between knots were up to ~6% of the radius off the surface (0.30 on a radius-5 cylinder) — tolerated by the geometry tests with 0.2–0.5 tolerances and a comment calling it lost precision. The pass now carries each row's weight sum; all four quadrics are exact to 1e-12 at off-knot samples |
| 92 | Done | Faceted profile arcs: Extrude built a circle profile as box topology from four points with a cylinder surface pasted on (a radius-5 disc extruded 10 integrated to 500 against 785), and every profile arc — in Extrude, Revolve, Sweep and Loft alike — was taken as its chord (a slot's round ends vanished: 80 against 111.4); a circle section made Loft return nothing. One shared sampler now follows arcs and facets circles at `segments` or `chordTolerance`, recording each chord's arc, so an extrusion is the exact inscribed prism, lateral arc facets carry their ideal cylinder and arc chords their circle, a half disc revolves to a sphere, circles loft to a frustum and sweep to a pipe. `ExtrudeFeature` gains `segments`/`chordTolerance` for curved profiles |
| 93 | Done | Patterns merge what overlaps; ideals survive: a pattern kept every instance as its own shell regardless of spacing (merge "deferred"), so three 10mm boxes 5 apart integrated to 3000 against 2000 with faces buried inside the part — and a document test asserted six overlapping shells as correct. Touching or overlapping instances are now unioned; separate ones stay separate bodies. Pattern clones and Boolean results also dropped `analyticSurface`/`analyticCurve`, so a bore or a patterned boss no longer resolved to its cylinder for mates; both now carry the ideals through (Boolean fragments look them up per operand by provenance) |
| 94 | Done | Mitered fillet chains: two selected edges meeting at a vertex were refused, so a faceted cylinder's rim — 32 chords, two at every vertex — could not be filleted, nor two adjacent top edges of a box. Where two edges share one face and the unselected third edge joins their other faces, the blends now meet on the plane bisecting the turn (Phase 89's sweep miter): bands stay planar, the removed volume is exactly cross-section × centroid-path length (asserted to 1e-9 for corner pairs, open chains, a square rim and a cylinder rim), and a turn too tight for the radius is refused |
| 95 | Done | Twisted lofts: a band between a square and the same square turned has non-coplanar corners, so its loop encloses no well-defined volume — mass properties fanned it one way (180.8 at 0.6 rad) while the renderer drew the ruled patch (150.7). Non-planar levels are now cut along their rulings into strips and triangulated on alternating diagonals: every facet is flat, display and computation agree, and since a bilinear patch bounds exactly the mean of its two triangulations, the faceted volume equals the ruled loft's exactly (asserted to 1e-9) |
| 96 | Done | Interference checking reachable and quantified: the Phase 48 checker had no caller outside its tests and reported only whether two solids clash. Pairs now carry the shared volume (Boolean intersection); `AssemblyDocument::findInterference()` places each resolved, unsuppressed component and reports clashing pairs by component, listing unresolved ones as unchecked; assemblies gain a Check Interference command. Face contact is not interference |

### Production-readiness track

The [production-readiness roadmap](docs/superpowers/specs/2026-09-23-production-readiness-roadmap.md)
set out six milestones from Phase 97 on, starting with data safety, to make
the product safe to trust with real work. All six are done.

| Phase | Status | Description |
|-------|--------|-------------|
| 97 | Done | Build integrity: compiler warnings and sanitizers had been defined in `cmake/` but never applied to a target, so the build ran with no warning flags and the CI "AddressSanitizer" job was an uninstrumented Debug build. Both now apply to every first-party target (warnings as errors on Linux CI, UBSan fatal), and the suite is clean under ASan + UBSan + LeakSanitizer. Adds a `linux-system-qt` preset, an LF line-ending policy and consistent clang-tidy flags |
| 98 | Done | Never lose work: quitting discarded every open document without a prompt, and 2D edits never marked a drawing modified at all. The modified state now follows the undo stack (undoing back to the saved state clears it); quitting and closing a tab ask Save / Discard / Cancel; modified tabs show `*`. Every writer replaces its file atomically (temporary + flush + rename) instead of truncating it first — a save that failed half-way, including on DXF text in a legacy code page, used to leave a 0-byte file. Paths are opened as UTF-8, so non-ASCII folders work on Windows. The first offscreen `MainWindow` tests drive all of it |
| 99 | Done | Crash containment and diagnostics: a rotating log file (with Qt's own warnings, the version and the OpenGL driver) replaces two stdout lines a GUI executable discards; an exception escaping an event handler — or a kernel op throwing during a rebuild — is contained and reported instead of terminating with every document open; loads and saves say why they failed ("parse error at line 3, column 5", "it is a folder, not a file") instead of "Failed to open file." and never throw; a viewport without OpenGL says so instead of staying blank |
| 100 | Done | Hostile input: every native-reader access is checked (a missing key on a `const json` was undefined behaviour no `try` could catch), integer and enum fields are range-checked, newer-format files are refused instead of silently losing content, and counts are clamped (a pattern count of 2e9 tried to allocate on open). A truncated DXF no longer hangs; DXF and STEP numbers ignore the C locale ("1.5" read as 1 under de_DE) and STEP reals round-trip exactly; expressions are bounded in depth and size. libFuzzer targets for each reader, with their seed corpus and every crashing input replayed as a CTest test in every build |
| 101 | Done | Autosave and crash recovery: modified documents are snapshotted every two minutes to a per-session recovery directory that a clean exit removes; after a crash the next start offers them back (Recover / Discard / Later), reopened as modified documents marked "(recovered)" that save to their original paths. Session locks tell a crashed session from a running second instance |
| 102 | Done | Multi-body regeneration: every feature that builds geometry replaced the part — a second extrude discarded the first, so a plate with a hole could not be modelled. Extrude, Revolve, Loft, Sweep and primitives now combine their body with the part by an operation — join, cut, intersect or new body — through `BooleanOp`, with closed-form volume tests; the Extrude and Revolve commands ask for it and refuse a cut that would leave nothing. Files written before load every body as its own, instead of only the last |
| 103 | Done | Failures explained: every feature reports why it failed instead of "Feature 'X' failed to execute" — an open profile names where its ends are, a fillet names the edge it cannot find, a Boolean says whether the result is empty or could not be sewn — and the Extrude and Revolve commands show that reason instead of guessing. Extrude refuses a zero distance and a direction lying in the sketch plane, which used to produce zero-volume solids. Shapes drawn with the Rectangle and Polyline tools are now profiles — they could not be extruded at all |
| 104a | Done | Undoable model edits: adding, editing, reordering, deleting and suppressing features, and inserting components and mates, are undo-stack commands. Undo rebuilds what it changed, and the modified marker follows the stack. Editing a feature's values used to leave the document unmodified, so closing discarded the edit without a prompt. Suppression is saved; format version 17 |
| 104b | Done | Real 3D commands: primitives, Combine (Union / Subtract / Intersect of the part's bodies), Fillet, Chamfer, Shell, Draft and Linear / Circular Pattern ask for their inputs and add features. They used to drop fixed demo meshes into the viewport, outside the document. Edges and faces are chosen from lists until the viewport can pick them. Loft and Sweep wait for sketches on planes (104c) |
| 105 | Done | Topology gates: the Euler–Poincaré check rejected every torus and every part with a hole through it, so STEP import and Fillet refused them. It is now `V − E + F − R = 2(S − G)`, with the genus reported. Every feature result, and every Join / Cut / Intersect, is held to the manifold, Euler and geometric checks, and a malformed solid stops at its feature with a reason. Profiles that cross themselves are refused |
| 106 | Done (first cut) | Persistent naming. New extrusions name their side faces after their sketch entities and their edges after the faces they separate, so adding a vertex elsewhere no longer moves a fillet to a different edge. A face a Boolean splits gets a name per piece, and references resolve to the pieces. Older files keep positional names; format version 18 |
| 106b | Done | Boolean faces put back together: the CSG left every face of a result as triangles, so Fillet refused any edge of a part that had been through a Boolean. Each face is now merged back, and a face with a hole is cut through the hole into two. A plate with a hole can be filleted, undone, saved and reopened through the window, which was the Milestone 2 goal |
| 107 | Done | Import/export reachable: File ▸ Import (STEP as a new part, DXF into the drawing as one undoable step) and File ▸ Export (STEP, STL, glTF, DXF), where STEP, glTF and STL had no menu and STL had no writer at all. Opening or importing a file lists what it left out and why, instead of dropping it silently |
| 108a | Done | DXF geometry: polyline arcs (bulges), mirrored coordinate systems, the old POLYLINE / VERTEX form, partial ellipses, inserts with unequal or mirrored scales (averaged before: (2, 1) drew at 1.5), and blocks inside blocks are read as the file means them, with what had to be approximated reported. Also fixed: arcs mirrored in an axis away from their centre landed in the wrong place |
| 108b | Done | DXF text, colour and units: MTEXT read in order, one text per line, with its codes read, not shown; TEXT at its alignment point with its %% and \U+ codes; the full 256-colour index and true colour; Windows-1252/1251 files read as UTF-8; $INSUNITS scaled into millimetres. Saved files write true colour, escaped text and their units |
| 109 | Done | STEP fidelity: solids scaled into millimetres from the file's LENGTH_UNIT (SI prefixes and conversion-based units such as the inch); a solid that cannot be read is reported and the rest come in; faces with holes are drawn, measured and cut with their holes |
| 110 | Done | 2D correctness: snaps and picks reach 10 screen pixels at any zoom; hidden and locked layers are not snapped to or trimmed at; typed snaps (endpoint, midpoint, centre, quadrant) and a new intersection snap, with object snaps before the grid; Trim, Break, Extend, Chamfer and Fillet keep line type and group. Milestone 3 complete |
| 111 | Done | Application essentials: ten shortcuts that did nothing (bound to both menu and ribbon) now work; Open Recent; window and dock layout kept; Preferences (autosave, language, grid snap, snap reach, display units); Help ▸ About with version, revision and build; files open from the command line; every dock in View |
| 112 | Done | UI test harness: a smoke test runs every one of the window's 166 commands on an empty drawing, part and assembly and on a selection; the drawing tools are driven through the viewport's own mouse handling (draw, select, delete, undo) |
| 113 | Done | Render efficiency: the GL mesh cache drops meshes that left the scene (every edit used to leak the model's GPU buffers); the constraint analysis runs when the document changes, not every frame; the unread per-frame picking pass is gone; the renderer and text overlay work at device pixels, so text is sharp on high-DPI screens |
| 114 | Done | Off-thread work: a model rebuild that would freeze the window runs on a worker, from a snapshot, with progress and cancel, applied only if the part has not changed; large STEP imports and interference checks run in the background; atomic ID counters. Milestone 4 complete |
| 115 | Done | One version: set only in `project()`, generated into a header for the About box, `--version` and the log, and into the installer through CPack; the source revision is kept current on every build. The CPack settings file shadowed CMake's CPack module, so no installer configuration had ever been written. The CHANGELOG's "1.0.0" is 0.1.0 |
| 116 | Done | Packaging: an application icon (window, executable, installer, Linux desktop); Linux desktop entry, AppStream metadata and MIME types (validated); install rules for translations, licences, third-party notices and every bundled library's licence text, and on Windows the Qt runtime; an AppImage script (linuxdeploy); a test of the install tree |
| 117 | Done | Release pipeline: a tag `vX.Y.Z` builds, tests and packages Windows (NSIS) and Linux (AppImage, tarball) with SHA-256 checksums into a draft release; a Linux Release CI job with `-Werror`; the vcpkg binary cache works (it restored nothing, so Qt was rebuilt from source in every job); Dependabot for actions; [docs/RELEASING.md](docs/RELEASING.md) |
| 118 | Done | Governance: a [security policy](SECURITY.md) (private reporting; scripts and plugins are not sandboxed), a [code of conduct](CODE_OF_CONDUCT.md), issue forms, a pull request template with the CI gates, and a current [CONTRIBUTING](docs/CONTRIBUTING.md). The licence is the GNU GPL v3 or later: `LICENSE` holds its full text (it held a fragment), and the README, the About box and the AppStream metadata say so. Milestone 5 complete |
| 119 | Done | PDM integrity: a check-out is one exclusive file creation, so two users racing for a document cannot both get it (with the old lock file, both did); a damaged lock or archive fails closed, where an archive used to read as empty and its next commit overwrote the history; SHA-256 content hashes, verified on every read and push, with old FNV-1a archives still readable |
| 120 | Done | Scripting & plugin safety: a script's `doc` is removed when its run ends, and a copy the script kept raises instead of reaching a destroyed document (a use-after-free under ASan); scripting is off by default because it is not sandboxed; `PluginRegistry::prepareLoad` checks a plugin again when it is loaded, refuses one whose manifest changed since it was enabled, and returns the entry source it checked |
| 121 | Done | CAM & FEA honesty: G-code starts from a known modal state, loads its tool and starts the spindle before moving, climbs before every rapid crosses, stops the spindle at the end, and refuses programs or parameters that could rapid through the cut, cut with the spindle stopped or carry a non-finite number; FEA refuses solids that are not boxes, where it used to analyse their bounding box; the maturity table says which modules the application can reach. Milestone 6 complete |

### Product-completeness track

A fresh audit after Phase 121 found what still stands between a trustworthy
core and a product people can do real work in: a few crashes, hangs and
silently wrong answers; no way to print; no typed input; no sketching off
the XY plane or picking in 3D; and a view and a kernel that do not yet scale.
The [product-completeness roadmap](docs/superpowers/specs/2026-09-24-product-completeness-roadmap.md)
sets out six milestones from Phase 122 on, starting with the crashes.

| Phase | Status | Description |
|-------|--------|-------------|
| 122 | Done | 2D crash and hang fixes: starting Insert Block while it was active destroyed the active tool and then called it (a use-after-free; ASan confirms the new test catches it); removing an entity rebuilt the whole spatial index, so undoing a large DXF import took minutes (the R-tree now deletes in place, O(log n), and undo puts entities back in their drawing order); finding an entity by id was a scan of the drawing, done per selected entity in box selection, grips (every frame), commands and snapping (now an O(1) lookup); eight property commands left the index stale |
| 123 | Done | Kernel honesty: Shell rebuilt a new cup from two faces, dropping every hole and boss on the part and ignoring all but the first open face; it now hollows only a plain prism and refuses anything else, and offsets by the profile's winding (on an L-shaped part the cavity broke through the wall); Draft and Chamfer take inward and outward from loop winding, not a centroid; a failing feature leaves the part as it stood before it instead of an empty viewport, and is not rebuilt on every redraw; surface area on non-convex faces (a U-shaped cap counted 116 for 52); each three-edge fillet corner exported a whole sphere to STL and glTF |
| 124 | Done | Hostile files, round 2: DXF hatch boundaries are read path by path (reading every coordinate as one polygon merged a hatch's islands and seed points into a garbled outline), with arcs, ellipses and splines followed and islands reported; blocks nested in blocks that multiply (a million entities from a few kilobytes) are flattened only up to a budget, and the rest reported; DXF lineweights are ones readers accept, and layers carry theirs; a native file with two entities under one ID renumbers the second instead of hiding the first |
| 125 | Done | Background work that stops: Cancel now stops a STEP import and an interference check on their workers (they used to run to the end, and quitting waited for them); a job whose worker thread could not start is done on the calling thread instead of being waited for forever; an autosave that cannot be written, or cannot start, is shown in the status bar instead of only the log; a document whose recovery was followed by another crash is no longer recovered by default at every start; recovered documents are snapshotted before the crashed session's copies are deleted; an OpenGL context older than 3.3 no longer gets OpenGL 3 calls; typed fillet and chamfer sizes such as "1.2.3" are refused, not read as 1.2; the seven empty catch blocks are gone |
| 126 | Done | A safety net that catches: CI gains a ThreadSanitizer job (a worker now rebuilds while the window edits), a coverage job with a table by module in each run's summary, and a libFuzzer job that fuzzes every reader of untrusted input for a minute each (it only replayed seeds before), with new targets for the binary format, plugin manifests and the PDM's archive and locks; the Windows build reports its /W4 warnings by code; packages must carry the translations (compiled with any lrelease, and checked in every build), which the release build could not compile before; the vcpkg cache is kept when a later step fails; every job has a time limit, tests run in parallel, and the workflow only reads the repository |
| 127 | Done | Print and PDF, without the printer: the drawing exports to PDF and SVG on a chosen paper (A0–A4, Letter, Legal, Tabloid) and orientation, fitted or to scale (1:200 to 10:1), with line weights in millimetres, dash patterns and text to scale, white plotted black; a scale that does not fit is warned about first; block contents now all draw, on screen too (text, hatches, dimensions and nested blocks were left out); mirrored blocks are mirrored (they were turned half round) and keep it in files and DXF; printing itself waits on the Qt PrintSupport decision |
| 128 | Done | Precise input: points typed at the keyboard in the drawing tools (x,y; @dx,dy; @length<angle; or a length toward the cursor), refused whole when they are not a point; the line tool chains; object snap, grid snap, ortho and polar tracking switched from the status bar or F3/F9/F8/F10, and kept; a line's, circle's or arc's geometry typed into the property panel, one undo step each; selecting an entity no longer pushed edits of it onto the undo stack |
| 129 | Done | Dimensions and text: a dimension style editor, one undo step, with the unit dimensions show (mm, cm, m, in, ft) and whether they say it; continue and baseline dimensions; the linear dimension takes typed points; text of several lines, written, edited in the panel, plotted, and saved to DXF as MTEXT that reads back as one text |
| 130 | Done | Layers and blocks: layers can be renamed (carrying their entities, in blocks too, and the current layer) and their line weight set; Create Block takes a base point, puts the entities back in their drawing order on undo, and restores the same block reference on redo; the README's feature list now says only what the product does |
| 131 | Done | Sketches on planes: a sketch on XY, XZ, YZ, a flat face or a datum plane, drawn into with every 2D tool in its own coordinates; profiles with holes and separate regions, extruded and revolved (about the sketch's own axis); notes no longer break a profile; sketches keep their constraints in files; Shell's face list no longer named faces by their inside |
| 132 | Done | 3D picking: faces and edges of the part chosen by clicking (Shift to add), highlighted on hover; Fillet, Chamfer, Shell, sketch-on-face and Add Mate take what was clicked; the part draws its own edges, not its triangles'; picked on the CPU, so tested through the window |
| 133 | Done | The missing commands: Loft, Sweep and datum plane/axis/point commands (datums drawn, sketches on them); loft and sweep failures say why; feature definitions edited with angles in degrees, labelled fields, and directions and axes chosen (from the axes or a clicked face or edge); rollback from the feature tree, undoable and saved |
| 134 | Done | Extrude and pattern options: extrude both ways, through all (one way or both) and reversed; patterns of chosen features (a hole repeated, not the part); primitives placed at a point along an axis, editable and saved; features built knowing the part before them |
| 135 | Done | Finding and seeing: every modelling command in the Model menu and the Ctrl+K palette, with shortcuts; Fit All frames solids; orthographic that survives a resize; Back, Bottom and Left views; shaded, shaded-with-edges and wireframe display; a section plane; mass properties with a material |
| 136 | Done | A 2D view that scales: the drawing built once and kept until it changes, batched by pen (circles and arcs with the lines), only the chunks in view drawn, the text overlay painted only when it changes; the grid no longer hides the drawing, the overlay is the right way up, Stretch keeps what it stretches, and a tool changed mid-drag puts the drag back |
| 137 | Done | The GUI thread stays free: a feature added is built once, on a worker when builds are slow, and one that fails itself is withdrawn; a part opened is built on a worker; a large part or DXF is read on a worker; the model is tessellated once for each build |
| 138 | Done | Bounded memory: the DOF analysis sparse and by cluster, each with its own status, and off the paint path; an undo limit (Preferences); solves that keep only what they moved; one mesh for every instance of a part, one GPU buffer for each mesh; parts released when no component holds them |
| 139 | Done | Stable names: primitives named after their feature; fillet and chamfer keep the names of edges they do not touch; Revolve, Sweep and Loft faces named after their profile; a curve is one edge and a curved face one face (picked, listed and filleted whole); older files keep their names (format 19) |
| 140 | Done | Fillet and chamfer at any angle: oblique and concave edges between planar faces, exact; blend and chamfer ends on oblique end faces; parts of several bodies; oblique or concave corner blends refused by name |
| 141 | Done | Curved faces measured as curved: Mass Properties reports the part as modelled (its facets) and as designed (each face on the surface it approximates, refined and extrapolated: a cylinder, cone, sphere, torus, revolve and filleted box match their closed forms to 1e-9), and says which faces have no ideal; STEP cylinders, cones, spheres and tori come in as facets of their surfaces, where a cap bounded by one circle was skipped |
| 142 | Done | Boolean robustness: every random placement, a million millimetres out, at a thousandth and a hundred thousand times the size, and with faces in exact contact gives a valid, volume-conserving solid or refuses with a reason (a far box came back inside out, and a tiny part did not sew); tolerances relative to the operands; an iterative BSP that needs no deep stack; a face cut by any number of holes merged. Milestone 11 complete |
| 143 | Done | Placing components: an Assembly menu and tree (components and mates; remove, suppress, rename, edit a mate); Move and Rotate, undoable, with the mates solved again; inserts placed beside the others; a component clickable as soon as it is inserted, and Add Mate listing faces as a click picks them; removing a component takes its mates |
| 144 | Done | Living assemblies: a part saved in its tab or changed on disk shows changed in every assembly placing it, with the mates solved again; one closed unsaved leaves them as its file is; Open Part from the assembly; a bill of materials (one line per file, however spelled) exported as CSV. Milestone 12 complete |
| 145 | Done | A safety net that catches: CI's Qt had no platform plugin, so no window test had ever run in CI and the Linux build could not open a window; Qt built with what the product needs, the 181 window tests run in every job and are counted, the Linux Release job and the release run their tests, `horizon --self-test` starts the application on Xvfb before it ships, the viewport is drawn on OpenGL in CI, the 2D tools are tested by what they draw (Join and Tab fixed), and coverage floors hold (UI 1.3% → 68.6%) |
| 146 | Part 1 | Workbench controllers: the assembly commands (placing, mates, interference, bill of materials, following their parts) moved out of MainWindow into an AssemblyWorkbench that reaches the window only through a narrow WorkbenchHost, and runs without one in its tests; the part commands follow |
| 147 | Done | Clean projection: a drawing's edges are sharp, tangent or silhouette, seams inside a curved face are left out, a rim is drawn on its circle, visibility through a bounding-volume tree (a 2,048-facet cylinder in 0.7 s where it took 27 s), and the Front view looks from the front |
| 148 | Done | A drawing sheet: Drawing ▸ New Drawing from Part opens a tab of the part's four standard views in third-angle projection, at the largest ISO 5455 scale that fits, with border and title block, on locked layers; saved as `.hzdwg` (version 2: sheet, title block, views and a part path relative to the drawing); title block, paper and scale set in forms; drawn again when the part is saved, read again or changed on disk; plotted on its own paper at 1:1; views at a scale state their lengths at 1:1 |
| 149 | Done | Views and dimensions on a sheet: section views cut through a view (hatched 3 mm apart on paper, captioned SECTION A-A, the cut drawn on its view) and detail views clicked on one (circled there, at a standard scale); views placed where there is room, moved by clicks and removed with those taken from them; dimensions picked on edges, kept by the edges' stable names, measured again from the part (a circle's diameter, an arc's radius, an edge's length, spanning all of a partly hidden edge) and reported when their edge is gone; centre lines on holes and bosses (a cross end-on, the axis side-on, fillets left alone); View Properties for each view's hidden edges, tangent edges and centre lines; `.hzdwg` version 3 |
| 150 | Done | Drawings that follow the model: what is drawn on a sheet by hand saved with it and moved with its view (when the view is moved, laid out at another scale or paper, or moved by the part changing size); drawings of assemblies, their components drawn together and named apart, a balloon on each part and a parts list above the title block, drawn again when the assembly or any of its parts changes. Milestone 14 complete |
| 151 | Done | STEP export as designed: each curved face written once, on the surface its facets record (a cylinder's side one face, its rims circles), cut along a seam where it closes; rims a Boolean left without their circle recovered; what cannot yet be (spheres, apexes, holes a Boolean named by position cuts) written as facets and said at export. A moved or patterned part's facets now keep one surface |
| 152 | Done | STEP import of trimmed surfaces: a curved face with any outline, or with holes, cut into facets on its surface within its trim (triangulated in its surface's (u, v), holes bridged, made Delaunay, points as close as the surface turns), where it was one flat facet; a slant-cut cylinder and a pocketed one measure exactly |
| 153 | Done | STEP assemblies: a file's product structure read — each part once, placed as often and where its assemblies say, nested assemblies and their length units compounded, as OCC, SolidWorks and writers that put the part second write them — into a part (a body per placement, each named apart) or kept as an assembly of part files; an assembly tab exported as a STEP assembly, names in any script. Milestone 15 complete |
| 154 | Done | Document units: a unit per document, assembly and sheet (mm, cm, m, in, ft), saved in the file (a file without one reads as millimetres) and set by Edit ▸ Document Units as one undo step; new documents take the preference; the cursor readout, measuring, Mass Properties, mate distances and interference volumes in it; one table of units, with typed values ("2 in", "1' 6\"", "3/4 in", "30 deg") read by `math::parseLength`/`parseAngle`. Every length and angle field (feature forms, the Edit form, assembly and drawing commands, the property panel, array and block dialogs, constraint values) shows the unit and takes a value typed in any ("2 in", "0.5 rad"). Typed points, fillet and chamfer lengths and Rotate's angle take units too ("@2 in<30", "1 cm"), their prompts saying the unit |
| 155 | Part 1 | Variables: Edit ▸ Variables lists a document's named values ("wall" = "3 mm", "width" = "10 * wall"), each worked out as it is typed and what it measures checked (a length is not added to a number; units in expressions: mm, cm, m, in, ft, deg, rad); names, loops and errors refused with why; one undo step. Feature parameters as expressions of them follow |

The full multi-year design is in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md),
with per-phase implementation plans under
[docs/superpowers/plans/](docs/superpowers/plans/).

## Contributing

Contributions are welcome. Please open an issue to discuss a change before
submitting a pull request, and see [CONTRIBUTING](docs/CONTRIBUTING.md) for
how the code is laid out and what CI checks. Security problems go through the
[security policy](SECURITY.md), not public issues. Everyone taking part
follows the [code of conduct](CODE_OF_CONDUCT.md).

## License

Horizon CAD is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version. It is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE. The full text is in [LICENSE](LICENSE).

The libraries it is built on keep their own licences; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
