# Horizon CAD

An open-source 2D drafting application built from scratch in C++20. Horizon provides a familiar CAD workflow with drawing tools, dimension annotations, constraints, layers, blocks, undo/redo, and file I/O — all rendered with OpenGL and wrapped in a modern Qt6 desktop interface.

## Features

### Drawing Tools
- **Line**, **Circle** (center-radius), **Arc** (3-click), **Rectangle** (2-corner), **Polyline** (multi-click, open or closed)
- **Ellipse** (center + semi-axes), **Spline** (cubic B-spline with control points)
- **Text** (standalone text entities with height, rotation, alignment)
- **Hatch** (boundary-fill with ANSI line, cross, and custom patterns)
- Snap-to-geometry engine with endpoint, midpoint, center, and intersection snapping

### Editing Tools
- Select with **window/crossing box selection** (left-to-right = enclosed only, right-to-left = overlapping), click, and Shift multi-select
- **Entity grouping** (Ctrl+G / Ctrl+Shift+G) — lightweight selection groups without block overhead
- Move, Duplicate, Offset, Trim, Fillet, Chamfer, Break, Extend, Stretch, Mirror, Rotate, Scale
- Copy/Paste with clipboard support (Ctrl+C/X/V)
- Rectangular and Polar array operations
- Polyline editing (add/remove vertices, toggle closed, join polylines)
- Grip editing for direct point manipulation

### Dimensions & Annotations
- **Linear** dimensions (horizontal, vertical, aligned) with auto-orientation detection
- **Radial** dimensions (radius or diameter) placed on circles and arcs
- **Angular** dimensions measuring the angle between two lines
- **Leader** annotations with custom text
- Text override support on all dimension types
- Configurable dimension style (text height, arrow size, precision)

### Measurement Tools
- **Distance** measurement between two points
- **Angle** measurement between two lines
- **Area** measurement of closed polygons

### Geometric Constraints
- Coincident, Horizontal, Vertical, Perpendicular, Parallel, Tangent
- Equal length, Fixed position, Distance, and Angle constraints
- Constraint solver with real-time visual indicators

### Blocks & Components
- Create reusable block definitions from selected entities
- Insert block references with position, rotation, and scale
- Explode block references back to individual entities

### Layers & Properties
- Layer management with visibility, lock, color, line width, and line type
- **Line types**: Continuous, Dashed, Dotted, DashDot, Center, Hidden, Phantom — rendered via GPU shader
- ByLayer property inheritance — entities can inherit color, line width, and line type from their layer
- Property panel for inspecting and editing selected entities
- Layer panel with add, remove, rename, and per-layer controls

### Document System
- Full undo/redo with composite command support
- Native JSON file format (`.hcad`/`.hzpart`, format v16) with backward-compatible versioning
- DXF import/export (LINE, CIRCLE, ARC, LWPOLYLINE, TEXT, MTEXT, SPLINE, HATCH, INSERT)
- New, Open, Save, Save As workflow

### Modern UI
- **Dark theme** with Fusion style, custom palette, and QSS stylesheet
- **Ribbon toolbar** with tabbed categories (Home, Draw, Modify, Annotate, Constrain, Measure, Blocks, View)
- **Programmatic icons** — 45+ vector-style icons generated via QPainter (no external assets)
- **Keyboard shortcuts** for all tools (single-key access: L=Line, C=Circle, etc.)
- **Enhanced status bar** showing coordinates, active tool, snap mode, selection count, and tool prompts
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
| `HZ_ENABLE_SCRIPTING` | `ON` | Embedded Python, when Python 3 and pybind11 are found |

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
| geometry (NURBS) | stable | Curves/surfaces/tessellation, rational surfaces exact between knots; Coons patches experimental |
| topology (B-Rep) | stable | `Solid::isValid()` is combinatorial (twin/Euler structure); `GeometryValidator` adds the geometric checks — vertex-chain and twin coincidence, degenerate edges/faces, loop planarity against the face's own carrier, self-intersecting loops, shell closure |
| modeling — Booleans | experimental | BSP-CSG with face splitting, exact fragment classification, coplanar handling, manifold sewing; volumes closed-form-tested. Operates on the faceted boundary, which since Phase 84 is the whole solid rather than a 4-sided caricature of it — a bored cylinder now comes out within the facet error instead of 36% light. Analytic surface–surface intersection is still future work |
| modeling — extrude/revolve/primitives/patterns | stable | Exact volumes verified. Profile arcs and circles are faceted (`segments` / `chordTolerance`) with their ideal cylinder and circles recorded, so an extruded circle is a true inscribed prism rather than a square. Curved primitives (cylinder/cone/sphere/torus) and revolves are faceted at construction — `segments` is tunable, `segmentsForTolerance()` derives it from a chord-sag budget — so their volumes converge to the analytic value rather than matching it exactly; each facet records the analytic surface it approximates. Revolve takes any closed profile through any angle in (0, 2π], and refuses a profile that crosses the axis or a plane the axis does not lie in |
| modeling — fillet/chamfer | experimental | Chamfer rebuilt on `SolidSewer` (82) with vertex blends (83), and both ops now work on faceted geometry (85–86): a cylinder rim chamfers to the exact truncated cone up to the cap's inradius, and fillet blends are faceted across the arc so their volume converges to `r²(1−π/4)L` instead of integrating as a chamfer. Capacity is the adjacent face's width, not a proxy. Both refuse geometrically invalid output. Straight edges of planar-faced solids with orthogonal, convex corners; oblique corners are refused, not mis-built. Fillet chains (two edges per vertex, e.g. a cylinder rim) are mitered |
| modeling — loft/sweep/shell/draft | experimental | Core paths tested; complex inputs unverified. Twisted loft bands are faceted on alternating diagonals, so a loft's volume is the ruled solid's exactly. Sweep carries the profile by a rotation-minimizing (mitered) frame, so its volume is exactly section area × path length; path arcs are sampled at a `segments` resolution. A path that crosses itself far from any single turn is caught only by the geometric gate |
| modeling — sheet metal | experimental | Validated against analytic bend formulas |
| fileio — native (.hcad/.hzpart/.hzasm) | stable | JSON + FlatBuffers binary, backward compatible v1-v9. Feature faceting resolution round-trips; files written before it existed load at the feature defaults |
| fileio — DXF | stable | Entity subset documented |
| fileio — STEP AP242 | experimental, library-only | Core B-Rep subset with documented limitations (untrimmed analytic carriers, no BREP_WITH_VOIDS, no assembly structure — all pinned by fixture tests in `tests/fileio/fixtures/step/`). Not yet reachable from the application (Phase 107) |
| fileio — glTF/STL/drawings | experimental, library-only | Export-only slices, not yet reachable from the application (Phase 107) |
| render — OpenGL path | stable | The shipping viewport |
| render — Vulkan / GPU tessellation / path tracer | experimental | Staged bring-up, opt-in |
| simulation (FEA) | prototype, library-only | Educational/basic analysis: structured box meshing, linear-static/thermal/modal on tets, validated against analytic bars — not a general-purpose FEA workbench. Meshes a solid's bounding box, so only boxes are analysed correctly |
| pdm / sync / collaboration | experimental, library-only | Deliberately conservative: append-only, pessimistic locks, no merges. A check-out is one exclusive file creation, so two users cannot both win it; content is verified against its SHA-256 on every read and push; an archive or lock that cannot be read fails closed instead of reading as empty or free |
| kinematics | prototype, library-only | Serial chains only |
| cam | prototype, library-only | Contour/drill/rect-pocket slices; no offsetting engine, gouge checking, or post-processor architecture yet. Not safe to run on a machine (Phase 121) |
| plugin registry | experimental, library-only | Fail-closed validation without code execution; the execution bridge is future work |
| scripting (Python) | experimental, library-only | Optional embedded CPython; not sandboxed |
| ui / app | experimental | Qt ribbon shell, i18n catalogs, 2D drafting tools with full undo. The 3D ribbon's primitive, Boolean, fillet and chamfer commands are still fixed demos outside the document (Phase 104); Extrude and Revolve are real features that join, cut, intersect or start a body. The first offscreen UI tests exist; the broad harness is Phase 112 |

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

The kernel is broad; the product around it is not yet safe to trust with real
work. The [production-readiness roadmap](docs/superpowers/specs/2026-09-23-production-readiness-roadmap.md)
sets out six milestones from Phase 97 on, starting with data safety.

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
