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
- Native JSON file format (`.hcad` v12) with backward-compatible versioning
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

**Prerequisites:** A C++20 compiler (MSVC, GCC, or Clang), CMake 3.28+, vcpkg

```bash
# Configure
cmake --preset debug

# Build
cmake --build build/debug --config Debug

# Run tests (~900 tests)
ctest --test-dir build/debug -C Debug

# Run
./build/debug/src/app/Debug/horizon.exe
```

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
tests/           One suite per module + cross-module integration tests (~900 tests)
```

## Feature Maturity

"Phase done" in the roadmap below means an **honest core slice** exists —
implemented, tested, and documented — not that the area is
production-hardened. This table is the truthful per-module picture. Ratings:

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
| document / undo | stable | Feature tree + multi-document are newer but well-tested |
| constraint | stable | Newton-Raphson + LM solver, exercised by sketch tests |
| geometry (NURBS) | stable | Curves/surfaces/tessellation; Coons patches experimental |
| topology (B-Rep) | stable | Caveat: validators are combinatorial only — they check twin/Euler structure, not that twin geometry coincides or faces avoid self-intersection |
| modeling — Booleans | experimental | BSP-CSG with face splitting, exact fragment classification, coplanar handling, manifold sewing; volumes closed-form-tested. Curved faces participate as their inscribed loop polyhedra (analytic SSI is future work) |
| modeling — extrude/revolve/primitives/patterns | stable | Exact volumes verified |
| modeling — fillet/chamfer | prototype | Known defect (pinned in `test_AdversarialModels`): output is combinatorially valid but volume integration is inconsistent; rebuild on `SolidSewer` planned |
| modeling — loft/sweep/shell/draft | experimental | Core paths tested; complex inputs unverified |
| modeling — sheet metal | experimental | Validated against analytic bend formulas |
| fileio — native (.hcad/.hzpart/.hzasm) | stable | JSON + FlatBuffers binary, backward compatible v1-v9 |
| fileio — DXF | stable | Entity subset documented |
| fileio — STEP AP242 | experimental | Core B-Rep subset with documented limitations (untrimmed analytic carriers, no BREP_WITH_VOIDS, no assembly structure — all pinned by fixture tests in `tests/fileio/fixtures/step/`) |
| fileio — glTF/STL/drawings | experimental | Export-only slices |
| render — OpenGL path | stable | The shipping viewport |
| render — Vulkan / GPU tessellation / path tracer | experimental | Staged bring-up, opt-in |
| simulation (FEA) | prototype | Educational/basic analysis: structured box meshing, linear-static/thermal/modal on tets, validated against analytic bars — not a general-purpose FEA workbench |
| pdm / sync / collaboration | experimental | Deliberately conservative: append-only, hash-verified, pessimistic locks, no merges |
| kinematics | prototype | Serial chains only |
| cam | prototype | Contour/drill/rect-pocket slices; no offsetting engine, gouge checking, or post-processor architecture yet |
| plugin registry | experimental | Fail-closed validation without code execution; the execution bridge is future work |
| scripting (Python) | experimental | Optional embedded CPython |
| ui / app | stable | Qt ribbon shell, i18n catalogs |

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
| 80 | Done | Era 4 — 1.0 release prep: full roadmap swept to Done, [CHANGELOG.md](CHANGELOG.md) authored era-by-era, ~900 automated regression tests green across Windows/Ubuntu/AddressSanitizer CI, honest per-phase scope + documented deviations (OpenCAMLib, Embree, STEPcode) in the [findings note](docs/superpowers/notes/2026-07-03-era2-roadmap-findings.md); installers/marketplace/published-benchmarks are post-1.0 productization, not code slices |

All 80 roadmap phases plus the 61b sheet-metal insert have delivered their
core slices — see [Feature Maturity](#feature-maturity) above for what that
does and does not mean per module; Horizon is not a production-ready 1.0 CAD
system. Deferred-by-design items (Phase 73 CFD, and the productization tail
of Phase 80 — signed installers, the hosted plugin marketplace,
SolidWorks/FreeCAD benchmark publication) are called out in the per-phase
notes and remain future work beyond the code kernel.

The full multi-year design is in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md),
with per-phase implementation plans under
[docs/superpowers/plans/](docs/superpowers/plans/).

## Contributing

Contributions are welcome. Please open an issue to discuss changes before submitting a pull request.

## License

MIT
