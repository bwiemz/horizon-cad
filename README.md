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

# Run tests (109 tests)
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
                 multi-document manager, assembly documents
  render/        OpenGL renderer, camera, grid, shaders, selection, scene graph
  constraint/    Geometric constraint solver (Newton-Raphson + LM damping)
  fileio/        Native formats (.hcad, .hzpart, .hzasm), DXF import/export
  geometry/      NURBS curves and surfaces with adaptive tessellation
  topology/      Half-edge B-Rep with TopologyID genealogy and Euler operators
  modeling/      Extrude, revolve, Booleans, fillet/chamfer, primitives
  ui/            Qt widgets, ribbon toolbar, tools, panels, document tabs
  app/           Application entry point, dark theme, resources
tests/           One suite per module + cross-module integration tests
```

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
| 63 | Done | Era 3 — Python API (phase 2): sheet-metal, fatigue (`horizon.SNCurve`), and FEA static + modal + steady-state thermal analysis on the current solid (`doc.static_analysis`, `doc.modal_analysis`, `doc.thermal_analysis`), bound into the embedded `horizon` module |
| 64 | Done | Era 3 — Stabilization: end-to-end integration test composing annotated/framed drawing → DXF round-trip, model → FEA, and PDM commit → semantic diff on one model |
| 71 | Done | Era 4 — CAM (core): 2.5-axis contour + drilling toolpaths with cutting/rapid length accounting, RS-274 G-code output (G0/G1, modal feed), feeds & speeds (spindle RPM, chip-load feed) |
| 72 | Done | Era 4 — Kinematics: serial-chain forward kinematics (revolute/prismatic joints) + CCD inverse kinematics; validated vs analytical arm poses |
| 73 | Done | Era 4 — Advanced simulation: (a) modal analysis — consistent-mass linear-tet element + generalized eigensolve `K φ = ω² M φ` for natural frequencies and mode shapes, validated by exact scaling invariances (f ∝ √E, 1/√ρ, 1/size), free-free rigid-body modes, and ordering; (b) stress-life fatigue — Basquin S-N curve, cycles-to-failure, Goodman/Soderberg mean-stress correction, and endurance safety factor. Exposed as `doc.modal_analysis`, `horizon.SNCurve`, `horizon.fatigue_safety_factor` |
| 74 | Done | Era 4 — Configuration management: named design-table configurations of parameter overrides driving part-family variants, applied onto the parameter registry |
| 50 | Done | Era 2 — STEP AP242 import/export: in-house ISO 10303-21 writer/reader, lossless (rational) B-spline mapping of the NURBS B-Rep, MANIFOLD_SOLID_BREP reconstruction with manifold validation, PLANE/LINE/CIRCLE/CYLINDRICAL_SURFACE interop for external files, round-trip idempotence guard |
| 51 | Done | Era 2 — Native binary format: FlatBuffers `.hzpart`/`.hzasm` container (file id `HZBF`) wrapping the canonical JSON envelope + typed zero-copy tessellation cache; lightweight mesh loads skip the JSON parse entirely; verifier-gated against corrupt/truncated files; JSON/binary sniffing shares the extensions |
| 61 | Done | Era 3 — Advanced fillets & drawings: variable-radius fillets (radius-stop table → exact ruled conic patches), 3-edge spherical corner vertex blends; fillet reconstruction rewritten to strict half-edge assembly (proper chord-cut ends, perpendicular-convex gate refuses oblique/reentrant edges instead of emitting wrong geometry, Newell-normal side selection); drawing section views (tessellation cut → closed profiles + 45° hatching + retained outline, DXF `Section`/`Hatch` layers); model-driven radial/diameter dimensions (circle-fit measurement, R/⌀ leaders in DXF); cylinder rims now carry true arc curves |
| 61b | Planned | Era 3 — sheet-metal 3D features (flange bodies) |
| 65 | Done | Era 4 — Rendering abstraction layer: `RenderBackend` interface (buffers/textures/shaders/passes/draws/compute per roadmap §7.1), `OpenGLBackend` over the existing Qt GL path (incl. GL 4.3 compute dispatch), `VulkanBackend` staged bring-up behind quiet SDK detection (instance + discrete-GPU device + queue + host-visible buffer allocation, verified on RTX 5070 Ti; textures/draws staged pending SPIR-V pipeline); RAL contract tests + opt-in GL runtime tests |
| 66 | Done | Era 4 — GPU compute (core): Vulkan compute pipeline (SPIR-V one-shot dispatch with storage-buffer descriptor sets, fence-synchronized, host readback), NURBS surface evaluation kernel (Cox–de Boor in GLSL, compiled at build time via glslangValidator, matching `NurbsSurface::evaluate`'s two-pass rational semantics exactly), `GpuTessellator` grid evaluator — verified against the CPU reference on plane/cylinder/sphere/torus grids on an RTX 5070 Ti; compute shaders only per roadmap §7.2 (MoltenVK-portable) |
| 65-80 | Planned | Era 4 — Vulkan/Metal backend, PBR/ray tracing, cloud sync, live collaboration, 1.0 release |

The full multi-year design is in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md),
with per-phase implementation plans under
[docs/superpowers/plans/](docs/superpowers/plans/).

## Contributing

Contributions are welcome. Please open an issue to discuss changes before submitting a pull request.

## License

MIT
