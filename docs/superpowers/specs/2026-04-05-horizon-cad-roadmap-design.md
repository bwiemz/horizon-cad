# Horizon CAD — Multi-Year Roadmap & Technical Specification

**Date:** 2026-04-05
**Author:** Brandon Wiemer + Claude (Principal Architect role)
**Status:** Design approved, pending implementation planning
**Scope:** Complete evolution from 2D CAD to professional-grade 3D parametric modeler

---

## 1. Strategic Direction

### 1.1 Vision
Horizon CAD evolves from the best open-source 2D drafting tool into a professional-grade 3D parametric modeler capable of competing with SolidWorks. The approach is hybrid: perfect the 2D foundation first, then progressively add 3D capabilities through a custom geometry kernel.

### 1.2 Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Competitive target | SolidWorks (long-term) | Largest MCAD market segment; mechanical engineering focus |
| Geometry kernel | Custom-built (no OCCT) | Full API control, no legacy baggage, modern C++20 design |
| Development model | Solo-first, community-ready | Small shippable phases; architecture supports contributors |
| Platform strategy | Windows + Linux now, macOS via Vulkan/MoltenVK later | Linux is free with Qt6/OpenGL; macOS needs rendering migration |
| 3D feature focus | Mechanical engineering (parts → assemblies → drawings) | Exercises the kernel hardest; largest market |
| File formats | Evolve native `.hcad` → binary `.hzpart`/`.hzasm`; STEP AP242 when kernel matures | No premature STEP parser before kernel can consume B-Rep |
| Development approach | Feature-interleaved (vertical slices) | Every phase ships visible value; ~10-15% rework accepted |

### 1.3 Current State (Phase 24 Complete)

Horizon CAD is a well-architected 2D CAD application with:
- **45+ tools** across drawing, editing, dimensioning, measuring, blocks, and constraints
- **10-type constraint solver** using Newton-Raphson with Levenberg-Marquardt damping (Eigen-based)
- **Command-pattern undo/redo** with CompositeCommand support
- **OpenGL 3.3 Core** rendering with GPU line-type patterns
- **DXF import/export** + native `.hcad` JSON format (v12, backward-compatible to v1)
- **Blocks, layers, dimensions, hatches, splines** — full 2D drafting feature set
- **114 unit tests** (90 math + 24 constraint)
- **Clean module separation:** math → geometry → drafting → constraint → document → render → fileio → ui → app

**Architecture gaps identified:**
- No spatial indexing (O(n²) snap/hit-test/intersection)
- Constraint solver not wired into undo/redo workflow
- No expression engine for parametric dimensions
- All 2D entities assume absolute world XY (no sketch plane abstraction)
- Empty `topology/` and `modeling/` modules (placeholder for 3D)
- No scripting API
- No 3D geometry, no B-Rep, no NURBS surfaces

---

## 2. Roadmap Overview

The roadmap is organized into four eras, each building on the previous:

```
Era 0: Foundation Hardening          (Phases 25-30)   — Fix 2D bottlenecks, prepare for 3D
Era 1: The Geometry Kernel           (Phases 31-40)   — Custom B-Rep kernel + first 3D features
Era 2: Assembly & Advanced Features  (Phases 41-52)   — Multi-part assemblies, scripting, STEP
Era 3: Drawings, Simulation & PDM   (Phases 53-64)   — Professional workflow completion
Era 4: Cloud, Rendering & Parity    (Phases 65-80)   — Market-competitive product
```

### Phase Map

| Phase | Name | Era | Key Deliverable |
|-------|------|-----|-----------------|
| 25 | Spatial Indexing | 0 | R*-tree for O(log n) spatial queries |
| 26 | Parametric Sketch Integration | 0 | Constraint solver wired to undo/redo |
| 27 | Expression Engine | 0 | AST-based expression evaluator + design variables |
| 28 | Sketch Planes | 0 | Local coordinate systems for 3D-ready sketching |
| 29 | Linux CI/CD | 0 | Cross-platform builds + AppImage |
| 30 | Stabilization | 0 | Refactor, test coverage, `.hcad` v13 |
| 31 | NURBS Curves | 1 | Full NURBS curve library with exact conics |
| 32 | NURBS Surfaces | 1 | Tensor-product NURBS surfaces + tessellation |
| 33 | Half-Edge B-Rep | 1 | Topology data structure + TopologyID genealogy |
| 34 | Primitive Solids | 1 | Box, cylinder, sphere, cone, torus (first 3D visuals) |
| 35 | Extrude & Revolve | 1 | First parametric 3D features + feature tree |
| 36 | Boolean Operations | 1 | Union, subtract, intersect with exact predicates |
| 37 | Fillet & Chamfer | 1 | Edge fillets (single chains only, no vertex blends) |
| 38 | Feature Tree UI | 1 | Parametric editing, rollback, rebuild with TNP resolution |
| 39 | 3D Viewport Polish | 1 | Navigation, selection, PBR-lite rendering |
| 40 | Kernel Hardening | 1 | Fuzz testing, regression suite, `.hcad` v14 |
| 41 | Multi-Document Architecture | 2 | `.hzpart` / `.hzasm` document types |
| 42 | Assembly Mates | 2 | 3D constraint solver with kinematic graph pre-analysis |
| 43 | Loft & Sweep | 2 | Complex 3D shapes from profile interpolation |
| 44 | Shell & Draft | 2 | Thin-wall features with self-intersection detection |
| 45 | Patterns | 2 | Dual-mode: geometry pattern (default) + feature pattern |
| 46 | Reference Geometry | 2 | Datum planes, axes, points |
| 47 | Python API (Phase 1) | 2 | pybind11 scripting console + macro recording |
| 48 | Collision Detection | 2 | Assembly interference checking |
| 49 | Measurement & Mass Properties | 2 | Volume, CG, inertia, material assignment |
| 50 | STEP AP242 Import/Export | 2 | STEPcode integration for industry interchange |
| 51 | Native Binary Format | 2 | FlatBuffers `.hzpart`/`.hzasm` with tessellation cache |
| 52 | Era 2 Stabilization | 2 | Assembly solver stress test, Boolean robustness sweep |
| 53 | 2D Drawings from 3D | 3 | HLR, section views, model-driven dimensioning |
| 54 | GD&T | 3 | ASME Y14.5 / ISO 1101 feature control frames |
| 55 | BOM & Balloons | 3 | Bill of materials + assembly annotations |
| 56 | Print/Plot Pipeline | 3 | PDF/printer output at exact scale with line weights |
| 57 | FEA — Linear Static | 3 | Defeaturing + TetGen meshing + CalculiX solver |
| 58 | Thermal Analysis | 3 | Steady-state thermal + thermal-structural coupling |
| 59 | PDM — Local Versioning | 3 | Feature-tree semantic diffs, revision browser |
| 60 | PDM — Multi-User Vault | 3 | Check-out/check-in with pessimistic locking |
| 61 | Advanced Fillets | 3 | Vertex blends (Gregory patches), variable-radius |
| 62 | Sheet Metal | 3 | Base flange, edge flange, flat pattern, DXF export |
| 63 | Python API (Phase 2) | 3 | Full modeling API, batch CLI, plugin system |
| 64 | Era 3 Stabilization | 3 | Drawing regen tests, FEA validation, STEP round-trip |
| 65 | Vulkan/Metal Backend | 4 | Rendering abstraction layer, macOS support |
| 66 | GPU Compute Acceleration | 4 | Compute-shader tessellation + spatial queries |
| 67 | PBR Rendering | 4 | Metallic-roughness materials, IBL, material library |
| 68 | Ray-Traced Rendering | 4 | Embree-based progressive path tracer |
| 69 | Cloud Sync | 4 | Local-first with optional self-hosted cloud vault |
| 70 | Live Collaboration | 4 | Presence + feature-level token locking |
| 71 | CAM Integration | 4 | OpenCAMLib-based 2.5D milling + G-code export |
| 72 | Kinematics & Motion | 4 | Mechanism simulation + animation |
| 73 | Advanced Simulation | 4 | Nonlinear, dynamic, fatigue, optional CFD |
| 74 | Design Tables | 4 | Configuration management + part families |
| 75 | Surfacing Workbench | 4 | Freeform NURBS surface modeling |
| 76 | Import/Export Ecosystem | 4 | IGES, Parasolid, STL/3MF, glTF, PDF 3D, DWG |
| 77 | Accessibility & i18n | 4 | Localization, screen reader support, documentation |
| 78 | Large Assembly Optimization | 4 | Lightweight mode, frustum/occlusion culling, instancing |
| 79 | Plugin Marketplace | 4 | Community ecosystem with optional paid plugins |
| 80 | 1.0 Release | 4 | Full regression, benchmarks, installer polish |

---

## 3. Era 0: Foundation Hardening (Phases 25-30)

### 3.1 Phase 25: Spatial Indexing & Performance Infrastructure

**Goal:** Replace O(n²) spatial queries with O(log n) R*-tree queries across snap, hit-test, and selection operations.

**Module:** `hz::math` (generic R*-tree), `hz::draft` (SpatialIndex in DraftDocument)

**Design:**
- Template `RStarTree<BBoxType, ValueType>` in `hz::math` supporting both 2D and 3D bounding boxes
- Implementation notes: cache-friendly contiguous node layout (flat array of nodes, not heap-allocated per-node) for CPU traversal performance
- `SpatialIndex` class in `DraftDocument` wrapping the R*-tree, synchronized via observer pattern on entity add/remove/move signals
- Refactor `SnapEngine` to query R*-tree instead of iterating all entities
- Refactor `SelectTool::boxSelect()` and all hit-testing to use R*-tree range queries
- Refactor intersection detection to use R*-tree candidate pair filtering

**Tests:** R*-tree unit tests (insert, remove, range query, nearest-neighbor). Benchmark: 10,000 entities snap query < 1ms.

### 3.2 Phase 26: Parametric Sketch Integration

**Goal:** Wire the constraint solver into the editing workflow so constraints are functional, not decorative.

**Module:** `hz::doc`, `hz::cstr`, `hz::ui`

**Design:**
- After any entity modification command, trigger constraint re-solve for affected entities
- `ApplyConstraintSolveCommand`: snapshots all affected entity positions before solve, applies solved positions, undoable as a single step via `CompositeCommand`. When a user changes a dimension from 10 to 20 and the solver moves 50 entities, Ctrl+Z undoes the entire solve in one step.
- DOF visualization: green = under-constrained (free DOF remaining), black = fully constrained, red = over-constrained. Matches SolidWorks convention.
- Constraint dimension editing: double-click a distance/angle constraint to change its value → triggers re-solve
- `ParameterRegistry` in `Document`: maps string names to double values (design variables). Constraints can reference variables instead of literals.

**Tests:** Constrain a triangle (3 lines + coincident + horizontal + distance). Edit distance → verify triangle reshapes. Undo → verify original shape. Over-constrain → verify red highlight.

### 3.3 Phase 27: Expression Engine & Driven Dimensions

**Goal:** Enable parametric relationships via algebraic expressions in dimensions and constraints.

**Module:** `hz::math` (expression evaluator)

**Design:**
- Recursive descent parser producing a serializable **AST** (Abstract Syntax Tree)
- Supported operations: `+`, `-`, `*`, `/`, `^`, `sin`, `cos`, `tan`, `sqrt`, `abs`, `pi`, `atan2`, parentheses, variable references
- AST node types: `Literal(double)`, `Variable(string)`, `BinaryOp(op, left, right)`, `UnaryOp(op, child)`, `FunctionCall(name, args)`
- AST serializes naturally to `.hcad` JSON for persistence
- `DimensionStyle` gains optional `expression` field alongside `value`. When expression is non-empty, value is computed from the expression AST.
- Design variables (from Phase 26) are accessible in expressions: `width * 2 + offset`
- Circular dependency detection: topological sort of the expression dependency graph. Error on cycles with clear message ("Circular dependency: width → height → width")
- Propagation chain: variable change → re-evaluate all dependent expressions → re-solve all dependent constraints → re-render

**Tests:** Parse and evaluate known expressions. Variable substitution. Circular dependency detection. Serialization round-trip (AST → JSON → AST → evaluate → same result).

### 3.4 Phase 28: Sketch Planes & Local Coordinate Systems

**Goal:** Decouple 2D sketches from absolute world XY so they can live on arbitrary 3D planes — the prerequisite for all 3D modeling.

**Module:** `hz::draft` (SketchPlane, Sketch), `hz::ui` (ViewportWidget)

**Design:**
- `SketchPlane`: defined by origin (`Vec3`), normal (`Vec3`), and X-axis direction (`Vec3`). Y-axis derived via cross product. Forms a complete local coordinate frame (origin + orthonormal basis).
- `Sketch`: container owning a `SketchPlane` + collection of `DraftEntity` objects + `ConstraintSystem`. Replaces the flat "all entities in one bag" model for constrained sketching.
- All `DraftEntity` objects inside a `Sketch` store coordinates in **local 2D** (relative to the sketch plane)
- Coordinate transforms: `Sketch::localToWorld(Vec2) → Vec3` and `Sketch::worldToLocal(Vec3) → Vec2`, implemented as `Mat4` transforms
- **Rendering:** When editing a sketch, camera aligns to the sketch plane normal ("Look At" mode). 2D tools work in local coordinates exactly as today. When orbiting away, sketch renders as a flat drawing floating in 3D space.
- **Backward compatibility:** Existing "default sketch" is a `Sketch` on the world XY plane at origin. Existing `.hcad` files load into this default sketch. No behavior changes for 2D-only users.
- `ViewportWidget::setActiveSketch(Sketch*)`: when active, mouse coordinates project onto the sketch plane before passing to tools. Tools receive `Vec2` local coords — no tool changes required.

**Tests:** Create sketch on XY plane → verify local coords match world coords. Create sketch on angled plane → verify localToWorld/worldToLocal round-trip. Render sketch on angled plane → verify visual correctness.

### 3.5 Phase 29: Linux CI/CD & Cross-Platform Hardening

**Goal:** Establish cross-platform discipline before the codebase grows with 3D code.

**Module:** CI/CD infrastructure

**Design:**
- GitHub Actions CI matrix: Ubuntu 22.04 (GCC 12) + Windows (MSVC 2022)
- CMake presets: `linux-debug`, `linux-release` alongside existing `debug`/`release`
- Fix platform-specific assumptions: file paths (forward slashes), OpenGL context creation, font rendering differences
- Package as AppImage for Linux distribution
- `clang-tidy` static analysis + `clang-format` enforcement in CI (fail build on violations)
- All existing tests pass on both platforms

### 3.6 Phase 30: Stabilization & Refactor Pass

**Goal:** Clean up before the 3D push.

**Design:**
- Refactor `ViewportWidget`: extract `ViewportRenderer` (rendering logic) and `ViewportInputHandler` (mouse/keyboard event routing)
- Unit test coverage for R*-tree, expression engine, sketch plane transforms — target >80% branch coverage per new module
- Performance benchmark: 10,000-entity stress test. Verify no regressions from R*-tree integration.
- Update `.hcad` to v13: add expressions, design variables, sketch plane definitions
- Write contributor documentation: architecture overview, how to add an entity type, how to add a tool, coding standards

---

## 4. Era 1: The Geometry Kernel (Phases 31-40)

### 4.1 Phase 31: NURBS Curve Library

**Goal:** Full NURBS curve implementation — the universal curve representation for CAD.

**Module:** `hz::geometry`

**Design:**
- `NurbsCurve`: control points (`std::vector<Vec3>`), weights (`std::vector<double>`), knot vector (`std::vector<double>`), degree (`int`)
- **Core algorithms:**
  - **De Boor's algorithm** for point evaluation at parameter `t`. Must handle knot multiplicities (repeated knots at endpoints for clamped curves) without division by zero — guard with `Tolerance::kZero`.
  - **Boehm's knot insertion** for refinement without shape change
  - **Degree elevation**
  - **Derivative evaluation** (1st and 2nd order — tangent and curvature)
  - **Arc-length parameterization** via Newton iteration on integrated derivative
  - **Closest-point projection** (`point → parameter t`) via Newton iteration on the distance-squared function derivative
- **Exact conic representation:** Factory functions producing NURBS curves that exactly represent circles, arcs, and ellipses using rational weights. `DraftCircle` and `DraftArc` can be internally represented as NURBS with zero approximation loss.
- **Adaptive tessellation:** `NurbsCurve::tessellate(double tolerance) → std::vector<Vec3>` — curvature-based subdivision (tight curves get more points, straight sections get fewer)
- **User-facing:** Upgrade spline tool to support NURBS with weight editing. Control-point grip editing with weight handles.

**Tests:** Evaluate known curves (line as degree-1 NURBS, circle as 9-point rational NURBS). Verify derivatives match finite-difference approximation. Verify closest-point converges. Verify conic accuracy < `Tolerance::kLinear`.

### 4.2 Phase 32: NURBS Surface Library

**Goal:** Tensor-product NURBS surfaces — the face geometry for B-Rep solids.

**Module:** `hz::geometry`

**Design:**
- `NurbsSurface`: control point grid (`Vec3[][]`), weight grid, knot vectors in U and V, degrees in U and V
- **Core algorithms:**
  - Point evaluation at `(u, v)` via De Boor in U then V
  - Partial derivatives `dS/du`, `dS/dv`
  - Surface normal: `N = (dS/du × dS/dv).normalized()`
  - Closest-point projection `(point → u, v)` via Newton iteration on the 2D gradient of distance-squared
  - Iso-parametric curve extraction: fix U or V, return a `NurbsCurve`
- **Adaptive tessellation:** Surface → triangle mesh based on curvature. Flat regions get coarse triangles; curved regions get fine. Output is `MeshData` (vertex + index buffer) compatible with `SceneNode::setMesh()`.
- **Factory surfaces:** Planar, cylindrical, spherical, conical, toroidal — all representable as NURBS. These are the surfaces appearing on mechanical parts.

**Tests:** Evaluate known surfaces at known parameters and compare to analytic results. Verify normals. Verify tessellation produces watertight meshes (no T-junctions).

### 4.3 Phase 33: Half-Edge B-Rep Topology

**Goal:** The data structure that defines how vertices, edges, and faces connect — the foundation for Boolean operations, fillets, and feature editing.

**Module:** `hz::topology`

**Design:**

**Data structure (half-edge):**
```
Solid
├── Shell[] (closed shells; a solid with a cavity has 2 shells)
│   ├── Face[]
│   │   ├── OuterLoop (Wire — ordered list of half-edges)
│   │   ├── InnerLoop[] (Wires — holes in the face)
│   │   └── Surface* (NurbsSurface geometry)
│   ├── Edge[]
│   │   ├── HalfEdge (forward, belongs to Face A)
│   │   ├── HalfEdge (twin, belongs to Face B)
│   │   └── Curve* (NurbsCurve geometry)
│   └── Vertex[]
│       └── Point (Vec3)
```

**TopologyID genealogy system (critical — solves the Topological Naming Problem):**
- Every `Vertex`, `Edge`, and `Face` carries a `TopologyID` — a deterministic hash encoding *how* the entity was created
- IDs are based on genealogy, NOT memory addresses or array indices
- Phase 34 (primitives): `hash(primitiveType, faceRole)` — e.g., `hash("box", "top")`
- Phase 35 (extrude): `hash(featureID, sketchEntityID, "lateral")` for extruded side faces
- Phase 36 (Boolean): split faces inherit parent ID as prefix: `hash(parentFaceID, "split", intersectionIndex)`
- Phase 37 (fillet): stores `TopologyID` edge references. On rebuild, queries B-Rep for matching IDs.
- **Resolution algorithm:** genealogy match — search current B-Rep for entities whose `TopologyID` descends from the stored reference. Exact match preferred; prefix/ancestry match handles splits.

**Euler operators (the only safe way to modify topology):**
- `makeVertexFaceSolid()` — create initial topology
- `makeEdgeVertex()` — split a vertex into an edge + new vertex
- `makeEdgeFace()` — split a face with a new edge
- `killEdgeVertex()` / `killEdgeFace()` — reverse operations
- `makeVertexFaceShell()` — add inner shell (hollowed solids)
- All operators maintain Euler's formula: V - E + F = 2 per shell

**Topological queries:**
- `face.adjacentFaces()`, `edge.leftFace()`, `edge.rightFace()`, `vertex.incidentEdges()`
- `solid.isValid()` — Euler formula check, manifold check, orientation check

**Memory management:** Stable ID + pool allocation (not raw pointers). Critical for undo/redo — topology changes must be reversible.

**Debug visualization:** Wireframe overlay showing edges (green), face normals (arrows), vertices (dots). Essential for kernel development.

**Tests:** Create cube and cylinder via Euler operators. Verify Euler formula. Verify adjacency queries. Verify manifold invariants.

### 4.4 Phase 34: Primitive Solid Construction

**Goal:** Procedurally construct basic solids — box, cylinder, sphere, cone, torus — via Euler operators with NURBS geometry binding.

**Module:** `hz::modeling`

**Design:**
- Factory functions: `makeBox(w, h, d)`, `makeCylinder(r, h)`, `makeSphere(r)`, `makeCone(rBot, rTop, h)`, `makeTorus(R, r)`
- Each factory builds the solid via Euler operators (ensuring invariants) and binds NURBS geometry to faces/edges
- TopologyIDs: role-based — `hash("box", "top")`, `hash("cylinder", "lateral")`, etc. Stable across rebuilds.
- `Solid::tessellate(tolerance) → MeshData` — iterates all faces, tessellates each trimmed NURBS surface, merges into watertight mesh
- **User-facing:** 3D Primitive toolbar. Camera switches to perspective with orbit/pan/zoom. First 3D visuals in Horizon.
- Renders through existing Phong shader via `SceneNode` + `MeshData`

**Tests:** Each primitive's Euler formula. Tessellation produces closed meshes. Normals point outward.

### 4.5 Phase 35: Extrude & Revolve

**Goal:** The core parametric workflow — sketch a 2D profile, extrude or revolve it into a 3D solid.

**Module:** `hz::modeling`

**Design:**

**Extrude:**
- Input: closed 2D profile (from `Sketch` on `SketchPlane`) + direction + distance
- Algorithm: validate closed loop → for each profile edge, create ruled NURBS surface → create top/bottom cap faces → build B-Rep topology via Euler operators → orient normals outward
- TopologyIDs: `hash(featureID, sketchEntityID, "lateral"|"cap_top"|"cap_bottom")` — faces traced back to their source sketch entities
- Variants: blind (fixed distance), through-all, mid-plane. Up-to-surface deferred.

**Revolve:**
- Input: closed 2D profile + axis + angle (0-360°)
- Algorithm: create surfaces of revolution (NURBS) → build caps if angle < 360° → handle degenerate axis-coincident edges → build B-Rep
- TopologyIDs: same genealogy pattern as extrude

**Feature Tree:**
- `FeatureTree` in `hz::modeling`: ordered list of features (sketch + operation)
- Each feature stores its inputs (sketch reference, parameters)
- Editing a feature re-executes all subsequent features (parametric rebuild)
- Features store TopologyID references to inputs, resolved via genealogy on rebuild

**Profile validation:** The 2D profile must be a single closed loop with no self-intersections. Implement subdivision-based NURBS curve-curve intersection: recursively bisect both curves until bounding boxes are small, then Newton-refine.

**User-facing:** Extrude and Revolve tools. Draw closed profile → click Extrude → drag or type distance → solid appears. The "hello world" of Horizon 3D.

**Undo:** `ExtrudeCommand`/`RevolveCommand` store feature inputs. Undo removes feature from tree and regenerates.

**Tests:** Extrude rectangle → 6-face box. Extrude circle → 3-face cylinder. Revolve rectangle → hollow cylinder. Feature tree replay produces identical solid.

### 4.6 Phase 36: Boolean Operations

**Goal:** Union, subtract, intersect — the hardest algorithm in computational geometry. Required for cut-extrudes, holes, and combining parts.

**Module:** `hz::modeling`

**Design (face-classification approach):**

1. **Surface-surface intersection (SSI):** Find intersection curves between all face pairs of two input solids. Subdivide surfaces into patches, R*-tree bounding-box overlap for candidate pairs, Newton iteration for intersection curves.
2. **Edge splitting:** Split face boundary edges at intersection crossings. Insert new vertices and edges into both B-Rep structures.
3. **Face classification:** For each face fragment, determine INSIDE/OUTSIDE/ON the other solid via ray-casting from interior point.
4. **Face selection:** Union = keep OUTSIDE from both. Subtract = OUTSIDE of A + INSIDE of B (normals reversed). Intersect = INSIDE from both.
5. **Topology reconstruction:** Sew selected faces via Euler operators into valid B-Rep. Verify Euler formula and manifold properties.

**TopologyID handling:** Split faces inherit parent genealogy: `hash(parentFaceID, "split", intersectionIndex)`. Classification doesn't change identity; only splitting does.

**Robustness:**
- **Exact predicates** (Shewchuk-style adaptive precision arithmetic) for all classification decisions (orientation tests, inside/outside)
- **Tolerance-based vertex merging** (vertices within `Tolerance::kLinear` treated as identical)
- **Degenerate case handling:** coincident faces (merge or skip), tangent intersections, sliver face removal

**User-facing:** Cut-extrude (extrude + subtract). Boolean toolbar (union, subtract, intersect). Create box + cylinder → subtract → hole. Breakthrough feature.

**Expectation:** This phase takes 2-3x longer than others. Get the easy 90% working first (perpendicular planar faces, simple cylinders), then iteratively harden edge cases.

**Tests:** Box-box intersection (all configurations). Cylinder through box. Box-box union. Verify watertight output. Verify Euler formula. 100 random-transform Boolean stress test.

### 4.7 Phase 37: Fillet & Chamfer on Solids

**Goal:** Rounded and beveled edges on solid bodies.

**Module:** `hz::modeling`

**Design:**

**Edge fillet:**
- Input: one or more edges + radius
- Algorithm: compute rolling-ball fillet surface (locus of sphere rolling along edge tangent to both adjacent faces). Planar-planar = cylindrical NURBS. Planar-cylindrical = toroidal NURBS. General = marching algorithm with swept cross-section.
- Trim adjacent faces to accommodate fillet surface. Insert fillet face into B-Rep.
- **Scope restriction (Era 1):** Single continuous edge chains only. No vertex blends (where 3+ filleted edges meet). No variable-radius fillets. Tool validates no intersecting fillet zones and refuses with clear error if detected.
- Vertex blends and variable-radius deferred to Phase 61 (Era 3).

**Edge chamfer:** Replace edge with planar face at specified distance. Two variants: equal-distance and two-distance.

**TopologyID:** Fillet stores edge references as TopologyIDs. On rebuild, finds edges via genealogy match.

**Tests:** Fillet one edge of box. Fillet all edges of box (sequential, no vertex blends). Fillet cylinder-to-plane edge. Verify watertight B-Rep. Verify smooth normals at boundaries.

### 4.8 Phase 38: Feature Tree UI & Parametric Editing

**Goal:** The feature tree panel — the signature UI of parametric CAD.

**Module:** `hz::ui`

**Design:**
- `FeatureTreePanel` (QDockWidget): ordered list of features showing sketch + operation hierarchy
- **Double-click to edit:** Opens sketch or changes parameters. Triggers parametric rebuild — re-execute all features from modified point forward.
- **Drag to reorder:** Within dependency constraints. Solid regenerates.
- **Rollback bar:** Visual divider in tree. Features below are suppressed. Drag to roll back/forward.
- **Feature failure handling:** If rebuild fails (e.g., fillet radius too large), mark failed feature red, show error. Solid renders up to last successful feature.
- **TNP integration:** Rebuild uses TopologyID genealogy resolution to reconnect downstream features to regenerated topology. Failed resolution = feature error (red in tree) with message ("Edge reference lost — select replacement edge").

**Architectural pattern:** Feature tree is a build script. Each feature is a pure function: `(inputSolid, parameters) → outputSolid`. Rebuild replays from scratch (or from modified feature).

### 4.9 Phase 39: 3D Viewport & Navigation Polish

**Goal:** Professional-grade 3D interaction.

**Module:** `hz::render`, `hz::ui`

**Design:**
- **Navigation:** Middle-mouse orbit (SolidWorks-style), scroll zoom-to-cursor, shift+middle pan. Configurable mouse mapping.
- **Edge rendering:** Silhouette edges (black), sharp edges (dark gray), sketch edges (blue)
- **Transparency mode** for see-through editing
- **Section plane tool** (Boolean clip for visualization)
- **GPU color-picking** for 3D selection: render entity IDs to offscreen framebuffer, read pixel under cursor. Faster and more robust than ray-casting for complex geometry.
- **PBR-lite:** Metallic-roughness model (not full PBR — no environment maps yet). Matte gray default.

### 4.10 Phase 40: Stabilization & Kernel Hardening

**Goal:** Stress-test and harden the kernel.

**Design:**
- **Fuzz testing:** Random extrude/revolve/Boolean sequences. Verify Euler formula and manifold invariants after every operation.
- **TNP regression tests:** Edit sketch → rebuild → verify all downstream features resolve correctly via TopologyID. This is the critical integration test.
- **Performance profiling:** Benchmark Booleans on increasingly complex solids. Profile tessellation. Optimize SSI (likely bottleneck).
- **Regression suite:** 50+ integration tests for sketch → extrude → Boolean → fillet pipeline.
- **`.hcad` v14:** Serialize B-Rep topology + feature tree + NURBS geometry + TopologyIDs. Full save/load round-trip verification.
- **Memory audit:** AddressSanitizer on Linux CI. Verify Euler operations don't leak topology elements.

---

## 5. Era 2: Assembly Modeling & Advanced Features (Phases 41-52)

### 5.1 Phase 41: Multi-Document Architecture & Part Files

**Goal:** Support multiple document types for the parts → assemblies → drawings workflow.

**Module:** `hz::doc`

**Design:**
- **Document types:** `.hzpart` (single solid + feature tree), `.hzasm` (hierarchical part references + mates), `.hzdwg` (deferred to Era 3)
- `DocumentManager`: manages open documents, tracks cross-document references, handles external file-change notifications
- **In-memory caching:** Referenced parts cache tessellated mesh. Feature tree only loaded on edit.
- **Lightweight vs. resolved mode:** Lightweight = tessellation + transform only (for large assemblies). Resolved = full feature tree in memory (for editing).

### 5.2 Phase 42: Assembly Mates

**Goal:** Position parts relative to each other via geometric constraints.

**Module:** `hz::modeling`

**Design:**

**Mate types:** Coincident, Concentric, Distance, Angle, Parallel, Perpendicular, Tangent, Fixed

**Assembly solver:** Extends Newton-Raphson architecture from sketch solver. Each part has 6 DOF (tx, ty, tz, rx, ry, rz). Mates contribute equations reducing DOF.

**Kinematic graph pre-analysis (critical):**
- Before numerical solve, build constraint graph (parts = nodes, mates = edges)
- Run rigid sub-group detection: mutually fully-constrained parts collapse to single rigid body
- Feed reduced system to Newton-Raphson solver
- Detect and report redundant constraints to user

**DOF visualization:** Parts with remaining freedom are draggable along free axes. Fully constrained = locked. Under-constrained = warning.

**Mate references use TopologyIDs:** `(partA, faceTopologyID_A, partB, faceTopologyID_B)`. Part rebuilds resolve mates via genealogy.

### 5.3 Phase 43: Loft & Sweep

**Goal:** Complex 3D shapes from profile interpolation and path sweeping.

**Module:** `hz::modeling`

**Design:**

**Loft:** Two or more closed profiles on sketch planes + optional guide curves. NURBS surfaces interpolating between profiles. Chord-length parameterization for profiles, centripetal for loft direction. Same-vertex-count profiles required initially; automatic vertex matching deferred.

**Sweep:** Closed profile + 3D path curve. Profile's local frame moves along path (Frenet frame or user-specified). Optional twist angle. NURBS surface of revolution generalization.

**Tests:** Cylinder as trivial loft. Known swept shapes vs. analytic comparison.

### 5.4 Phase 44: Shell & Draft

**Goal:** Thin-wall features with robust self-intersection handling.

**Module:** `hz::modeling`

**Design:**

**Shell:** Input: solid + wall thickness + faces to remove. Algorithm: offset all faces inward → remove open faces → intersect offset surfaces → build B-Rep connecting outer and inner shells.

**Self-intersection detection (critical):** After computing offset surfaces, tessellate at fine resolution, detect triangle-triangle intersections (swallowtails), trace intersection curves, trim loops, rebuild surface. Refuse to shell if wall thickness exceeds minimum radius of curvature with clear error message.

**Draft:** Tilt selected faces by draft angle relative to pull direction. Rebuild adjacent topology.

### 5.5 Phase 45: Linear & Circular Pattern

**Goal:** Repeated features with dual-mode architecture for performance.

**Module:** `hz::modeling`

**Design:**

**Geometry Pattern (default, 100-1000x faster):** Evaluate source feature once. Duplicate resulting B-Rep faces, transform topology, batch-Boolean into solid as single operation. TopologyIDs: `hash(sourceFeatureID, "pattern", instanceIndex)`.

**Feature Pattern (opt-in):** Re-execute feature tree per instance. Required only when patterned feature adapts to variable end-conditions (e.g., "extrude up to next surface" where each instance hits a different surface).

**UI:** Checkbox toggle "Geometry Pattern" (default) / "Feature Pattern". Individual instance suppression for irregular patterns.

### 5.6 Phase 46: Reference Geometry

**Goal:** Construction geometry for complex modeling — datum planes, axes, points.

**Module:** `hz::modeling`

**Design:**
- **Datum plane:** Offset from face, through 3 points, angle to face, midplane between faces
- **Datum axis:** Intersection of planes, center of cylinder, through 2 points
- **Datum point:** At vertex, face center, edge intersection
- Non-geometric features in the tree — used as inputs by sketches and features
- Render: translucent rectangles (planes), dashed lines (axes), dots (points)

### 5.7 Phase 47: Python Scripting API (Phase 1)

**Goal:** Embedded Python scripting for automation and power users.

**Module:** `hz::scripting` (new)

**Design:**
- **pybind11 embedding** with bundled CPython interpreter
- **API modules:** `horizon.math`, `horizon.geometry`, `horizon.draft`, `horizon.model`, `horizon.doc`, `horizon.ui` (read-only initially)
- **Script console:** Embedded Python REPL panel in UI
- **Macro recording:** GUI actions emitted as equivalent Python API calls
- **Thread safety:** Scripts run on main thread (GIL). Long-running scripts yield to event loop periodically.
- **Undo integration:** Script operations push commands to undo stack

### 5.8 Phase 48: Collision Detection & Interference Checking

**Goal:** Detect overlapping parts in assemblies.

**Module:** `hz::modeling`

**Design:**
- **Broad phase:** R*-tree bounding-box overlap. O(n log n).
- **Narrow phase:** Boolean intersection test on overlapping pairs. Nonzero volume = interference.
- **Visualization:** Interference volumes in translucent red. Conflict list panel.

### 5.9 Phase 49: Measurement & Mass Properties

**Goal:** Engineering measurements and material analysis.

**Module:** `hz::modeling`

**Design:**
- **3D measurements:** Point-to-point, face-to-face distance, edge angle, radius of curvature
- **Mass properties:** Volume (divergence theorem), surface area (triangle sum), CG (volume-weighted centroid), moments of inertia (volume integrals)
- **Material assignment:** Density per part. Preset library (steel, aluminum, titanium, ABS plastic) + custom.

### 5.10 Phase 50: STEP AP242 Import/Export

**Goal:** Industry-standard 3D interchange format.

**Module:** `hz::io`

**Design:**
- **Integrate STEPcode** (BSD-licensed) for EXPRESS schema parsing. Do NOT write a custom lexer for ISO 10303-21.
- STEPcode deserializes STEP file → memory structs. Horizon writes the mapping layer: STEPcode structs → `hz::topology` B-Rep.
- **Target AP242** (not AP203 — AP242 supports colors, layers, GD&T, and assembly product structure)
- Era 2 scope: geometry + topology only. Product structure (assembly hierarchy) in Era 3.

### 5.11 Phase 51: Native Binary Format

**Goal:** High-performance file format for 3D data.

**Module:** `hz::io`

**Design:**
- **FlatBuffers-based** `.hzpart` / `.hzasm` — zero-copy deserialization
- Structure: header → feature tree → B-Rep topology → NURBS geometry → tessellation cache (optional)
- **Tessellation caching:** Pre-tessellated meshes stored in file. On load, render immediately from cache while feature tree rebuilds in background (progressive loading).
- Keep `.hcad` JSON for 2D-only drawings. New 3D documents use binary format.

### 5.12 Phase 52: Era 2 Stabilization

**Goal:** Harden assembly and advanced features.

**Design:**
- Assembly solver: 100-part assembly with 200+ mates, solve < 1 second
- Boolean robustness: full test suite under random coordinate perturbations (±ε). No crashes, no invalid topology.
- STEP round-trip: export → import → export → binary compare
- Feature tree rebuild: 50-feature part < 5 seconds
- Python API: scripted creation of complete part (sketch + extrude + fillet + pattern)
- Memory: 100-part assembly < 2 GB RAM

---

## 6. Era 3: Drawings, Simulation & PDM (Phases 53-64)

### 6.1 Phase 53: 2D Drawing Generation from 3D Models

**Goal:** Produce manufacturing drawings from 3D geometry — HLR, section views, model-driven dimensioning.

**Module:** `hz::modeling` (view projection), `hz::doc` (drawing document)

**Design:**
- **Drawing document (`.hzdwg`):** References `.hzpart`/`.hzasm` files. Contains sheets (paper layouts).
- **Hidden Line Removal (HLR):**
  - Project B-Rep edges onto view plane
  - **Performance (critical):** Project face bounding boxes into 2D view plane, build 2D R*-tree. For each edge, query R*-tree for overlapping face projections only. O(N log M) instead of O(N*M).
  - Classify edges: visible (solid), hidden (dashed), tangent (thin)
  - Front-to-back painter's algorithm with interval tree for incremental visible/hidden marking
- **Standard views:** Front, Top, Right, Isometric — auto-generated
- **Section views:** Boolean intersection with half-space → project cut profile with cross-hatching
- **Detail views:** Circular crop at enlarged scale
- **Auxiliary views:** Projection perpendicular to selected angled face
- **Model association:** Views store source part + direction + scale + TopologyID references. Model changes → views regenerate.
- **Model-driven dimensioning:** Dimensions on drawing views snap to projected edges and reference 3D TopologyIDs. Dimension values auto-update on model change.

### 6.2 Phase 54: GD&T

**Goal:** ASME Y14.5 / ISO 1101 geometric tolerancing.

**Module:** `hz::draft`

**Design:**
- Feature control frames with compartments (symbol | tolerance | datum refs)
- Tolerance types: flatness, straightness, circularity, cylindricity, perpendicularity, parallelism, angularity, position, concentricity, symmetry, runout, profile
- Datum symbols: `[A]`, `[B]`, `[C]`
- Semantic attachment to 3D TopologyIDs — repositions on drawing regeneration
- QPainter rendering on drawing views

### 6.3 Phase 55: BOM & Balloons

**Goal:** Assembly documentation for manufacturing.

**Module:** `hz::ui`

**Design:**
- BOM generation: traverse assembly tree, collect unique parts with quantities
- Balloon annotations: numbered circles with leader lines, linked to BOM item numbers
- Custom columns (cost, supplier, part number). CSV/Excel export.

### 6.4 Phase 56: Print/Plot Pipeline

**Goal:** Exact-scale printed output.

**Module:** `hz::ui`, `hz::io`

**Design:**
- QPrinter → PDF or physical printer at specified scale
- ISO 128 line weights (0.25mm, 0.35mm, 0.5mm, 0.7mm)
- Multi-sheet support (A0-A4, Letter, Legal, custom)
- Parametric title blocks (ISO, ANSI, DIN templates) with auto-populated fields

### 6.5 Phase 57: FEA — Linear Static Analysis

**Goal:** Structural validation via finite element analysis.

**Module:** `hz::simulation` (new)

**Design:**

**Defeaturing/healing (critical pre-step):**
- Before meshing, scan B-Rep for edges shorter than mesh-size tolerance, faces with area < tolerance², gaps between near-coincident boundaries
- Collapse short edges, merge sliver faces, heal gaps
- Produces "virtual" simplified B-Rep for meshing only — original untouched
- Defeaturing tolerance user-adjustable (aggressive for coarse mesh, conservative for fine)

**Meshing:** Surface mesh from B-Rep tessellation → volume mesh via TetGen (MIT-licensed). Mesh quality metrics (aspect ratio, Jacobian). Adaptive refinement.

**Solver:** CalculiX integration (GPL-2.0). Write mesh + BCs + material to Abaqus `.inp` format → launch subprocess → read results.

**Boundary conditions UI:** Fixed support (face selection), applied loads (pressure, point force, remote force), material assignment (Young's modulus, Poisson's ratio, density + preset library).

**Results:** Von Mises stress contour, displacement, factor of safety, deformed shape overlay (exaggerated). Min/max annotations.

### 6.6 Phase 58: Thermal Analysis

**Goal:** Heat transfer simulation.

**Module:** `hz::simulation`

**Design:**
- Same mesh from Phase 57, different solver mode
- BCs: fixed temperature, heat flux, convection, radiation (Stefan-Boltzmann)
- Results: temperature distribution, heat flux vectors
- Thermal-structural coupling: temperature results → thermal loads → structural solver

### 6.7 Phase 59: PDM — Local Version Control

**Goal:** Design revision tracking with semantic diffing.

**Module:** `hz::pdm` (new)

**Design:**
- Each save creates a revision (timestamp, author, comment)
- **Semantic diff (critical):** Diff the Feature Tree + ParameterTable as structured data (JSON serialization), NOT binary FlatBuffers diff. Feature additions/removals/parameter changes are human-readable.
- Tessellation caches stored as periodic full snapshots (every Nth revision), not delta-compressed.
- Revision browser: timeline UI, click to preview, diff highlighting
- Branching: "Save As Variant." Compare but no merge (merging conflicting fillet radii is undefined).
- Storage: `.hzarchive` directory alongside part file

### 6.8 Phase 60: PDM — Multi-User Vault

**Goal:** Team collaboration with pessimistic locking.

**Module:** `hz::pdm`

**Design:**
- Shared directory vault with manifest (JSON) tracking lock state + revisions
- Check-out/check-in: only one user edits at a time. Others see last checked-in version (read-only).
- Lock display: shows current editor. Lock release request.
- Notifications: part-updated alerts for dependent assembly users.
- File-system based — no server process. Works on NFS/SMB/cloud-synced folders.

### 6.9 Phase 61: Advanced Fillet & Chamfer

**Goal:** Vertex blends and variable-radius fillets (deferred from Era 1).

**Module:** `hz::modeling`

**Design:**
- **Vertex blends:** Where 3+ filleted edges meet, generate N-sided surface patch. Coons patches (4-sided) and Gregory patches (N-sided) for G1 continuity.
- **Variable-radius fillet:** Radius varies along edge, defined by radius-at-parameter table. Interpolated rolling ball.
- **Face fillet:** Smooth transition between non-adjacent faces.

### 6.10 Phase 62: Sheet Metal Module

**Goal:** Design-to-flat-pattern workflow for sheet metal fabrication.

**Module:** `hz::modeling`

**Design:**
- Features: base flange (extrude thin), edge flange, miter flange, hem, tab/slot, bend relief
- Flat pattern computation: unfold 3D body to 2D pattern accounting for bend allowance (K-factor per material/thickness)
- DXF flat pattern export for laser/plasma/waterjet cutting

### 6.11 Phase 63: Python API (Phase 2)

**Goal:** Full automation and plugin ecosystem.

**Module:** `hz::scripting`

**Design:**
- Full modeling API: sketch, extrude, fillet, Boolean, pattern, assembly mates via Python
- Drawing API: programmatic standard views, dimensions, GD&T, BOM
- Batch CLI: `horizon.exe --script convert_step.py` (headless mode)
- Plugin system: Python packages in plugin directory declaring menu items, tools, panels
- Plugin package manager: JSON registry on GitHub, `horizon --install-plugin <name>`

### 6.12 Phase 64: Era 3 Stabilization

**Goal:** Validate the professional workflow.

**Design:**
- Drawing regeneration: modify part → verify views update, dimensions recompute, GD&T repositions
- FEA validation: cantilever beam and pressurized cylinder vs. textbook solutions (< 5% error)
- STEP AP242 round-trip with GD&T
- PDM stress test: 10 users, 500 parts, concurrent check-out/check-in, no corruption
- Sheet metal: unfold → refold → verify geometry match within tolerance
- Performance: 1000-part assembly loads < 10s (lightweight). Drawing with 20 views regenerates < 5s.

---

## 7. Era 4: Cloud, Rendering & Market Parity (Phases 65-80)

### 7.1 Phase 65: Vulkan/Metal Rendering Backend

**Goal:** Modern rendering API, macOS support, multi-threaded draw submission.

**Module:** `hz::render`

**Design:**
- **Rendering Abstraction Layer (RAL):** `RenderBackend` interface with `VulkanBackend` and `OpenGLBackend` (legacy fallback)
- RAL interface: `createBuffer()`, `createTexture()`, `createShader()`, `beginPass()`, `draw()`, `endPass()`, `submitCompute()`
- MoltenVK for macOS (Vulkan → Metal translation)
- Multi-threaded command buffer submission for large assemblies
- Migration strategy: build alongside OpenGL, feature-flag at startup, OpenGL becomes fallback

### 7.2 Phase 66: GPU Compute Acceleration

**Goal:** GPU-accelerated tessellation and spatial queries.

**Module:** `hz::render`, `hz::geometry`

**Design:**
- **Compute shaders only (critical):** No hardware tessellation shaders. Upload NURBS control points + knots → compute shader evaluates surface → output vertex/index buffers to VRAM. Translates cleanly through MoltenVK → Metal.
- GPU spatial queries: R*-tree broad-phase on compute (bounding box upload → parallel overlap → candidate pair readback)
- Adaptive LOD: distance-based tessellation density computed per-frame on GPU

### 7.3 Phase 67: PBR Rendering & Materials

**Goal:** Realistic material visualization.

**Module:** `hz::render`

**Design:**
- Metallic-roughness workflow (glTF 2.0 compatible): base color, metallic, roughness, normal map, AO
- Material library: brushed aluminum, polished steel, matte plastic, rubber, glass, carbon fiber, wood
- HDR environment maps for IBL (image-based lighting). Prefiltered specular + irradiance diffuse.
- Per-part or per-face material assignment

### 7.4 Phase 68: Ray-Traced Rendering

**Goal:** Photorealistic on-demand renders.

**Module:** `hz::render`

**Design:**
- **Embree** (Intel, Apache 2.0) for CPU path tracing — avoids Vulkan RT driver fragmentation
- BVH from tessellated meshes for ray-scene intersection
- Monte Carlo progressive refinement (noisy at 1s, clean at 10s, photorealistic at 60s)
- Camera: focal length, depth of field, exposure
- Output: PNG, EXR (HDR). Viewport overlay with progressive refinement.

### 7.5 Phase 69: Cloud Sync

**Goal:** Local-first with optional cloud replication.

**Module:** `hz::cloud` (new)

**Design:**
- Horizon is always fully functional offline. Cloud is a sync layer.
- Cloud replicates vault contents (from Phase 59-60) to server
- Pessimistic locking extends to cloud — check-out locks propagate
- Server: lightweight REST API (Rust or Go). File storage (S3-compatible), lock manager, OAuth2 auth, webhooks.
- Self-hostable: Docker container. No vendor lock-in.

### 7.6 Phase 70: Live Collaboration

**Goal:** Real-time presence with feature-level locking.

**Module:** `hz::cloud`

**Design:**
- **Feature-level token locking (critical):** NOT OT/CRDT on the feature tree. When User A opens Sketch 1, that sketch and downstream features are locked. User B sees User A's cursor and edits in real-time but cannot modify locked features. Edit token releases on sketch confirm + rebuild.
- WebSocket for real-time updates
- Presence: user avatars on viewport, colored selection per user
- Session-based: one user starts "Live Session," others join via link

### 7.7 Phase 71: CAM Integration

**Goal:** 2.5D milling toolpath generation and G-code export.

**Module:** `hz::cam` (new)

**Design:**
- **Integrate OpenCAMLib** (LGPL, C++) for toolpath computation — do NOT write toolpath offset math from scratch. OpenCAMLib handles Voronoi-based pocketing, waterline offsets, medial-axis, gouge checking.
- Horizon builds: CAM UI, visual toolpath simulation, G-code post-processor architecture
- Operations: face milling, pocket milling, contour milling, drilling
- G-code: ISO 6983 with post-processors for Fanuc, Haas, LinuxCNC, Grbl
- Toolpath visualization: animated tool movement in 3D viewport

### 7.8 Phase 72: Kinematics & Motion

**Goal:** Mechanism simulation for assemblies with moving parts.

**Module:** `hz::modeling`

**Design:**
- Kinematic joints: revolute (hinge), prismatic (slider), cylindrical, gear, cam
- Motion study: motor inputs (angular/linear velocity), time-step simulation
- Animation: play back in viewport, export MP4
- Interference during motion: collision detection per time step

### 7.9 Phase 73: Advanced Simulation

**Goal:** Extended FEA capabilities.

**Module:** `hz::simulation`

**Design:**
- Nonlinear static (plasticity, large deformation, contact)
- Dynamic analysis (modal, harmonic, transient)
- Fatigue (S-N curve, cyclic loading, safety factor contour)
- CFD (deferred — evaluate in Era 4+ based on demand): if pursued, OpenFOAM integration for external flow using same pattern as CalculiX (write mesh + BCs, launch subprocess, read results)

### 7.10 Phase 74: Configuration Management

**Goal:** Design tables for part families.

**Module:** `hz::modeling`

**Design:**
- Spreadsheet-like design table: rows = configurations, columns = design variables
- Configuration selector: switch → feature tree rebuilds with row values
- Assembly configurations: specify part configuration per instance
- BOM integration: list all or active configurations

### 7.11 Phase 75: Surfacing Workbench

**Goal:** Freeform NURBS surface modeling for consumer product design.

**Module:** `hz::modeling`

**Design:**
- Boundary surface (3-4 edge curves, G0/G1/G2 continuity)
- Fill surface (N-sided boundary, Gregory patches)
- Surface trim, extend, thicken (surface → solid), knit (join → watertight solid)

### 7.12 Phase 76: Import/Export Ecosystem

**Goal:** Comprehensive file format support.

**Module:** `hz::io`

**Design:**
- IGES import/export (legacy NURBS format)
- Parasolid `.x_t` import (documented text format — competitive advantage)
- STL/3MF export (3D printing)
- glTF export (web/AR/VR, PBR materials map directly)
- PDF 3D export (U3D/PRC for non-CAD sharing)
- DWG import (ODA libraries or LibreDWG)

### 7.13 Phase 77: Accessibility, Localization & Documentation

**Goal:** Worldwide usability and onboarding.

**Module:** `hz::ui`, documentation

**Design:**
- i18n: Qt `tr()` for all strings. Translations: EN, DE, JA, ZH, ES, FR, KO
- Accessibility: keyboard navigation, screen reader, high-contrast theme
- Documentation: context-sensitive F1 help, online API reference, video tutorials
- Onboarding: first-run wizard, sample files library

### 7.14 Phase 78: Large Assembly Optimization

**Goal:** 10,000+ part assemblies at 60 FPS.

**Module:** `hz::render`, `hz::doc`

**Design:**
- Extended lightweight mode: tessellation + transform only on disk until user edits
- Frustum culling: R*-tree query with view frustum volume
- Occlusion culling: hardware queries (Vulkan) or hierarchical Z-buffer
- Instanced rendering: identical parts use GPU instancing (one draw call for 1000 bolts)
- Progressive loading: cached tessellation renders immediately; feature trees resolve in background
- Octree spatial partitioning: only traverse visible nodes

**Target:** 10,000-part assembly opens < 5s. Orbit at 60 FPS. Zoom-to-part loads details on demand.

### 7.15 Phase 79: Plugin Marketplace

**Goal:** Community-driven feature expansion.

**Module:** `hz::scripting`

**Design:**
- Plugin API: custom menu items, toolbar buttons, dock panels, tools, entity types, file formats, simulation types, feature tree features
- Web marketplace: browse, install from within Horizon
- Optional paid plugins (80/20 revenue split)
- Plugin sandboxing: restricted Python environment, explicit permissions for filesystem/network

### 7.16 Phase 80: 1.0 Release

**Goal:** Production-ready product launch.

**Design:**
- 500+ automated regression tests
- Published benchmarks vs. SolidWorks and FreeCAD
- Security audit (plugin sandbox, file format fuzzing, network protocol)
- Installers: Windows (MSI/MSIX), Linux (AppImage + Flatpak + .deb), macOS (DMG)
- Website, forum, contribution guide, public roadmap
- Marketing: "The open-source SolidWorks alternative"

---

## 8. Architecture Cross-Cutting Concerns

### 8.1 The Topological Naming Problem (TNP)

The TNP is the single most critical architectural concern. Every topological entity (vertex, edge, face) carries a deterministic `TopologyID` based on genealogy:

| Feature Type | TopologyID Scheme |
|-------------|-------------------|
| Primitive | `hash(primitiveType, faceRole)` |
| Extrude/Revolve | `hash(featureID, sketchEntityID, role)` |
| Boolean (split) | `hash(parentFaceID, "split", intersectionIndex)` |
| Boolean (survive) | Parent ID passes through unchanged |
| Fillet | `hash(featureID, "fillet", sourceEdgeID)` |
| Pattern | `hash(sourceFeatureID, "pattern", instanceIndex)` |

**Resolution:** On rebuild, features search B-Rep for entities matching stored TopologyIDs. Exact match preferred; prefix/ancestry match for split entities. Failed resolution = feature error.

### 8.2 Threading Model

The kernel (constraint solving, Boolean operations, tessellation, feature tree rebuild) runs on the **main thread** in Era 0-2. This is pragmatic for a solo developer — thread safety in a B-Rep kernel is extremely difficult to retrofit.

**Era 3+:** Introduce background threading for:
- Feature tree rebuild (worker thread, UI shows progress bar, result swapped to main thread on completion)
- Tessellation (parallel per-face tessellation using `std::async` or a thread pool)
- FEA meshing and solver launch (already subprocess-based, naturally async)
- Drawing HLR computation (per-view parallelism)

**Rule:** The B-Rep topology data structure is **never mutated from multiple threads**. Parallelism is achieved by partitioning independent work (per-face tessellation, per-view HLR), not by concurrent mutation.


### 8.3 Performance Budgets

| Scenario | Target |
|----------|--------|
| Snap query (10k entities) | < 1ms |
| Constraint solve (50 constraints) | < 100ms |
| Feature tree rebuild (50 features) | < 5s |
| Boolean (box - cylinder) | < 500ms |
| Assembly solve (200 mates) | < 1s |
| Assembly load (10k parts, lightweight) | < 5s |
| Drawing view regen (20 views) | < 5s |
| Viewport FPS (10k parts) | 60 FPS |
| FEA mesh (100k tets) | < 30s |

### 8.4 External Library Integration

| Library | Purpose | License | Phase |
|---------|---------|---------|-------|
| Eigen | Matrix operations (solver) | MPL-2.0 | Existing |
| nlohmann-json | JSON serialization | MIT | Existing |
| spdlog | Logging | MIT | Existing |
| Google Test | Unit testing | BSD-3 | Existing |
| Qt6 | UI framework + OpenGL | LGPL-3.0 | Existing |
| FlatBuffers | Binary serialization | Apache-2.0 | Phase 51 |
| pybind11 | Python embedding | BSD-3 | Phase 47 |
| STEPcode | STEP file parsing | BSD-3 | Phase 50 |
| TetGen | Tetrahedral meshing | AGPLv3 (evaluate Netgen as MIT alternative) | Phase 57 |
| CalculiX | FEA solver | GPL-2.0 | Phase 57 |
| Embree | Ray tracing | Apache-2.0 | Phase 68 |
| OpenCAMLib | CAM toolpaths | LGPL | Phase 71 |

### 8.5 File Format Evolution

| Version | Format | Contents | Phase |
|---------|--------|----------|-------|
| v1-v12 | `.hcad` JSON | 2D entities, layers, blocks, constraints, dimensions | Existing |
| v13 | `.hcad` JSON | + expressions, design variables, sketch planes | Phase 30 |
| v14 | `.hcad` JSON | + B-Rep topology, feature tree, NURBS, TopologyIDs | Phase 40 |
| v1 | `.hzpart` FlatBuffers | 3D part: feature tree + B-Rep + NURBS + tess cache | Phase 51 |
| v1 | `.hzasm` FlatBuffers | Assembly: part refs + mates + hierarchy | Phase 51 |
| v1 | `.hzdwg` FlatBuffers | Drawing: sheets + views + annotations | Phase 53 |

---

## 9. Risk Register

| Risk | Impact | Mitigation |
|------|--------|------------|
| Boolean robustness (Phase 36) | Kernel unusable if Booleans fail on edge cases | Exact predicates + tolerance merging. Budget 2-3x time. Fuzz test aggressively. |
| TNP resolution failures | Feature tree breaks on rebuild | Prefix/ancestry matching. Extensive regression tests from Phase 40. |
| Shell self-intersections (Phase 44) | Shell tool crashes on concave geometry | Self-intersection detection + user-friendly error when thickness exceeds curvature |
| STEP complexity (Phase 50) | Parser becomes multi-year project | Use STEPcode library, only write mapping layer |
| Pattern performance (Phase 45) | 1000-hole patterns take minutes | Geometry Pattern as default (batch B-Rep copy), Feature Pattern opt-in only |
| Assembly solver singularity (Phase 42) | Over-constrained assemblies crash solver | Kinematic graph pre-analysis + rigid sub-group detection |
| FEA mesh quality (Phase 57) | Dirty B-Rep produces garbage FEA results | Defeaturing/healing pre-step before TetGen |
| GPU tessellation portability (Phase 66) | MoltenVK hardware tessellation fails on macOS | Compute shaders only, no tessellation pipeline stages |
| CAM toolpath robustness (Phase 71) | Offset/gouge math breaks on complex contours | Integrate OpenCAMLib, don't write from scratch |
| Live collaboration conflicts (Phase 70) | OT/CRDT on feature tree causes topology mismatch | Feature-level pessimistic token locking, no concurrent tree mutation |

---

## 10. Success Criteria

### Phase 30 (Era 0 complete)
- [ ] All existing 2D tools work with R*-tree spatial indexing
- [ ] Constraints solve and undo correctly in the editing workflow
- [ ] Expressions drive dimensions parametrically
- [ ] Sketches live on arbitrary 3D planes
- [ ] CI passes on Windows + Linux
- [ ] 10,000-entity drawings load and interact smoothly

### Phase 40 (Era 1 complete)
- [ ] Users can sketch → extrude → Boolean → fillet to create a part
- [ ] Feature tree edits propagate correctly (TNP resolution works)
- [ ] 50+ kernel integration tests pass
- [ ] .hcad v14 round-trips B-Rep topology

### Phase 52 (Era 2 complete)
- [ ] Multi-part assemblies with mates solve in < 1s
- [ ] STEP AP242 imports/exports B-Rep geometry
- [ ] Python scripts can create complete parts
- [ ] 100-part assemblies run in < 2 GB RAM

### Phase 64 (Era 3 complete)
- [ ] 2D drawings auto-generated from 3D models
- [ ] FEA results match textbook solutions within 5%
- [ ] PDM tracks revisions with semantic diffs
- [ ] Sheet metal flat patterns export to DXF

### Phase 80 (1.0 Release)
- [ ] 10,000-part assemblies at 60 FPS
- [ ] Photorealistic renders via path tracing
- [ ] Cloud sync with live collaboration
- [ ] Plugin marketplace operational
- [ ] Published benchmarks competitive with SolidWorks on standard test models
