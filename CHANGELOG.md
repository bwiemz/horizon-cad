# Changelog

All notable changes to Horizon CAD are recorded here. The project was built
phase-by-phase against the roadmap in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md);
this file summarizes that work by era. Each phase shipped as an honest core
slice — where the roadmap named a heavy third-party dependency, an in-house
implementation was built instead to keep CI lean and the code testable
headless. Those deviations (STEPcode/OCCT, Embree, OpenCAMLib) are documented
in [the era findings note](docs/superpowers/notes/2026-07-03-era2-roadmap-findings.md).

## Unreleased — Kernel hardening (post-1.0 review response)

Response to the external senior review: fix the Boolean/kernel reality gap
first, then the architecture boundary, interop pinning, and CI enforcement.

- **Boolean rework (the headline).** `BooleanOp` is now a BSP-tree CSG
  pipeline: the trimmed boundary is extracted from face loops
  (`BoundaryMesh`, with ear-clip triangulation of non-convex faces and
  global outward orientation), faces are split along the other solid's face
  planes with exact fragment classification and coplanar-face resolution
  (`MeshCsg`), and selected fragments are welded, T-junction-freed, and sewn
  into a manifold half-edge solid (`SolidSewer`).  Results carry TopologyID
  provenance, chain into further Booleans, and round-trip through STEP.
  Volumes are closed-form-exact for planar-faced solids (unions, subtracts,
  intersects, cavities, through-holes, multi-shell operands).  Curved faces
  participate as their loop polyhedra — analytic surface–surface
  intersection remains future work.
- **Faithful boundary evaluation everywhere.**
  `ExactPredicates::tessellateSolid` and `SolidTessellator` no longer
  tessellate the over-covering bounding-rectangle surface patches for planar
  faces (or re-emit shared curved surfaces once per face) — classification,
  display, interference, and export now agree with the actual solid.
- **CSG-exact interference.** `InterferenceChecker`'s narrow phase is now
  "does the Boolean intersection enclose volume", replacing edge-crossing
  heuristics that missed exactly-grazing symmetric configurations; surface
  touching no longer counts as interference, and cavities are respected.
- **Modeling kernel decoupled from render.** The mesh POD moved to
  `hz::geo::MeshData`; `render::MeshData` is an alias.  `hz_modeling` and
  `hz_document` no longer link `Horizon::Render`.
- **STEP interop pinned by fixtures.** Hand-authored third-party-style
  Part-21 fixtures (FreeCAD/OCC and SolidWorks formatting, analytic
  PLANE/LINE geometry, assembly product structure, `BREP_WITH_VOIDS`) with a
  drop-in directory contract for real vendor exports; documented
  limitations are enforced as tests, and restyled-reimport tests pin parser
  robustness (comments, reflow, entity reordering).
- **Adversarial model suite.** Hole patterns against multi-shell operands,
  nested pockets, non-convex bracket extrudes, shelled containers, Boolean
  volume-conservation fuzzing, and long feature chains — all with
  closed-form expected volumes.  The suite exposed and pins a real
  `ChamferOp` defect (combinatorially valid topology with geometrically
  inconsistent loops; volume integrator undercounts) for a future rebuild on
  `SolidSewer`.
- **clang-tidy is now a gate.** CI fails on any `bugprone-*`/`performance-*`
  finding outside a 10-check known-dirty backlog (which stays visible as
  advisory warnings).
- **Honest maturity documentation.** README gained a per-module Feature
  Maturity table (stable / experimental / prototype) and no longer implies
  1.0 production readiness.

Adversarial-review follow-ups (self-verified from code after the review's
automated verify pass was cut short):

- Fixed a self-inflicted regression: `ExactPredicates::tessellateSolid` (the
  `DrawingProjection` hidden-line occluder) had been switched to pure
  loop triangulation, which collapses curved solids — a torus's eight ring
  corners are coplanar — to a flat, zero-volume mesh.  It now delegates to
  `SolidTessellator`, keeping smooth surface tessellation for curved faces
  and restoring the `tessTol` control.
- The `checkManifold()` output contract now applies on **every** `BooleanOp`
  path, including the disjoint fast paths (previously only the CSG path was
  gated).
- Tolerance stack made consistent: the CSG on-plane epsilon is exported as
  `kCsgPlaneEps`, and `BooleanOp` welds CSG fragments at that tolerance so
  seams the splitter is allowed to open are always reconcilable during
  sewing.
- `SolidSewer`'s degenerate-face area test is now computed relative to the
  loop's first vertex, so a far-from-origin loop's true zero area no longer
  drowns in `R²` cancellation noise.
- Documented (in headers) the remaining known limitations the review
  surfaced: unbalanced BSP recursion depth, discarded face inner-loops on
  inputs, greedy twin pairing at non-manifold edges, and Booleans against
  coarse-box-topology curved primitives (torus/revolve).

## 1.0.0 — Production readiness

All 80 roadmap phases (plus the 61b sheet-metal insert) delivered. ~900
automated regression tests pass across the Windows, Ubuntu, and
AddressSanitizer CI gates, with clang-format and clang-tidy checks. GPU paths
were verified on an RTX 5070 Ti via headless Vulkan.

### Era 0 — Foundation (Phases 1–30)

2D drafting application: math library, OpenGL 3.3 renderer, camera/grid/Qt
shell; document model with undo/redo, selection, snapping, native `.hcad` JSON
format and DXF I/O. Full drawing toolset (line, arc, circle, rectangle,
polyline, ellipse, spline, text, hatch), editing tools (offset, trim, fillet,
chamfer, break, extend, stretch, mirror, rotate, scale, arrays), dimensions and
annotations, a Newton–Raphson/LM constraint solver, blocks/components, layers
with ByLayer inheritance, line types via GPU shader, box selection, grouping,
UI modernization (dark theme, ribbon), an R\*-tree spatial index, parametric
sketch solving, an expression engine, and Linux CI.

### Era 1 — Geometry kernel (Phases 31–40)

NURBS curves and surfaces with adaptive tessellation; a half-edge B-Rep with
TopologyID genealogy and Euler operators; primitives; extrude and revolve;
Boolean operations; fillet/chamfer; feature-tree UI; viewport polish; kernel
hardening.

### Era 2 — Assemblies, interop & scripting (Phases 41–52)

Multi-document architecture with FlatBuffers `.hzpart`/`.hzasm` formats and
lightweight/resolved assembly loading; assembly mates (8 types, 6-DOF solver);
loft and sweep; shell and draft; linear/circular patterns; reference geometry;
Python scripting (embedded CPython via pybind11); collision detection;
measurement and mass properties (Eberly integrals); STEP AP242 import/export
(in-house ISO 10303-21); native binary format with zero-copy tessellation
cache; stabilization (sparse assembly solve, Boolean robustness, memory
guards).

### Era 3 — Professional workflow (Phases 53–64)

2D drawing generation (hidden-line projection, standard/section/detail views,
`.hzdwg`); GD&T feature control frames; BOM and balloons; sheets and title
blocks; in-house FEA (linear static and steady-state thermal); PDM local
version control and multi-user vault locking; advanced fillets (variable-radius
+ spherical corner blends) and drawing section views (61); sheet-metal core
(bend allowance/K-factor/flat pattern, 62) and 3D flange bodies (61b); Python
API phase 2; end-to-end stabilization.

### Era 4 — Cloud, rendering & market parity (Phases 65–80)

Rendering abstraction layer (`RenderBackend`) with OpenGL and staged Vulkan
backends; GPU compute NURBS tessellation (SPIR-V, verified GPU≡CPU); PBR
material library with IBL-lite ambient; an in-house CPU Monte Carlo path tracer;
local-first cloud sync of vault revisions; live-collaboration sessions with
feature-level token locking; CAM (2.5-axis toolpaths + G-code); kinematics
(forward + CCD inverse); advanced simulation (modal + stress-life fatigue);
configuration management (design tables); surfacing (Coons patches, 75); glTF
2.0 GLB export (76); localization infrastructure with starter catalogs (77);
large-assembly instancing + frustum culling (78); a zero-code-execution plugin
registry with a fail-closed permission model (79); and 1.0 release prep (80).

### Notable deviations from the roadmap (in-house instead of a dependency)

- **STEP** — in-house ISO 10303-21 Part-21 reader/writer rather than
  STEPcode/OCCT.
- **Ray tracing** — in-house Monte Carlo path tracer rather than Embree.
- **CAM** — closed-form 2.5-axis toolpaths rather than OpenCAMLib (general
  free-form pocketing/waterline staged behind that integration).

### Deferred beyond the 1.0 code kernel

Phase 73's CFD (deferred by the roadmap itself), and the productization tail of
Phase 80 — signed installers (MSI/AppImage/DMG), the hosted plugin marketplace,
and published SolidWorks/FreeCAD benchmarks — are future work, not code slices.
