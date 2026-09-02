# Changelog

All notable changes to Horizon CAD are recorded here. The project was built
phase-by-phase against the roadmap in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md);
this file summarizes that work by era. Each phase shipped as an honest core
slice — where the roadmap named a heavy third-party dependency, an in-house
implementation was built instead to keep CI lean and the code testable
headless. Those deviations (STEPcode/OCCT, Embree, OpenCAMLib) are documented
in [the era findings note](docs/superpowers/notes/2026-07-03-era2-roadmap-findings.md).

## Unreleased — Geometric validation, faceted geometry, working blends (Phases 81–87)

Post-1.0 kernel work, continuing from the review response below. Where kernel
hardening fixed what the Booleans *did*, this pass fixes what the kernel could
not *see*.

- **Geometric B-Rep validation (81).** `Solid::isValid()` walks twin/next/prev
  linkage and counts entities — it never reads a coordinate, so a solid can
  pass every structural check while its loops are self-intersecting,
  non-planar, or spanning positions its twin half-edges disagree about. The
  new `hz::topo::GeometryValidator` checks vertex-chain consistency
  (`he->next->origin == he->twin->origin`), twin positional coincidence,
  degenerate edges and faces, loop planarity against the face's own carrier
  (curved carriers are skipped rather than guessed at), loop
  self-intersection, and closed-shell area-vector balance — with advisory,
  non-failing reports for edge curves that miss their vertices and for
  distinct vertices at the same position.
- **Sharp cones were degenerate (81).** `makeCone` built box topology from two
  4-point rings; with `topRadius = 0` that ring collapses to a point, giving
  four zero-length edges around a zero-area cap. The Cone command asks for
  exactly that shape, so every cone inserted from the UI passed Euler and
  manifold while being geometric nonsense. Sharp cones now build apex topology
  (5V/8E/5F) with the analytic conical carrier; a cone with both radii or zero
  height is refused rather than returning a degenerate solid.
- **ChamferOp rebuilt on `SolidSewer` (82).** This closes the defect pinned in
  `test_AdversarialModels`: a 20mm box with two 2mm edge chamfers integrated
  to 3200 instead of 7920. Two things were wrong — loop construction pushed
  the original vertex back into the loop at corners the chamfer had already
  replaced, and reconstruction replayed the soup through Euler operators with
  a convergence loop that terminated on valid linkage that was not the linkage
  the polygons described. Loops are now vertex-driven and the soup is sewn by
  the same pipeline `BooleanOp` uses, so original faces keep their carriers
  and TopologyIDs. FilletOp needed no rebuild — its Phase-61 strict half-edge
  assembly already produced consistent loops — and both ops now refuse
  geometrically invalid output instead of returning it.
- **Chamfer vertex blends (83).** Two or three selected edges meeting at a
  corner used to be refused outright. The material a chamfer removes is the
  half-space beyond its chamfer plane, so a corner is just that corner clipped
  by the planes of every chamfer meeting there — blends fall out of the
  single-chamfer code path, with no invented corner patch, which is the
  standard planar-chamfer result. Validated against inclusion–exclusion
  volumes of the union of cutting prisms; chamfering all twelve edges of a
  cube yields the chamfered cube (18 faces, 32 vertices) at its exact volume.

Remaining known limits in this area are documented in the headers: chamfers
are for straight edges of planar-faced solids with orthogonal, convex corners
(oblique corners are refused by the geometric gate, not silently mis-built),
and inner face loops are not carried through the chamfer rewrite.

### Faceted curved primitives (84)

The kernel evaluates a solid from its face loops everywhere that matters —
Boolean classification, interference, mass properties, drawing projection,
export. The curved primitives leaned on that being invisible: `makeCylinder`
built box topology (8V/12E/6F) and bound a cylindrical NURBS patch to its four
lateral faces, so the solid *was* a square prism to every computation and only
the renderer disagreed. A cylinder's volume came out 500 against π·r²·h = 785,
a sphere's 192 against 524, and a torus — genus 0 with eight vertices —
enclosed no volume at all. Subtracting one cylinder from another gave 320
where the answer is 503.

- **Curved primitives are now faceted at construction**, built on `SolidSewer`
  from a polygon soup with a tunable `segments` count;
  `PrimitiveFactory::segmentsForTolerance()` inverts a chord-sag budget when
  you want to pick it from a tolerance instead. Facets carry planar patches
  that match their loops, so the B-Rep is exactly what the rest of the kernel
  treats it as. Volumes converge from below — at the default 32 segments a
  cylinder is within 0.65% and at 128 within 0.04% — and Booleans on curved
  solids work: boring a cylinder yields a real tube, correct to the facet
  error rather than 36% light. The torus is now a genuine genus-1 shell.
- **Facets remember what they approximate.** `topo::Face::analyticSurface` and
  `topo::Edge::analyticCurve` record the ideal geometry a facet stands in for,
  distinct from the carrier that actually bounds it. That is what keeps a
  cylindrical mate frame and a radial dimension resolvable from a single pick
  on one facet; binding the cylinder to a planar quad instead would put
  display and computation straight back out of step.
- **A rendering defect fell out of the same diagnosis.** `SolidTessellator`
  emits a face's whole *untrimmed* carrier whenever that carrier is curved, so
  faces sharing a surface each re-emitted all of it: a sphere drew six
  overlapping spheres at 480,000 triangles, and the resulting mesh enclosed
  three times the solid's volume. Faceted primitives take the loop path, so
  each face is emitted once — 960 triangles for that sphere, 124 for a
  cylinder — and the display mesh and the mass-properties integrator now
  agree. Both bounds are pinned by tests.

Two consequences are deliberate and pinned rather than papered over. A torus
is genus 1, so `V - E + F` is 0 and `Solid::checkEulerFormula()` — which has
no genus term — rejects it; `checkManifold()` and the geometric validator both
pass, and the tests assert exactly that. And a faceted cylinder exports to
STEP as planar B-spline faces rather than a rational cylindrical surface: the
written file is exactly the model in memory. Emitting analytic faces would
mean un-faceting on export, which is a separate feature, not a property of the
round trip.

### Blends that work on faceted geometry (85–86)

Faceting the primitives exposed that neither blend operation could act on
them, and checking why turned up defects rather than missing features.

- **Chamfer capacity measured the wrong thing.** It capped the distance at
  half the shortest edge of the adjacent face — a proxy that holds for a box
  and means nothing once faces are faceted. On a 32-sided cylinder the
  shortest edge is the facet chord, so every chamfer over 0.49 was refused
  where the geometry is exact past 4.9; it also rejected a 6mm chamfer on a
  10mm cube that builds correctly. Capacity is now how far the face reaches
  along the offset direction — a necessary condition that never rejects a
  distance that would have worked, with the geometric gate doing the real
  guaranteeing. A cylinder rim now chamfers to the exact truncated cone it
  removes, at any distance up to the cap's inradius: ten times the old range,
  bounded by geometry rather than a heuristic.
- **Vertex identity was inconsistent across the pipeline.** The clip
  deduplicated at 1e-9 × scale, the sewer welds at an absolute 1e-7, and the
  geometric validator judges a loop degenerate relative to its own extent. A
  1.075e-7 segment therefore sewed as legal and validated as degenerate, which
  is what the "self-intersecting loops" refusals actually were. Clipping now
  deduplicates against the loop's extent so all three agree; intersections are
  snapped to the endpoint they land on, since a cut through an existing vertex
  recomputes it with cancellation; and a segment nearly parallel to the clip
  plane no longer has an intersection computed at all, the denominator there
  being the difference of two nearly equal distances. `SolidSewer`'s weld
  tolerance is a named constant so the two cannot drift apart again.
- **A fillet's boundary was the chord, not the arc.** The blend was one flat
  quad joining the two tangent lines, carrying the correct rational-quadratic
  arc surface — the Phase 84 disease in a second place. Every loop-based path
  integrated a chamfer while the renderer drew a fillet: `(1/2)r²L` removed
  instead of `r²(1 − π/4)L`, 2.33 times too much, at every radius. Blends are
  faceted across the arc now, sampled by spherical interpolation between the
  two tangent radii so the points are exact on the arc, with the arc patch
  kept on `Face::analyticSurface`. Corner blends follow the same samples, so
  the spherical patch matches the arcs it joins instead of spanning them
  flat. Volume converges quadratically — inside 0.001% at 16 chords — and
  `analyticSurface` now survives a later operation, which it did not before.
- **Revolve enclosed zero volume (87).** The builder handled exactly one
  input: a four-vertex profile turned a full 360°. For it, it rotated the four
  profile corners to 0° and 180° and built an eight-vertex box from the two
  quads — which are mirror images of each other through the axis, so the box
  is inside-out against itself and encloses nothing. It passed every check the
  suite made (Euler, manifold, `isValid()`, a NURBS surface on all six faces),
  because no test asked for a volume. A torus surface was pasted on all six
  faces, including the two that were the profile itself. Every other input
  returned `nullptr`: any partial angle, any profile that was not a
  quadrilateral. The UI's Revolve command offers 1–360°, so 359 of its 360
  settings silently produced nothing.

  Revolve is now a swept ring stack sewn by `SolidSewer`, the same pipeline as
  the Phase 84 primitives: any closed profile, any angle in (0, 2π], angular
  resolution tunable per call and derivable from a chord-sag budget with
  `Revolve::segmentsForTolerance()`. Partial turns are capped at both ends
  (genus 0); a full turn clear of the axis closes on itself as a genus-1
  torus, which is manifold but which the genus-free Euler check rejects — the
  same documented caveat as `makeTorus`. Profile vertices sitting *on* the
  axis do not move, so their bands collapse to triangles and a profile
  touching the axis sweeps a proper cone. A profile crossing the axis, or one
  whose plane the axis does not lie in, is refused rather than swept through
  itself — one test on the radial vectors catches both.

  Volume converges quadratically to the Pappus value: −9.97% at 8 steps,
  −2.55% at 16, −0.64% at 32, −0.16% at 64, −0.04% at 128, and a partial turn
  carries the same relative error as a full one at equal angular resolution.
  Every band is *exactly* planar — rotating two points about a shared axis
  leaves all four corners on the plane whose normal combines the angular
  bisector with the axis — so unlike the fillet blends, no face here needs an
  approximate carrier. Only the ideals go on the side: the cylinder or cone
  each curved band approximates on `Face::analyticSurface`, the circle each
  profile vertex traces on `Edge::analyticCurve`. A band sweeping a flat
  annulus records neither, because its planar carrier is already exact.

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
