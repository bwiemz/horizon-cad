# An exact B-rep kernel — design

Horizon CAD's solids are faceted today: every curved face is a set of flat
polygons, with the true surface carried beside them as a label. This design
replaces that with an exact boundary representation, built in-house, in
stages, each of which ships.

## Intent

To compete with SolidWorks, a part's geometry must be the geometry the user
designed, not an approximation of it that every operation then works on. The
faceted kernel shows its limits where SolidWorks users push hardest:

- complex fillet intersections, and fillets on curved faces;
- tangential intersections of curved surfaces;
- small features on large models, and thin walls;
- STEP round trips at high precision;
- Boolean time as resolution rises (a 2,048-facet cylinder takes seconds);
- chains of operations on curved faces, each adding error.

The owner has chosen to build the exact kernel in-house rather than adopt
OpenCASCADE, and accepts that it is long work. Done well, it is the part of
Horizon that matters most and that no one else's code decides.

## What exists today

Mapped from the code (November 2026):

- **Geometry.** Every surface is a `geo::NurbsSurface` and every 3D curve a
  `geo::NurbsCurve`; planes, cylinders, cones, spheres and tori are NURBS made
  by factories, and nothing records which kind a surface is. NURBS curves are
  fairly complete (knot insertion, segment, degree elevation); NURBS surfaces
  evaluate with first derivatives and invert points, but have no knot
  insertion, splitting, second derivatives or trimming.
- **Topology.** A half-edge B-rep (`topo::Solid`, `Face`, `Edge`, `Vertex`,
  `HalfEdge`, `Wire`), soft-deleted, with Euler operators. A face's boundary
  is its 3D loop; `Face::surface` is a planar patch for kernel-made faces, and
  `Face::analyticSurface` the ideal surface, shared by the facets of one
  cylinder. There are no trimming curves in a surface's parameters (pcurves)
  and no per-entity tolerance.
- **Operations.** Primitives, extrude, revolve (and so holes), loft and sweep
  build facets at construction (32 per turn by default; the count is a saved
  parameter). Booleans triangulate the loops and run a BSP CSG in doubles,
  then sew; fragments keep their ideal surface, new intersection edges lose
  theirs. Fillets and chamfers work between flat faces only, with an exact
  blend surface kept as the label of its facets. Shell and draft work on
  loops.
- **Consumers.** Rendering, mass properties, interference, drawings (hidden
  lines) and Booleans read faces through one function,
  `BoundaryMesh::extractFacePolygons`. Mates, silhouettes, STEP export and
  `IdealMassProperties` also read the ideal labels.
- **Robustness.** Tolerances are global constants (`math/Tolerance.h`).
  `ExactPredicates` is tolerance arithmetic, not exact predicates.
  `SurfaceSurfaceIntersection` intersects triangles and nothing uses it.
- **STEP import** reads exact faces and then facets them on purpose
  (`Faceting::facetCurved`), because nothing downstream could use them.
- **Naming.** Names are genealogy strings; facets are `<face>/facet:<k>`,
  chords `<edge>/chord:<k>`, Boolean pieces `/piece:<k>`, and
  `logicalFace`/`logicalEdge` strip those suffixes. Three naming schemes are
  kept for old files. Native files are format 31.

## Goals

1. **Faces are exact.** A face is a region of an exact surface, bounded by
   exact edges; a cylinder is three faces (side, two caps), not 34.
2. **Exact intersections.** Surface/surface, curve/surface and curve/curve
   intersection: closed form for the analytic pairs, robust marching for
   NURBS, including tangent and overlapping cases.
3. **Exact Booleans.** Union, subtract and intersect of exact solids, giving
   exact solids, replacing the BSP CSG.
4. **Exact blends.** Constant and variable radius fillets and chamfers between
   any faces the kernel supports, vertex blends where three or more meet.
5. **Every existing feature keeps working at the end**: extrude, revolve,
   hole, loft, sweep, patterns, mirror, shell, draft, sheet metal, assemblies,
   drawings, STEP, mates, the AI tools. During the stages an exact operation
   may fail on a model master builds (see the owner's decisions below), but
   never without saying why.
6. **Exact numbers.** Volumes, areas and centres of mass to 1e-9 relative
   for solids bounded by closed-form geometry, not from below as facets give
   them. Where a boundary is a fitted intersection curve, mass properties
   integrate against it refined on demand to that accuracy (an
   `IntersectionCurve` keeps its two surfaces so it can be). An imported solid
   is exact to the tolerance its file gives its edges, and the report says
   which. *(Builder check; my call.)*
7. **Exact exchange.** STEP import keeps faces exact; export writes them
   exactly. A round trip keeps each surface's shape within the modelling
   resolution and its kind as the file gave it: a `CYLINDRICAL_SURFACE` comes
   back as one, and a `B_SPLINE_SURFACE` as the same B-spline, even when it is
   a quadric. Recognising a NURBS as a typed surface applies to surfaces
   Horizon makes, not to imported B-splines. *(Builder check; my call.)*
8. **One kernel at the end.** The BSP CSG, the facet-at-construction code and
   the ideal labels are deleted once nothing needs them.

## Non-goals

- Adopting OpenCASCADE or any other kernel as a dependency. (OpenCASCADE stays
  a test oracle in CI, as it is today for STEP.)
- Exact (rational or multi-precision) arithmetic throughout. Geometry is
  doubles with a tolerance model; only topological decisions use robust
  predicates.
- Faster rendering, GPU tessellation, or new modelling features beyond what
  the exact kernel needs. (They may follow.)
- Changing sketches or the 2D drafting entities.

## Anti-goals

These would make the work a failure even if it compiled and ran:

- **Fallbacks that hide failure.** An exact operation that fails says why; it
  never quietly runs the faceted code instead.
- **Failures nobody tracks.** An operation switched to exact may fail on
  models the faceted one built (owner's decision 1, below); each such model
  in the regression set is listed when its stage switches over, with why it
  fails, and worked down. A failure outside the list is a bug.
- **Silent changes to saved parts.** An old part opened after a change says
  what changed (references that could not be carried over), never drops them
  quietly.
- **Unbounded slowdowns.** An exact operation may be slower than its faceted
  counterpart at 32 facets per turn, but not by more than the budgets below,
  and it must be faster at the accuracy the user asked for.

## Constraints

- C++20, in-house, under GPL-3.0-or-later; no new third-party geometry library.
- Every CI gate stays green on every PR: GCC 11, MSVC and Apple clang builds,
  AddressSanitizer, ThreadSanitizer, clang-tidy-15, clang-format-15, coverage,
  fuzzing, the OpenCASCADE STEP interop job.
- Existing tests keep passing, or are re-baselined on purpose: about a hundred
  assert facet counts (n + 2 faces for a cylinder) or volumes from below, and
  change when faces stop being facets. Each such change is named in its PR.
- Old native files (formats 1 to 31) and their three naming schemes still
  open (see Migration). Files saved while the stages land are best effort
  (owner's decision 2, below).
- Kernel code runs on workers (rebuilds, imports): it stays free of global
  mutable state.

## Approaches considered

1. **Replace in place, a layer at a time (chosen by the owner).** Exact
   representation first, then each operation converted, with two bridges that
   keep everything working meanwhile. Every stage ships something visible.
2. **A separate exact kernel, built to completion, then switched to.** More
   freedom in its design, but nothing visible for a long time and one large,
   risky switch at the end.
3. **Exact operations falling back to faceted ones on failure.** Safer at
   first, but two kernels forever, and the fallback hides exactly the failures
   that must be fixed.

**Old parts (chosen by the owner): one kernel.** Parts made by the faceted
kernel are rebuilt exactly when opened; their facet-level names map to whole
faces; what cannot be mapped is reported. Volumes move toward their true
values. Horizon is unreleased (0.1.0), so few such files exist.

**When an operation goes exact (owner's decision 1, after the builder check):
at its stage, for everyone.** Each converted operation replaces the faceted one
when its stage lands, not behind an opt-in. A model it cannot handle fails
with the reason, never by falling back to the faceted code. The regression set
(every test, the samples, the STEP interop corpus, the adversarial models, the
beta tasks) is run at each switch, and every model that built on master and
fails now is listed, with why, in the stage's PR and worked down after.

**Files saved during the stages (owner's decision 2): best effort.** Only parts
made by the final kernel, and those from before the change (format 31 and
older), are promised to open as they were. A part saved mid-way may lose
references when a later stage changes what its operations produce; they are
reported by the same "Not Everything Was Read" mechanism, but no per-stage
format versions or mappers are kept for them.

## The design

### Geometry

- **Typed surfaces.** A `geo::Surface` interface with `Plane`, `Cylinder`,
  `Cone`, `Sphere`, `Torus` (closed-form evaluation, derivatives to second
  order, point inversion, parameter ranges and periodicity) and `NurbsSurface`
  (today's class, extended with knot insertion, splitting, second
  derivatives). Later, `OffsetSurface` and swept/extruded surfaces as fillets
  and sweeps need them. A NURBS that is exactly a quadric (as STEP files and
  today's factories make them) is recognised and kept as its typed surface.
- **Typed curves.** `Line`, `Circle`, `Ellipse` and `NurbsCurve` in 3D;
  2D curves for pcurves (`Line2`, `Circle2`, `Nurbs2`). An `IntersectionCurve`
  holds the two surfaces it lies on and a fitted NURBS within tolerance of
  both, so it can be refined later.
- **Intersection library** (`geo::intersect`): surface/surface,
  curve/surface, curve/curve, each returning points, curves and overlap
  regions with their tolerances. Closed forms for the analytic pairs
  (plane with anything; coaxial and parallel quadrics); marching from seeded
  start points for the rest, with step control by curvature, singular and
  tangent points found and walked through, and loops closed.

### Topology

- The half-edge structure stays. A `Face` gets its surface as the carrier
  (no separate ideal), a sense (normal with or against the surface), and a
  pcurve for each of its edge uses. An `Edge` gets its 3D curve and a
  tolerance; a `Vertex` gets a tolerance. Tolerances grow only where an
  operation needs them to (tolerant modelling), and the validator checks every
  edge lies on its faces within them.
- `topo::Solid::isValid` keeps its combinatorial checks; a new exact
  validator checks geometry: pcurves agree with 3D curves, loops are closed
  and oriented in parameter space, faces do not self-intersect.

### Predicates and tolerances

- A tolerance model in one place: the modelling resolution (1e-7 of the part's
  size, scaled as `CsgTolerance` scales today), angular resolution, and how
  tolerances combine when entities meet.
- Robust orientation predicates (adaptive-precision, Shewchuk style) where a
  topological decision depends on a sign: point in 2D loop, ordering along a
  curve, side of a plane.

### The two bridges

- **Faceted copies for unconverted operations.** `Faceting::facetCurved`,
  generalised to any exact solid at a chord tolerance, gives an operation not
  yet converted the faceted solid it understands, labelled as today. Removed
  when the last such operation is converted.
- **Meshing at the read point.** `BoundaryMesh::extractFacePolygons` meshes an
  exact face (its trimmed surface, to the display or analysis tolerance) and
  hands consumers polygons as today, with each polygon's face. Rendering,
  interference, drawings and the CSG keep working unchanged. Mass properties
  move to exact integration over trimmed surfaces as soon as faces are exact.

### Naming

- A face keeps the name its feature gives it (`cylinder/side`, an extrude's
  `side:<entity>`), with no facet suffix: one face, one name.
- Faces split by a Boolean are named as pieces of their source, as now, but
  pieces of one face that end up connected are one face again.
- Edges made by intersecting two faces are named from those faces
  (`edge:<a>|<b>`), as `nameEdgesLogically` names them today, so a rim keeps
  its name through a rebuild.

### Migration

- Native format 32 records that a part is exact. A part of format 31 or older
  is rebuilt exactly on open. (One migration, from the faceted kernel; parts
  saved between stages get none, per decision 2.) References kept in
  assemblies and drawings are re-found by the same logical names when their
  part is rebuilt, and those that are not found are reported, as they are
  today. Each reference stored in it (sketch on a face,
  mates, dimensions, GD&T, hole callouts) is carried to the exact face or
  edge by `logicalFace`/`logicalEdge`, which already strip facet and chord
  suffixes. A reference that maps to nothing is listed in the "Not Everything
  Was Read" report, and the part is not saved over its old file without
  asking (the read-in-part rule).
- A feature's `segments` and `chordTolerance` parameters stop shaping the
  solid. They become its display tolerance, kept so old files read and so a
  user can still ask for a coarse display. A count low enough to look like
  intent (under 12 segments per turn, or a fillet of fewer than 4 arc
  segments) is rounded off too, and each such feature is listed in the
  report: "drawn as a 6-sided prism before; round now". *(Builder check; my
  call: few such files exist, and a polygon the user means is drawn as a
  polygon in the sketch.)*
- References stored under the two older naming schemes (Positional:
  `lateral_<i>`, `edge<N>`; FromGeometry: `side:<e>.f<k>`) cannot be mapped
  reliably: an exact rebuild has fewer faces, so `lateral_5` could name a
  different, real face. Every reference in such a feature is reported as not
  carried over, never guessed. *(Builder check; my call: a wrong face silently
  chosen is worse than one reported, and these schemes predate the Stable one
  every current feature uses.)*

### Stages

Each stage ends with master green, nothing worse than before, and something
a user can see. Each is several PRs.

1. **Foundations.** Typed surfaces and curves, pcurves, the tolerance model,
   robust predicates, the exact validator, meshing of trimmed faces at the read
   point. Nothing in the app changes yet; everything is tested on its own.
   *Landed:* typed analytic and NURBS surfaces and curves, pcurves, one
   tolerance model, exact predicates, the exact validator, Pattern copies that
   keep exact geometry, constrained Delaunay triangulation, and watertight
   meshing of trimmed faces at `BoundaryMesh`'s read point; nothing reads exact
   faces yet.

   Carried to later stages:
   - *Stage 2, performance:* cones and tori mesh heavily at the read point (a
     frustum 9.5k triangles, ~235 ms; a one-vertex torus 21k); refine in the live
     triangulation, grade by local radius, cache each face's mesh.
   - *Stage 2, importer:* seamless periodic faces (a band between two rims) are
     `LoopNotClosed` under the one-seam convention; the importer inserts seams.
   - *Stage 2, callers:* `BooleanOp`, `InterferenceChecker` and `SolidTessellator`
     catch `extractFacePolygons`' `std::runtime_error`; `SolidTessellator` tags one
     face per `topoId`.
   - *Stage 2, mass:* `MassProperties` surfaces the mesher's reason when props are invalid.
   - *Stage 2, validator:* a periodic edge over one period and an unwrapped
     loop's extent are unbounded; imported data is the first source.
   - *Stage 2, `triangulateRegion`:* a lower spread guard (~1e-60) and a fuzzer, as STEP pcurves are adversarial.
   - *Stage 2, transforms:* a non-similar transform (Ruling 9) converts the copy
     to NURBS instead of emptying its exact fields.
   - *Stage 2, STEP:* the writer folds Plane/Cylinder/Cone/Line scale factors (Ruling 5) into the parametrisation.
   - *Stage 2, mixed solids:* test or orient the faceted subset of an exact plus faceted solid.
   - *Stage 2, tolerance:* `ModelTolerance::forExtent(DBL_MAX)` gives an infinite tolerance.
   - *Stage 3:* pole and apex topology (degenerate edges or not, a loop's jump
     round a pole, `sameUv`'s arc-vs-chord there, a recognised sphere's frame z);
     exact primitives name their edges and vertices.
   - *Stage 4:* closed-form intersections read the surfaces' scale and axialScale.
   - *Stage 5:* the validator's O(n²) self-intersection and hole checks, a crossing
     at a sample point, near-concentric loops, and a global orientation check.
   - *Stage 6:* `Torus::invert` assumes R >= r; fillets can make spindle tori.
2. **Exact exchange.** STEP import keeps exact faces (no `facetCurved`);
   operations reach them through the faceted bridge; STEP export writes exact
   faces as they are; rendering and mass properties use the exact faces.
   *Seen:* an imported part's mass is exact, and a STEP round trip changes no
   surface.
3. **Exact construction.** Primitives, extrude, revolve (and with it holes,
   sheet metal's bends) build exact faces; migration of old parts; naming
   without facets. *Seen:* exact volumes, a cylinder of three faces, smooth
   silhouettes in drawings at any zoom. (A part with a Boolean is still
   faceted after it until stage 5.)
4. **Exact intersection.** The intersection library, analytic pairs first,
   then NURBS marching with tangencies and overlaps; property tests and
   fuzzing. Library only.
5. **Exact Booleans.** Face/face intersection into edges on both faces,
   splitting faces in parameter space, classifying pieces in, out or on, and
   sewing within tolerance; coincident faces (co-planar, co-cylindrical)
   merged. Replaces the BSP CSG for exact operands, then everywhere; holes,
   patterns, combine bodies and mirror become exact. *Seen:* clean holes in
   curved parts, Booleans that do not slow with accuracy.
6. **Exact blends.** Constant radius fillets between any two supported faces
   (rolling ball: cylinders and tori for analytic pairs, NURBS within
   tolerance otherwise), chamfers, vertex blends, variable radius; then shell,
   draft and offset on exact faces. *Seen:* fillets on curved edges and where
   fillets meet. The stage is bounded by a catalogue of blend cases written at
   its start (plane/plane, plane/cylinder, cylinder/cylinder, convex and
   concave, tangent chains, three- and four-edge vertex blends, a fillet
   meeting a fillet), each with a test; what is outside it is a later stage.
   *(Builder check; my call.)*
7. **Freeform and clean-up.** Loft and sweep make NURBS surfaces exactly
   (skinning, sweeping along curves); the faceted bridge, the BSP CSG and the
   ideal labels are deleted.

### Testing

- **Oracles, not snapshots.** Results are checked against what is known:
  closed-form volumes and areas, points lying on both surfaces of an
  intersection within tolerance, Euler characteristic, and the faceted
  kernel's answer converging to the exact one as facets grow.
- **OpenCASCADE as an independent check** in CI, extending the STEP interop
  job: the same Booleans and fillets compared by volume, area and topology.
  It is a test tool, never shipped.
- **Property tests and fuzzing** for the intersection library and Booleans
  (random placements of primitives, near-tangent and coincident cases), with
  every failure kept in a corpus, as the file readers' are.
- **Adversarial models** (`test_AdversarialModels`) and the existing modelling
  suites, re-baselined where facet counts or volumes from below were asserted.

### Performance budgets

Measured in Release on CI's Linux runner, recorded in the PR that sets them:

- a Boolean of two exact primitives under 50 ms;
- a hole through a plate in a 500-face part under 200 ms;
- rebuild of the bracket sample no slower than on master at 32 facets per
  turn, and faster than master at 1e-4 relative accuracy.

## Left to the builder

- The order of PRs within a stage, and their size.
- The data structures inside the intersection library and Booleans
  (spatial indices, marching step control, classification method).
- How each analytic pair's closed form is derived, and when marching takes
  over.
- How meshing of trimmed faces is done (as long as it is watertight across
  shared edges, which the renderer and interference check rely on).
- The exact numbers of the tolerance model, within the bounds above.

## Risks

- **Tangent and near-tangent intersections** are where exact kernels fail.
  Mitigation: tested first and hardest in stage 4, with a fuzzing corpus.
- **Tolerance stacking** across chained operations. Mitigation: per-entity
  tolerances that grow visibly, and the validator run after every operation in
  tests.
- **Naming instability** when faces stop being facets. Mitigation: the
  migration maps through logical names and reports what it cannot carry.
- **Scope.** Seven stages are years of work. Mitigation: each stage stands on
  its own and ships; the plan is revisited at the end of each.
