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
   *Landed:* the STEP reader keeps each solid's exact faces beside their
   carriers: typed planes, cylinders, cones, spheres and tori, the file's own
   B-splines, and an extrusion as a B-spline (goal 7's one exception). It
   builds every pcurve itself, never reading the file's; lays a seam on a
   periodic face that has none; grows each edge's tolerance to the gaps it
   measures, up to a thousandth of the solid's size, and says "exact to" that;
   turns a shell written inside out; and leaves out, with why, a solid it
   cannot make exact. Mass properties are integrated over the trimmed faces,
   with an error estimate that counts tolerant edges. The read point meshes
   faces that come to a pole or an apex. The writer writes typed surfaces,
   LINE, CIRCLE and ELLIPSE, a truthful `same_sense`, and scale factors folded
   in. A part made only of imported bodies carries an exact counterpart beside
   its facets, checked against their volume, meshed once and kept across
   rebuilds; the display, the mass dialog, the AI and MCP mass tools and STEP
   export read it, and the first other feature drops it. 62 of the interop
   corpus's 64 solids come in exact; OpenCASCADE's volumes agree with theirs
   to 2.4e-6 or better, and each export reads back exact, within its solid's
   own error estimate. Old part files still open: a body whose exact form
   fails opens as it did, and the load says so.

   Stage 1's carries closed here: the read point refines in place and grades
   tori (cones are staggered instead), seams, the callers' catches and one tag
   per face, the mass reason, the validator's period bounds, the
   triangulator's spread guard and fuzzer, the writer's scale factors and
   `forExtent`'s huge extents. Three go on to stage 3 (below).

   *Budgets,* measured once at the gate (Ruling 45): master's reader at
   `7b381819` and stage 2's, built alike (Debug, the same flags), timed with
   the same harness on the same texts, back to back, best of 3, twice.
   - Importing each corpus file is within 2× master: the worst is IronCAD at
     ×1.27, and the Moon buggy takes a tenth of master's time.
   - Reading back the STEP Horizon wrote before stage 2 (its faceted part,
     read as an import reads it) is within 2× master for 15 of the corpus's 18
     parts. KiCad (×2.73), Shapr3D (×2.42) and ProE (×2.11) are over. The
     time is the exact pass on B-spline faces: general-path pcurves (29–48% of
     the read) and the validator (15–25%), which runs twice on these three
     because its first pass finds last-bit rounding to take up (Ruling 66).
   - An old part file holds each imported body as it was read, one B-spline
     face per face, not that faceted export. Master's open parsed each body
     and built its carriers, and faceted it in the build after. Stage 2's
     open makes each body exact, at up to ×6.4 master's read per part (×3.7
     over all 64 bodies): ProE's 14 bodies take 0.95 s (master 0.18 s),
     IronCAD's 0.39 s (0.06 s). Only irnas (×1.97) and the emblem, which
     fails at once, are within 2×. 60 of the 64 bodies reopen exact, and four
     as they were (below). Stage 3 could not reproduce master's side of this
     comparison: with its harness, `7b381819` reads ProE's 14 bodies in
     1.75 s, not 0.18 s (stage 3's budgets, below).
   - No ctest takes over 60 s; the slowest takes 35 s.

   *Regression list,* 2 corpus solids and 12 shape classes: what master read
   faceted and stage 2 leaves out, each with its reason (anti-goal "failures
   nobody tracks"). The two solids are in 2 of the corpus's 20 files (the
   manifest's `exact_failures`):
   - `occt_euclid_screw`, solid #12: face #709 is a spindle torus (minor
     radius 54.87, major 8.25), and `Torus::invert` takes every torus for a
     ring. Stage 6, as stage 1 carried it.
   - `creo_opencellular_emblem`, solid #119: Creo writes edge #2008's end
     vertex 0.374 mm off its line, beyond the limit of 0.0853. The emblem's
     twelve shells are one surface model, so all twelve bodies master read are
     left out.

   The classes, found by the stage's tests and probes. The first two are the
   corpus solids'; the last is the corpus parts' own exports from before
   stage 2:
   - a spindle torus (stage 6);
   - an edge, vertex or seam standing off its face beyond the limit, a
     thousandth of the solid's size;
   - a sphere, torus or closed B-spline face whose region holds a pole or
     wraps the surface, so that its loops run against its `same_sense`: a
     Parasolid-style ball with a flat, a torus cut into a patch and the rest
     (Ruling 17);
   - a ball split into two faces by two meridians;
   - a hole within about 0.03 rad of a sphere's pole ("inner loop lies outside
     the outer loop");
   - a B-spline ball bounded by one pole-to-pole seam;
   - a B-spline face whose pole row spreads 3e-6 or more ("closes only after
     winding round");
   - a B-spline sphere or torus face, or a B-spline face narrower than a
     thousandth of its part, written with a careless `same_sense` (the reason
     names both possibilities);
   - a torus band whose loops wind round it both ways, as a helix does
     (Ruling 29);
   - a closed B-spline band on which no seam can be laid: its two loops wind
     round it the same way, or its seam would cross where it closes (Ruling
     27; old part files hold Onshape's and Solid Edge's bands so, below);
   - a band rim tilted so that its gap peaks between the pcurve's samples
     (Ruling 23's aliasing: a rim tilted 8e-4 strays 3.7e-7 past its
     tolerance);
   - STEP that Horizon exported before stage 2, wherever a round face's
     radius is large beside its part: plain primitives as well as parts. The
     old writer wrote each flat face as a bilinear B-spline patch sized to its
     facet polygon, the 32-gon's oriented bounding box, whose half-side is
     r·cos(π/32). A cap's true circular rim bulges past that patch by the
     sagitta, r(1 − cos(π/32)) ≈ 0.0048 r, and the solid is left out once that
     passes the limit, a thousandth of the solid's size: for a radius more
     than about a fifth of the part's diagonal. A cylinder of radius 10 and
     height 20 is left out ("its curve stands 0.0482 off its face's surface,
     beyond the limit of 0.0346"), and so are a frustum (radius 6 to 2, height
     9) and a tube; a box with a hole, a boss, a sphere, a torus and an apex
     cone read. A pcurve straying from its edge on such a patch leaves a solid
     out too. 5 of the 18 corpus parts' exports are left out so (Alibre,
     build123d, FreeCAD, Fusion, the Moon buggy, the last 4.5 off at a limit
     of 4.49). Such a file is a fresh STEP import, so decision 1b holds: left
     out, with why. Old part files do not hold this text; they hold each body
     as it was read (Ruling 10).

   Changed, not left out: the faceted copy of a body, which features work on.
   A band's changes with its new seam: Onshape's faceted volume is now 0.81%
   under its exact one (it was 0.61%), Alibre's 0.63% (0.61%), and IronCAD's
   shrinks too. This holds for old part files on open as well, so a saved
   part's features rebuild on a different copy than they did. Of the corpus's
   64 old-part bodies, 8 read exact with a faceted copy of another volume
   (master's, then stage 2's):
   - IronCAD, 7.0216778e7 → 7.0213538e7 (−4.6e-5);
   - Alibre, 43.903179 → 43.895323 (−1.8e-4);
   - the Prusa hinge, 8541.6114 → 8542.0277 (+4.9e-5);
   - the NX plate, 35698.440 → 35698.696 (+7.2e-6);
   - four Moon buggy bodies, 1022987.76 → 1023026.43 and 869544.08 →
     869572.47 (+3.8e-5 and +3.3e-5), and two of 19181081.8 → 19181745.6
     (+3.5e-5).
   The Moon buggy bodies and the NX plate change with their new seams alone.
   The hinge changes with the B-spline fixes alone: `NurbsSurface::locate`
   and `project` (Ruling 12) place its facets' points otherwise. IronCAD and
   Alibre change with both. The other 56 bodies' faceted copies keep their
   volumes to 8.8e-8 or better, and the same edges and facets. Among them are
   the two bodies kept as they were, Onshape's and Solid Edge's, which move
   by the B-spline fixes alone (708529.956 → 708529.894, 684.443891 →
   684.443858).

   Carried to later stages:
   - *Stage 3, transforms:* a non-similar transform converts the copy to NURBS
     (stage 1's; no stage-2 path transforms an exact solid other than
     similarly).
   - *Stage 3, mixed solids:* orient the faceted subset of an exact plus
     faceted solid (stage 1's; stage 2 makes none).
   - *Stage 3, mesh cache:* each face's mesh cached (stage 1's). The display
     meshes once per build and keeps the counterpart across rebuilds; exact
     construction is what re-meshes faces.
   - *Stage 3, bridge:* `facetCurved` for faces with no carrier (every
     imported exact face keeps its carrier). `Pattern::bodyShells` tells voids
     by their polygons or carriers, so a carrier-less exact solid with a void
     could be misread.
   - *Stage 3, poles* (stage 1's pole topology, now with its cost on the
     regression list): faces whose region holds a pole or wraps the surface,
     a ball of two meridian faces, a hole near a pole, a pole-to-pole seam on a
     B-spline ball, spread pole rows. The mesher knows poles on the outer loop
     only: a hole touching a pole, a zero-length edge along one, or an edge a
     hair past an apex fails to mesh, saying so.
   - *Stage 3, seams:* a torus band winding round both ways needs both seams,
     which the joined loop's cancelling winding cannot place (Ruling 29).
   - *Stage 3, pcurves:* a gap that peaks between `pcurveOn`'s samples reads up
     to ~28% low, so an "exact to" can read low and a tilted band rim is left
     out (Rulings 23 and 64); the fix belongs with the tolerance model.
   - *Stage 3, assemblies:* exact STEP export of an assembly. Its components
     are built without their counterparts, so it writes facets, as master does
     (Ruling 56).
   - *Stage 3, mirror:* `Pattern::transformed` with a reflection leaves a
     cone's exact solid failing the validator ("closes only after winding
     round the surface"). Stage 2 never reaches it, since a mirror drops the
     counterpart (Ruling 47).
   - *Stage 3, callers:* a script's mass properties are still faceted (the
     dialog, the AI and MCP read the counterpart); an interference pair's
     reason is not shown; the status line's exact note is not translated;
     exact mass is integrated on the GUI thread under `RebuildMode::Always`
     (IronCAD 391 ms, Debug); STL export meshes the kept counterpart again.
   - *Stage 3, mass:* a dense B-spline face (~150 × 150 knots) can run out of
     its evaluation budget; a thin part's estimate stops at Gauss–Kronrod's
     floor.
   - *Stage 3, old parts:* a body whose exact form fails opens from facets
     with a note until format 32's migration (Ruling 10). Four of the
     corpus's 64 bodies do: Onshape's and Solid Edge's seamless bands, which
     the old writer wrote as closed B-splines ("its two loops wind round it
     the same way"), a Moon buggy body ("its seam would cross where its
     surface closes"), and the emblem (its vertex off its line).
   - *Stage 3, speed:* the overshoot of budget 2, and of opening an old part,
     is accepted for stage 2 (Ruling 67). Fresh imports, the main path, meet
     budget 1 (the worst is ×1.27). Opening the corpus's largest old part takes
     under 1 s in Debug, and the added time is the exact pass that is this
     stage's purpose. The speed work goes to stage 3: general-path pcurves on
     B-spline faces, the validator, and its second validation (Ruling 66 runs
     it twice on six of the 18 old exports). Stage 3 migrates every old part
     on open, so it meets this cost first. Two more costs, measured in Debug
     with the same harness:
     - *Open to display.* The first build after opening a part made of imports
       also meshes and integrates its exact counterpart, where it faceted the
       part alone before: the Moon buggy 1.35 → 4.83 s, IronCAD 0.23 → 1.35 s,
       ProE 3.11 → 4.57 s, Shapr3D 0.20 → 0.29 s. Each later build reuses the
       counterpart while the imports are as they were.
     - *Each rebuild.* A worker's rebuild copied the part by writing its
       imported bodies as STEP and reading them again through the exact pass,
       and the copy, a last bit otherwise, meshed the counterpart again
       whenever a build changed threads (Ruling 51's key). The copy now shares
       the document's bodies, which nothing changes once read, so a rebuild
       reads no STEP and either thread reuses the other's counterpart. Per
       rebuild of an import-only part (copy, read and build, the document's
       counterpart offered): IronCAD 1656 → 229 ms (master 478), the Moon
       buggy 4991 → 1385 ms (master 9599), ProE 3484 → 3216 ms (master
       3785), Shapr3D 338 → 201 ms (master 277), and IronCAD's old part
       2050 → 228 ms (master 477).
   - *Stage 3, platforms:* two results can differ by the last bits a
     platform's maths library gives.
     - A body's faceted copy can differ by one column of a band's facet grid
       where the band's width divided by its column spacing is a whole number
       but for rounding (`trimmedFacets`, as on master). The Solid Edge old
       part gives 3,796 faces on one platform and 3,868 on another, its volume
       1.7e-5 apart, within the facets' own error. Making the grid
       deterministic would move the faceted copies of a dozen corpus bodies,
       so it waits until the bridge's last users go; stage 7 deletes it.
     - An edge whose stray sits on Ruling 66's rounding bound (16 units in the
       last place of what its measure is computed from) can be taken up on
       one platform and reported on another. No corpus file sits there: the
       manifest, whose `exact_failures` is strict both ways, reads alike on
       Linux, Windows and macOS.
   - *Stage 5:* still no geometric orientation check in the validator; the
     counterpart's volume check against its facets stands in for it.
   - *Stage 6:* the spindle torus, unchanged (the screw is on the regression
     list).
   - *For the owner, settled after stage 2 (2026-11-26):* the display's
     weight on a large part was its chord tolerance, not its angle. The Moon
     buggy drew 329,308 triangles, its cache 26.5 MB (5.8 MB on master), and
     302k of them were its four wheels: spheres of radius 254 and tori of
     127 about 254, meshed to 0.1 on a part 4488 across. The angle did not
     move it (329,324 at 0.5 rad, 329,436 with no angle limit at all). The
     owner chose 0.5 rad between neighbouring triangles (26 chords a turn
     where the angle sets them, where 0.35 made 36) and a chord tolerance of
     max(0.1, 1.5e-4 × the part's diagonal). Triangles and cache, stage 2 →
     the owner's tolerance: the Moon buggy 329,308 → 49,462 (26.5 → 3.9 MB);
     IronCAD (1263 across) 41,242 → 20,892 (3.9 → 2.1 MB); ProE (1235
     across) 76,238 → 43,430 (7.9 → 4.7 MB); irnas (112 across, at 0.1
     still) 10,000 → 5,568 (1.3 → 0.8 MB).

     Stage 3 steps the part-relative tolerance down to a quarter octave
     (stage 3's Ruling 29; `SolidTessellator::displayTolerance`): 0.1 ×
     2^(k/4), the largest such step not above 1.5e-4 of the diagonal, so
     never coarser than the owner's and at most 19% finer. A part up to
     about 793 across is drawn to 0.1, and a small edit to a larger one
     keeps its tolerance, and so its bodies' meshes. Measured in stage 3,
     the owner's → stepped: the Moon buggy 49,460 → 50,452 triangles
     (3.95 → 4.02 MB), IronCAD 20,878 → 23,844 (2.07 → 2.34 MB), ProE
     43,430 → 46,410 (4.71 → 5.10 MB); irnas, at 0.1, 5,608 either way.

   For the owner to check, the decisions stage 2's plan made:
   - *The exact counterpart, not exact parts everywhere.* Operations and every
     reader of ideal labels (mates, silhouettes, sketch on a face, fillets,
     drawings) keep getting the faceted part; only rendering, mass and export
     read the counterpart. It widens stage by stage until the faceted part is
     deleted in stage 7.
   - *The bridge stays `facetCurved` of the carriers,* since every imported
     exact face keeps its carrier.
   - *Pcurves are built, never read from the file:* one code path, as
     exporters parametrise analytic surfaces' pcurves differently.
   - *SURFACE_OF_LINEAR_EXTRUSION round-trips as a B-spline;* there is no
     typed extrusion surface yet.
   - *Old part files' embedded STEP is NURBS only,* so an imported body saved
     before stage 2 reopens with exact B-spline faces, not typed ones: still
     exact, and files saved between stages are best effort (decision 2).
   - *Deferred to stage 3:* non-similar transforms, mixed solids and the
     per-face mesh cache (above).
3. **Exact construction.** Primitives, extrude, revolve (and with it holes,
   sheet metal's bends) build exact faces; migration of old parts; naming
   without facets. *Seen:* exact volumes, a cylinder of three faces, smooth
   silhouettes in drawings at any zoom. (A part with a Boolean is still
   faceted after it until stage 5.)
   *Landed:* primitives, extrusions, revolves, holes' tools and sheet-metal
   flanges build exact faces, each named as master named its facets, less
   the facet (`<feature>/side`, `side:<entity>`), and their edges by
   `nameEdgesLogically`. A cylinder is three faces. A cone runs to its apex
   and a sphere to its poles with no degenerate edge: a loop jumps along the
   pole in (u, v). A torus is typed; one whose tube reaches its axis is
   refused, as master refused it (Ruling 2). An extrusion's side is a plane
   for a line, a cylinder for an arc swept square to its sketch, and a NURBS
   extrusion otherwise; a hole in its profile is an inner loop of each cap,
   not a Boolean, and each region is a shell of its own. A revolve turns
   each curve of its profile into a plane, a cylinder, a cone, a sphere, a
   torus or a NURBS surface of revolution. A hole's tool is its half-section
   turned exactly. A sheet's bend is two cylinders, its inside at the bend
   radius and its outside a thickness further, and the sheet's faces are
   named after its sketch, the same in every build, where master's changed
   with each (Ruling 28). Volumes are their closed forms to 1e-9: the pin
   sample is 270π (848.230016), where its 32 facets enclosed 842.790191. The
   rulings named in this entry are stage 3's
   (`docs/superpowers/notes/2026-11-26-exact-kernel-stage3-rulings.md`).

   A part is exact while every body in it came, as a new body, from one of
   these features or from an exact import. The build carries that exact part
   beside the faceted one, as stage 2 carried an import's counterpart. The
   display, the mass dialog (on a worker, under the rebuild mode's rule),
   scripts, the AI and MCP mass tools, and STEP, STL and glTF export read
   it, and an assembly's STEP export builds copies of its parts to write
   each exact. A Join, a Cut, an Intersect, a hole (which is a cut) and every
   feature not yet converted drop it: the part is faceted from the first of
   them on, until stages 5 and 6 convert them.

   Every operation not yet converted reads the faceted copy, as in stage 2.
   Where master built a feature's body by one construction (a primitive, an
   extrusion or revolve of one loop, a hole's tool, a sheet's flange), the
   copy is master's own construction, named by master's own pass (Rulings 44
   and 45). Its faces, loops and edges are in master's order, so an
   operation after it numbers pieces and chords as master did and gives
   master's result. An extrusion or revolve of several loops is cut from its
   exact solid instead (`facetExact`, Ruling 23). `segments` and
   `chordTolerance` shape only the copy now. A picked or hovered face of an
   exact part is highlighted from the copy's facets, masked by the face as
   drawn, so a coarse part shows the facets a click picks (owner A1, Ruling
   39).

   The display meshes each body on its own and keeps its mesh while the body
   and the part's tolerance hold: a primitive added to the IronCAD impeller
   and edited rebuilds in 248 ms, where it took 1006 ms (Debug). A large
   part's display tolerance steps by quarter octaves (Ruling 29; stage 2's
   settled display, above), so a small edit keeps it.

   A sheet of an exact part is drawn from a copy cut from its exact faces at
   0.01 mm on paper for the sheet's views, details aside (Ruling 35). A
   cylinder's outline on a 1:1 sheet lies within 0.01 mm of its silhouette,
   where a 45-sided faceted part's lay 0.12 mm inside. Projected edges are
   still chords, and a detail's sag up to 0.01 mm times its scale over its
   view's.
   A line inside a face of the copy is named for the cut it is part of
   (`<face>/line:<n>/of:<m>/at:<k>`, Rulings 31, 34 and 36), so a dimension
   on one is reported lost after a change of scale, or an edit that cuts its
   face again, and is never measured on another line. An assembly's sheet
   draws its components' faceted parts.

   Old parts are migrated when they open, and written as format 32. Every
   feature is named Stable. A stored reference that can no longer mean what
   it meant is left out and listed, never guessed: one that a feature named
   the old ways made, one into a sheet's faces (Ruling 28), and a piece,
   region, chord or seam of an extrusion or revolve of several loops (Ruling
   23). Facet and chord references are kept as written, since the copy
   numbers facets and chords as master did, and mapping a chord to its edge
   would widen it (Ruling 40). A part that loses a reference is read in
   part: Save becomes Save As, and saving over the original asks first
   (owner A2). A feature drawn with fewer than 12 chords a turn is listed as
   approximated ("drawn as a 6-sided prism before; round now"), which does
   not stop a save. A migrated part whose first exact build fails says so,
   and is read in part. An old assembly's mates that find nothing are
   reported when it opens, and so are a sheet's dimensions that find no
   edge ("Not drawn:"), which do not make the sheet read in part (Ruling
   41). The cached mesh of a part of format 31 or older, master's facets, is
   not read.

   The kernel under them: typed surfaces and curves convert to exact NURBS
   (`toNurbs`), so a non-similar copy stays exact and a mirrored cone
   validates. A ball of two meridian faces reads exact. A cusp at a pole is
   refused at once, saying why, where the read point failed after about
   12 s (Ruling 9). A pcurve's gap is measured at its peak, not only at its
   samples, so a tilted band rim reads exact, to the peak of its gap. A
   B-spline face's mass budget grows with its spans, to a ceiling of 1e8
   evaluations (Ruling 10). A torus band whose loops wind round both ways
   gets one seam and is cut in two across it, so a STEP round trip writes it
   as two faces (Ruling 12). Rims the old writer wrote the wrong way round
   are turned, so Onshape's and Solid Edge's old-part bodies open exact
   (Solid Edge's volume 684.683542908, OpenCASCADE's 684.683543).

   Stage 2's carries closed here: non-similar transforms and the mirrored
   cone; the bridge for faces with no carrier (`facetExact`, which refuses a
   face with no surface rather than outline it); voids beside a body built
   exactly (once a shell has no carriers, `Pattern::bodyShells` measures
   every shell whose faces are all exact by its exact mass); the ball of two
   meridian faces; bands winding round both ways; gaps that peak between
   samples; exact STEP export of assemblies; the callers (a script's mass,
   an interference's reason, the exact note's kind translated, exact mass
   off the GUI thread, STL and glTF from the kept mesh where it was made at
   their tolerance); a dense face's mass budget and the thin part's floor,
   which the dialog now states; and the speed of opening old parts (below).
   Settled otherwise:
   - mixed solids go to stage 5 (below): a part is exact only while every
     body is, so this stage makes none;
   - the mesh cache is per body, not per face, since the read point samples
     an edge from both its faces, so a face's mesh depends on its
     neighbours';
   - the other pole cases go to stage 5 (below): only imports make them
     now;
   - a body whose exact form fails still opens from its facets, with a
     note, after migration: two of the corpus's 64 old-part bodies.

   Stage 1's stage-3 carries are closed too. Poles and apexes have no
   degenerate edges; a loop's jump along a pole is laid round it the way the
   face's sense says; `sameUv` reads the arc along a pole, not its chord;
   and a recognised sphere's frame z is moot, since construction sets every
   frame and imports are never recognised (goal 7). Exact primitives name
   their edges. Vertices are not named, since nothing refers to one; one is
   named the day something does.

   *Budgets,* each measured when the work it times landed: the build and its
   reference built alike, timed with one harness on the same machine, back
   to back, best of 3, twice.
   - Rebuilding the bracket sample at 32 facets a turn, in Release (the
     budget above): 4.273 and 4.256 ms, against master's 4.409 and 4.375
     at `6e822892` (×0.969 and ×0.973). The exact builds add about 0.02 ms
     on the extrusion and 0.003 ms on a hole's tool, and the sewer's
     speed-ups make master's own construction faster than at `6e822892`.
     The budget's second half, faster than master at 1e-4 relative
     accuracy, waits for stages 5 and 6: the bracket is faceted after its
     fillet on both builds.
   - Reading the STEP Horizon wrote before stage 2 (stage 2's budget 2) is
     within 2× of `7b381819` for all 18 of the corpus's parts, in Debug.
     After construction landed, the two nearest the limit read at ×1.98
     (KiCad) and ×1.94 (Fusion).
   - Opening an old part. Its bodies' reads, summed over the corpus's 18
     old parts in Debug, take 2.23 s, where `7b381819` takes 8.20 s (×0.27)
     and stage 2's reader 2.77 s; every part's bodies read faster than on
     `7b381819`. ProE's 14 bodies take 0.69 s (`7b381819` 1.75 s, stage 2
     0.95 s), over the plan's 0.5 s target, and accepted (Ruling 15): they
     take 107 ms in Release, where the owner asked for well under 0.25 s,
     and what is left is the measuring itself (pcurves 42% of the read, the
     validator 18%, parsing 16%, the shell-sign check 12%).
   - Stage 2's comparison of this open with master's read ("up to ×6.4
     master's read per part"; ProE "master 0.18 s") does not reproduce.
     With the same texts and one harness, `7b381819` reads ProE's 14 bodies
     in 1.75 s, and stage 2's reader read every one of the 18 parts' bodies
     faster than `7b381819` did (2.77 s against 8.20 s in all). Master's
     side of that comparison measured something else.
   - The validator's quadratic checks, in Release (owner A4, Ruling 14):
     at most 2.6% of an old part's open and 1.6% of an import, 6.4–6.9% of
     an old faceted export (ProE 86 of 1357 ms). None passes 10% or 100 ms,
     so no prefilter.
   - A sheet of a corpus body and a primitive, drawn from the exact faces in
     Debug, within max(1 s, 3× master's sheet) (Ruling 30): the Thor base
     656 ms, the Shapr3D lid 611 ms, the Inventor hinge 983 ms. The IronCAD
     impeller's takes 1.72 s, over, and is accepted (Ruling 33): each
     B-spline face is cut by its largest curvature throughout.
   - No ctest takes over 60 s; the slowest takes 37 s. 3,109 tests pass; 20
     skip here, 10 that need a GL context and 10 a comma-decimal locale.
   - *Regression list,* 3 shape classes refused and 1 shown from its facets:
     what master built and stage 3 does not, each with its reason (anti-goal
     "failures nobody tracks"). Master is the commit just before stage 3,
     `6e822892` (Ruling 42). The regression set, run on both:
     - every test: master's 2,897 against the branch's 3,109. None of
       master's was dropped or changed for a refusal. The four it no longer
       has were renamed for what stage 3 now builds or migrates;
     - the samples. Each builds as on master. The bracket and the plate are
       faceted after their fillet and holes: master's counts, volumes and
       names. The pin is exact: 3 faces and 848.230016 (270π), where its 32
       facets enclose 842.790191. The assembly is placed by its mates, and
       the sheet is drawn as master drew it;
     - the STEP corpus: the manifest is unchanged, and every file reads as
       it says;
     - `test_AdversarialModels`: unchanged, 9 of 9. Its kinds of operand (a
       box, an L-shaped extrude), built as features, are exact, with master's
       faceted copies;
     - the beta tasks in `docs/BETA.md`, scripted through the document API,
       come out as on master:
       - task 1: a part with a hole and a fillet, edited, measured, saved and
         reopened;
       - task 2: two of it in an assembly with a coincident and a concentric
         mate, saved, reopened, and following the part when it changes;
       - task 3: a sheet with a length, saved, reopened and written as DXF.

       What a person does with the mouse, and opening the exports in other
       programs, was not scripted;
     - 124 probe models, built as a user builds them: primitives, extrudes,
       revolves, holes, sheet metal, and operations after them.

     Each refused or shown class is pinned in
     `tests/document/test_RegressionSet.cpp`, or in the test named with it.

     Refused. The feature fails with `could not be built exactly: <why>`, and
     master's construction never runs in its place (decision 1b):
     - *A revolve whose profile has an arc reaching the axis between its
       ends:* a circle touching the axis (a horn torus), an arc through the
       axis, or a ring whose inner side bulges to it. The reason is "an arc
       of the profile about (x, y) reaches the axis between its ends". Its
       surface pinches to a cusp on the axis inside the face. The exact faces
       have no topology for a pole there, and the read point would not mesh
       it. A torus primitive whose tube reaches its axis is refused as master
       refused it (Ruling 2). Carried to stage 6 with cusp meshing (Ruling
       9; below).
     - *A revolve whose profile has regions on both sides of the axis.* The
       reason is "its regions lie on both sides of the axis". Master turned
       each region and joined them with its Boolean. Two regions turned from
       both sides of one axis give shells that meet or overlap, which needs
       an exact Boolean: stage 5.
     - *A sketch whose curves meet within the profile's join tolerance
       (1e-6) but miss by more than a thousandth of the part's size,*
       extruded or revolved. Only parts under about a micrometre are
       affected; the test is a slot 0.3 µm across missing by 0.9 nm. The
       reason is "the profile's curves miss each other at (x, y) by g, more
       than its exact form can take at its size". The limit is the tolerance
       model's (`topo::ToleranceGrowth`), the same for imports. No stage moves
       it (`ExactConstruction.AFailedExactExtrudeFailsTheFeature`).

     Built, but shown and measured from its facets as master showed it, with
     a "shown from facets" note:
     - *A revolve whose face comes to a cusp at a pole:* its surface opens
       from the pole at under 30 degrees and wider further out. Two shapes
       do this: an apple (the part of a circle outside the axis it crosses,
       R/r above about 0.866), and an arc meeting the axis tangentially at
       its end. The read point refuses the pole at once (Ruling 9), and the
       exact counterpart is dropped with why. A cone's apex and a lemon's
       tip still mesh. Carried to stage 6 with cusp meshing.

     *The faceted copy is master's own construction* wherever master's
     construction of the body was one construction (Ruling 44). That holds for
     primitives, a single loop's extrude and revolve, a hole's tool and a
     sheet's flange. Its faces, loops and edges are in master's order and
     winding. Its names are master's own, with no copy cut for them (Ruling
     45). The exceptions: an exact seam's chords are named after the seam
     (Ruling 18), and a sheet's names come from its sketch (Ruling 28). So
     what an operation after it numbers by the order it meets them is
     master's:
     - the pieces of a face a Boolean splits (`<face>/piece:<n>`);
     - the chords and seams that a Boolean, shell or pattern names again.

     The regression set found that the bridge, stored otherwise, moved those
     numbers silently. A sheet's length saved on master on a drilled box's
     `…/edge:bottom/piece:0|front` read 6 where master read 14. The set's
     downstream cases are now named as master named them at the same places
     (`DownstreamNumbering`, tables recorded at `6e822892`): a join, a cut, a
     shell, a pattern, an extrude with arcs, a revolve and a hole. So are
     the bracket and plate samples, sheet and all. A partial turn's flat
     faces are master's quads again, and every bend is cut into its
     `bendSegments` chords, whatever its angle.

     Changed, not refused. An old part says so when it opens, wherever a
     reference of its depends on one of the first two (Rulings 23, 28 and
     40):
     - *Faceted copies that are not master's:* an extrude or revolve of
       several loops (holes, islands, regions). It is its exact solid's
       facets, numbered its own way (Ruling 23), not master's Boolean of its
       loops. A plate with two holes has 70 faces (master 72). A washer
       turned 90° encloses 80.399405, the facets' own volume; master's
       Boolean cut 5e-5 of it more (80.395175). Its pieces, regions, chords
       and seams, and the pieces an operation after it makes, are reported
       when an old part opens. A facet named by master's Boolean may stand
       for a part of the bridge's facet of that name, the same facet uncut.
     - *Names that are new:* sheet metal's (Ruling 28).
     - *Seam chords, renamed and not reported.* An exact seam (a sphere's
       meridian, a torus's circles) takes master's name for its first chord,
       and its chords are `<that name>/chord:<k>` (Ruling 18). A grid line
       inside a curved face keeps master's name, unless that name is now an
       exact seam's; then it is numbered past the names taken (Ruling 45).
       The migration does not report these: nothing refers to a smooth grid
       line, since fillets and chamfers work between flat faces (Ruling 18).
     - A part file of format 31 or older has its cached mesh, master's
       facets, left unread: an assembly reads and builds the part instead.
     - A primitive whose dimensions master refused is refused as before,
       now in the exact build's words ("could not be built exactly: these
       dimensions do not make a solid ...").

     Imports keep stage 2's list (above), now shorter: two of the corpus's 64
     old-part bodies open from their facets, the Moon buggy's body 15 ("its
     seam would cross where its surface closes") and the emblem. Onshape's
     and Solid Edge's bands now open exact.

   Carried to later stages:
   - *Stage 4:* closed-form intersections read the surfaces' scale and
     axialScale (stage 1's, unchanged).
   - *Stage 5, mixed solids:* a part is exact only while every body is, so
     no part yet holds exact and faceted bodies together. Stage 5's
     Booleans make the first, and stage 1's orientation of the faceted
     subset goes with them.
   - *Stage 5, the faceted copy:* every operation not yet converted reads
     it, so `segments` and `chordTolerance` still shape it until Booleans
     are exact, and an operation after a converted feature gives master's
     result. `BooleanOp`'s result depends on where a face's loop starts
     (`FragmentMerge`'s `removeCollinear`;
     `LargeProfileTest.AFinelyFacetedPrismsCapMeetsItsSidesCornerToCorner`),
     and the exact extrusion starts its caps' loops where master's prism did
     for it (`mastersCapStarts`). Both go when Booleans are exact.
   - *Stage 5, revolves across the axis:* a profile with regions on both
     sides of the axis is refused (above). Its two shells meet or overlap,
     which needs an exact Boolean.
   - *Stage 5, poles:* only imports make these now, and stage 5's Booleans
     make them too (a ball cut flat, a ball drilled at its pole). On import,
     a face whose region holds a pole or wraps its surface, a hole near a
     pole, a pole-to-pole seam on a B-spline ball and spread pole rows are
     left out (stage 2's list). The read point knows poles on a face's outer
     loop only: a hole touching a pole, a zero-length edge along one, or an
     edge a hair past an apex fails to mesh, saying so.
   - *Stage 5, transforms:* patterns and mirror, which stage 5 makes exact,
     copy exact solids by `Pattern::transformed`. Its old overload drops a
     non-similar copy's exact form without saying why (Ruling 4), and a
     sheared sphere's copy drops it with why, since its pole-to-pole seam
     needs a pcurve that knows poles (Ruling 5). Nothing in the application
     transforms an exact solid non-similarly yet.
   - *Stage 5, the validator:* its O(n²) self-intersection and hole checks
     (at most 2.6% of an old part's open in Release, above), a crossing at a
     sample point, near-concentric loops, and a global orientation check, as
     stage 1 carried them. The exact part's volume check against its facets
     stands in for the last.
   - *Stage 5, assembly sheets:* they draw their components' faceted parts,
     since a component is built without its exact part (Ruling 20). Most
     parts are faceted after a hole or a cut until stage 5, so few
     components have an exact part to draw before it.
   - *Stage 6, the spindle torus and cusps:* `Torus::invert` takes every
     torus for a ring (stage 1's; the screw stays on stage 2's list). A
     typed `Torus` is made only where its ring is wider than its tube: a
     revolve's apple, an arc of a circle that crosses the axis, is a NURBS
     surface of revolution, and a torus primitive whose tube reaches its
     axis is refused as master refused it, so nothing new reaches that
     limit. An apple is only the part of its circle outside the axis; the
     lemon inside is a second face, added when it is wanted (Ruling 2). The
     read point refuses a cusp at a pole at once (Ruling 9): one opening
     under 30 degrees, a threshold not scaled with the chord tolerance; it
     looks for poles on the outer loop only. With it go the horn revolve
     refused above and the apple and tangent arc shown from facets. Fillets
     make spindle tori, so these go with stage 6's blends.
   - *Stage 6, fillets' segments:* a fillet of fewer than 4 arc segments is
     not reported at migration (spec: Migration), since fillets are not
     exact yet and their segments still shape them.
   - *Stage 7, drawings:* a sheet is drawn from a faceted copy cut at
     0.01 mm on paper, so projected edges are chords, not arcs and
     ellipses, in views and in a sheet's DXF export. A detail view is drawn
     from its view's copy (Ruling 35). A B-spline face is cut by its largest
     curvature throughout (Ruling 33). The grid's cap of 400 chords a
     direction, a bound against hostile files, leaves an imported rim over
     about 325 mm in radius sagging a little more than 0.01 mm on a 1:1
     sheet (Ruling 32). Stage 7 deletes the faceted bridge, so drawings
     project exact edges by then.
   - *Stage 7, picks:* picks, mates, sketches on a face and the AI's face
     lists read the faceted copy. An exact part's face highlight is drawn
     from it (owner A1) in a part's tab only; an assembly's component is
     highlighted from its exact mesh. They read exact faces when the faceted
     copy goes.
   - *Stage 7, coarse display:* `segments` and `chordTolerance` shape only
     the faceted copy. A feature's coarse display (spec: Migration) waits
     until the bridge's last user goes.
   - *Stage 7, naming:* the Positional and FromGeometry builders stay for
     the code and tests that set those schemes, and master's Stable pass
     names the faceted copy beside the exact solid's own naming (Ruling 45).
     Both go with the faceted kernel.
   - *Stage 7, platforms:* the facet grid in `trimmedFacets` can differ by a
     column between platforms (stage 2's), and goes with the bridge. An edge
     on stage 2's Ruling 66 rounding bound stays as stage 2 left it: no
     corpus file sits there.
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
