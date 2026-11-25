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
     as they were (below).
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
   - *For the owner:* the display's 0.35 rad angle floor, not its chord
     tolerance, sets the triangle count and the part-file cache (the Moon
     buggy draws 329k triangles, and its cache grew from 5.8 to 26.7 MB). A
     coarser angle shrinks both.

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
