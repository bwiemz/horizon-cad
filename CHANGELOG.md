# Changelog

All notable changes to Horizon CAD are recorded here. The project was built
phase-by-phase against the roadmap in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md);
this file summarizes that work by era. Each phase shipped as an honest core
slice — where the roadmap named a heavy third-party dependency, an in-house
implementation was built instead to keep CI lean and the code testable
headless. Those deviations (STEPcode/OCCT, Embree, OpenCAMLib) are documented
in [the era findings note](docs/superpowers/notes/2026-07-03-era2-roadmap-findings.md).

## Unreleased — Production readiness (Phase 97)

Work against the [production-readiness roadmap](docs/superpowers/specs/2026-09-23-production-readiness-roadmap.md).

- **The warning and sanitizer gates did not gate (97).** `cmake/CompilerWarnings.cmake`
  and `cmake/Sanitizers.cmake` defined `hz_set_warnings()` and
  `hz_enable_sanitizers()`, and nothing called either. The whole project built
  with no warning flags, and the CI job named "AddressSanitizer" configured
  `-DHZ_ENABLE_SANITIZERS=ON` into a plain Debug build — its log contains no
  `-fsanitize` at all. Both functions are now applied from the root
  `CMakeLists.txt` to every first-party target, so a new module is covered
  without opting in. GCC/Clang build with `-Wall -Wextra -Wpedantic
  -Wconversion` (MSVC `/W4 /permissive-`), and `HZ_WARNINGS_AS_ERRORS` fails
  the Linux CI jobs on any warning. UBSan is built with
  `-fno-sanitize-recover`, since by default it reports and carries on, which
  lets a test pass straight over undefined behaviour. The full suite is clean
  under ASan + UBSan + LeakSanitizer. `-Wsign-conversion` (about 560 size/index
  sites) is left off as a tracked burn-down rather than suppressed site by
  site, and clang-tidy is told the same, because Clang reads GCC's
  `-Wconversion` as including it.

  The warnings found about 25 sites, mostly dead code: unused variables, a
  lambda nothing called, an unused `flipped` flag in the angular dimension's
  arrowheads, `Eigen::Index` narrowed to `int`, a Qt signal deprecated in 6.9,
  and `Solid` forward-declared as a `struct` but defined as a `class` — legal,
  but the tag is part of the MSVC-mangled name.
- **Building on Linux without compiling Qt.** Qt is now a default-on `qt`
  feature of the vcpkg manifest; the new `linux-system-qt` preset turns it off,
  so vcpkg provides only the small libraries and the build uses an installed
  Qt 6. C++20 module scanning is off (no modules are used), which also stops
  CMake ≥ 3.28 with GCC ≥ 14 writing flags into `compile_commands.json` that
  clang-tidy cannot parse.
- **Line endings.** A checkout copied from Windows to Linux showed all 559
  files as modified — CRLF in the working tree, LF in the index.
  `.gitattributes` now pins LF everywhere.

## Unreleased — Post-1.0 kernel work, continued (Phases 89–96)

Continues against the "Not yet addressed" list in the
[post-1.0 findings note](docs/superpowers/notes/2026-09-01-post-1.0-kernel-findings.md).

- **Sweep collapsed on any turning path (89).** `Sweep` carried the profile
  along the path by translation only, which the header described as a
  fidelity limit — "the profile keeps its orientation". For a turning path it
  was a correctness defect: an XY-plane square swept up and then along +X was
  translated edge-on for the second leg, so that leg was a zero-thickness
  sheet. The L-shaped sweep integrated to 40 against 72, with two degenerate
  faces the geometric validator reports and the topology-only test never
  asked about. Separately, `SweepFeature` reduced every arc in a path sketch
  to its chord, so a bent path was swept as one straight segment.

  The profile is now carried by a rotation-minimizing frame: at every interior
  path point the section is the cut of the incoming prism by the miter plane
  bisecting the turn, which makes the cross-section perpendicular to each
  segment the profile turned by the smallest rotation between consecutive
  directions, and the far cap the profile's plane carried through the same
  turns. Every lateral face is exactly planar (its corners lie on two lines
  parallel to the segment), and with the profile's centroid on the path the
  volume is *exactly* normal-section area × path length — which the tests
  assert on a path turning in three planes. A path that doubles back, a
  profile whose plane contains the sweep direction, a turn tight enough that
  the inside of the profile would travel backwards, and any result the
  geometric validator rejects are refused instead of returned.

  Path arcs are sampled at `segments` steps per turn — a `SweepFeature`
  parameter, editable and persisted like the Phase 88 resolutions — so a bent
  sweep converges to area × arc length from below.
- **Accuracy is a distance, not a count (90).** Phase 88 made the facet count
  a feature property, but a count is the wrong unit: a fixed n sags
  r(1 − cos(π/n)), so the default 32 facets are within 0.024 on a radius-5
  cylinder and 0.48 on a radius-100 one. `segmentsForTolerance()` existed on
  `PrimitiveFactory` and `Revolve` and nothing outside the tests called it.
  Curved primitives, revolves, sweeps and fillets now take a `chordTolerance`
  parameter: when positive, the count is re-derived from it and the governing
  radius on every rebuild — the widest circle of a cone or torus, the profile
  vertex farthest from a revolve's axis, each path arc's own radius, and the
  fillet radius over the quarter arc its blend spans (new
  `FilletOp::arcSegmentsForTolerance()`). A radius edit therefore keeps the
  accuracy rather than the count. Setting the count explicitly returns to
  count mode; 0 turns the tolerance off; the tolerance is persisted and
  reloads as a tolerance, not as the count it produced. The feature parameter
  dialog's 0.001 floor would have silently switched a tolerance of 0 on for
  anyone clicking through it, so that field now accepts 0.
- **Rational NURBS surfaces were evaluated off their surface (91).**
  `NurbsSurface::evaluate` runs De Boor in two passes — each row in V, then
  across the rows in U — and gave the second pass unit weights. That discards
  the U-direction rationality, so every point of a cylinder, sphere, torus or
  cone *between* knots sat off the surface: up to 0.30 on a radius-5 cylinder,
  0.18 on a radius-3 sphere, 0.42 on a torus (about 6% of the radius), while
  every knot value was exact. The geometry tests had tolerated this in
  comments ("the two-pass evaluation loses some rational precision", "~5%")
  with tolerances of 0.2–0.5. The second pass now carries each row's weight
  sum Σⱼ Nⱼ(v)·wᵢⱼ, which makes it the exact tensor-product rational surface;
  those tests assert 1e-12, including off-knot samples on all four quadrics
  and the weighted-surface centre against the closed-form homogeneous value.
  Mate frames were unaffected only because they happen to sample at knots.
- **Extrude turned circles into squares (92).** Phase 84 found the curved
  primitives were box topology wearing a curved surface. Extrude, the most
  used feature in the product, had the same disease and was not in that
  sweep: a circle profile was extruded through box topology from four points
  on the circle, with a cylinder surface pasted onto the four flat sides, so a
  radius-5 disc extruded 10 integrated to **500 against 785**. Every arc in a
  line/arc profile was taken as its chord — a slot's round ends vanished (80
  against 111.4). Profile extraction was shared, so Revolve, Sweep and Loft
  chorded arcs the same way, and a circle section made Loft return nothing.

  Profiles are now faceted by one shared sampler: arcs are followed along
  their curve and a circle becomes an N-gon, at `segments` chords per turn or
  at the count each arc's own radius needs under `chordTolerance`, recording
  which chords came from which arc. The extrusion is an exact inscribed prism
  (the tests assert the N-gon volume to 1e-9) that converges to the exact
  solid from below. The lateral facets of an arc record their cylinder on
  `Face::analyticSurface` (for extrusions along the sketch normal) and every
  arc chord its circle on `Edge::analyticCurve`, so mates and radial
  dimensions still resolve from a single pick. A half disc now revolves to a
  sphere, two circles loft to the inscribed frustum, and a circle sweeps a
  pipe. `ExtrudeFeature` gains `segments` and `chordTolerance`, reported only
  when the profile has an arc or circle; Sweep's `segments` now also facets
  its profile. Documents that extrude a circle rebuild as the faceted cylinder
  on open, which changes their edge numbering: a fillet or chamfer that
  referenced one of the old square's edges by TopologyID no longer finds it.
- **Patterns stacked overlapping instances; Booleans and patterns dropped
  the ideals (93).** A pattern cloned every instance into its own shell
  whatever the spacing — the header called the merge "deferred" — so three
  10mm boxes 5 apart integrated to 3000 against the 2000 they occupy, and the
  six copies of a unit square patterned about its own corner were six
  interpenetrating shells that a document test asserted as correct. Every
  structural and geometric check passed. Instances whose bounds touch or
  overlap are now merged with `BooleanOp::Union` (a merge that fails refuses
  the pattern instead of returning interpenetrating shells); instances that
  stay apart remain separate bodies with no Boolean.

  Separately, the ideal geometry Phases 84–92 record was lost by the next
  operation. The pattern clone copied carriers but not `analyticSurface` or
  `analyticCurve`, and Boolean fragments were sewn without them, so a bored
  cylinder's bore — or any instance of a patterned boss — no longer resolved
  to a cylinder for a concentric mate or radial dimension. Pattern now moves
  the ideals with each instance; `BoundaryMesh` and `SolidSewer` carry
  `analyticSurface`, Boolean fragments recover it from their source face
  (looked up per operand, since two primitives of one kind share face IDs),
  and result edges lying along a source edge inherit its ideal curve.
- **Filleting a cylinder rim (94).** The open item Phases 85–86 left: a
  faceted cylinder's rim is a closed chain of chords, every vertex of which
  has two of them, and FilletOp refused any vertex with two selected edges
  ("exactly three are required for a corner blend"). So did two adjacent top
  edges of a box. Where two selected edges meet at a three-edge vertex, share
  one face, and the unselected third edge joins their other faces, the blends
  now meet on the plane bisecting the turn in the shared face: each blend's
  end section is carried along its edge onto that plane, the construction
  Phase 89 used for sweeps. The bands stay planar, the cap receives the inset
  polygon and the side edge is shortened by one radius, and no patch is
  needed. The removed material is exactly the blend cross-section times the
  length of its centroid path, which the tests assert to 1e-9 for a corner
  pair, an open chain, a square rim, a miter turned in a side face, and a
  32-chord cylinder rim. A turn too tight for the radius is refused.
- **Twisted lofts integrated the wrong solid (95).** A loft between a square
  and the same square turned 0.6 rad has lateral bands whose four corners are
  not coplanar. Such a loop encloses no well-defined volume, so each path in
  the kernel picked its own: mass properties and Booleans fanned it along one
  diagonal and got **180.8**, while the renderer drew the ruled patch, which
  encloses **150.7** — 20% apart. A level with any non-planar band is now cut
  along its rulings into strips, each non-planar strip into two triangles
  with the diagonal alternating from strip to strip. Every facet is flat, so
  the display and the computation see the same solid; and because the volume
  a bilinear patch bounds is exactly the mean of its two triangulations,
  alternating diagonals make the faceted volume equal the ruled loft's
  exactly — asserted to 1e-9 against Simpson's rule, which is exact for the
  quadratic section area. `twistSegments` (default 8, rounded up to even)
  only sets how closely the facets follow the curved patch, which each facet
  records on `analyticSurface`. Planar bands stay single quads, so aligned
  and similar sections build exactly as before.
- **Interference checking was unreachable (96).** `InterferenceChecker`
  (Phase 48) had no caller outside its tests: no assembly API, no command.
  It also reported only *whether* two solids clash — its header deferred the
  volume because the intersection Boolean was "not yet robust enough", which
  predates the kernel hardening. Each interfering pair now carries the
  volume of material the two share, from the Boolean intersection (flagged
  if that cannot be resolved); `AssemblyDocument::findInterference()` places
  every resolved, unsuppressed component by its transform and reports
  interfering pairs by component id, listing unresolved components as
  unchecked rather than passing them silently; and assemblies gain a
  **Check Interference** command. Mated faces and tangent cylinders touch
  without interfering. `Pattern::transformed()` exposes the placement copy,
  which moves carriers and ideals with the solid.

## Unreleased — Geometric validation, faceted geometry, working blends (Phases 81–88)

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
- **Faceting resolution was unreachable from the document (88).** Phases 84–87
  made resolution the knob that decides how close a faceted solid's volume
  gets to the exact one — a cylinder at 32 segments is 0.64% under, at 128
  it is 0.04% — and then left it as a kernel call argument.
  `PrimitiveFeature`, `RevolveFeature` and `FilletFeature` each hard-coded the
  default, so no model could ask for a tighter one, no parameter edit could
  change it, and every reopened file replayed at whatever the build-time
  default happened to be. Accuracy was, in effect, a compile-time constant.

  The count is now a parameter of the feature that owns it: `segments` on
  curved primitives and on revolves, `arcSegments` on fillets, reported by
  `parameters()` and validated by `setParameter()` (three steps is the fewest
  that bounds a volume; one chord is the degenerate blend that removes a
  chamfer's worth of material). That is the path the UI's parameter editor and
  the save format already went through, so it became editable and persistent
  in one move rather than two. A box is exact, so it reports no resolution and
  refuses one.

  It is written into the JSON envelope and rides the FlatBuffers container
  with it. Files written before the field existed simply lack the key and load
  at the feature's default, which is what they were built with.

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
