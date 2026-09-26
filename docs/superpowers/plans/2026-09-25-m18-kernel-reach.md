# Milestone 18 — Kernel reach (Phases 162–165)

Roadmap: [2026-09-25-professional-workflows-roadmap.md](../specs/2026-09-25-professional-workflows-roadmap.md),
findings K1–K4, and A4's mirrored components (moved here from Phase 161).

## Baseline (checked in the code)

- **Nothing mirrors a solid.** 3D code has no mirror or reflect; the only
  mirrors are 2D drafting ones (`DraftEntity::mirror`, `MirrorEntityCommand`).
  - `Pattern::transformed`/`append` (`cloneInto`, `Pattern.cpp:55-194`)
    move points, carriers and true surfaces through any matrix, and keep
    every loop's order. With a reflection (determinant −1), every loop
    winds the other way: the solid comes out inside out.
  - Nothing notices. `checked()` looks at linkage, Euler and geometry, not
    orientation. Some consumers repair a solid that is inside out *as a
    whole* (`BoundaryMesh::extractFacePolygons`, the tessellator, mass
    properties, `outwardSign`). None repairs one that is mixed, which
    `Pattern::collect` (new bodies) and `append` (assemblies) make. STEP
    writes planar faces with their loops as they are.
  - There is no helper to reverse a face or a solid. `Solid` exposes no
    mutable half-edges or wires. The nearest code is `ringstack::
    orientOutward` (polygon soups) and a file-local `reverseSurfaceU` in
    `StepFormat.cpp`.
  - `Mat4` has no reflection and no determinant.
- **There is no Hole feature** (K3). A hole is a sketch and a cut
  extrusion today, with no counterbore, countersink or drill point.
  - `Revolve::execute` makes a true cylinder for a profile edge parallel
    to the axis and a cone for an oblique one, and may touch the axis. So
    a hole's half-section, revolved a full turn, is a drill, counterbore
    or countersink in one solid, with true surfaces for STEP and drawings.
  - A body feature (`createsNewBody()`, operation Cut) is given the part
    in `BuildContext::part` and returns its tool; the tree subtracts it.
    `PatternFeature` with targets repeats such a feature's tool, so a
    Hole patterns with no extra work.
  - A face is followed by its whole name (`planeOfFace`), as a sketch on
    a face and Extrude's Up to face do. A click gives the face, not a
    point on it (`ModelPick` has no hit point).
- **A new feature type needs NativeFormat both ways**: a background
  rebuild copies the document through JSON (`RebuildJob`), so a type the
  format does not read vanishes from worker builds.
- **There are no truly curved faces.** Cylinders, cones, tori, revolves and
  imported curved faces are planar facets, each carrying its ideal surface
  (`Face::analyticSurface`) and each chord its ideal curve. A
  "plane–cylinder edge" is a chain of straight chords between flat faces.
- **Shell (K1)** hollows only a right prism with one open face
  (`Shell.cpp:208-375`). It rebuilds the part from four rings of the
  profile offset inward, so the input's surfaces, ideals and names are
  lost. Other shapes, two open faces, holes and several bodies are
  refused, each with a message. There is no face or surface offset in the
  kernel. The pieces to build one exist: `planeOf`, `frameForFace` for a
  curved ideal's axis and radius, half-space clippers, `Mat3::inverse` for
  a three-plane corner, and `SolidSewer::sew` from a polygon soup with
  holes.
- **Fillet (K2)** is topology surgery, not a Boolean. It rolls a ball
  along each edge, with flat bands whose ideals are per-chord ruled
  patches, and sphere corners; mitered chains (Phase 94) fillet a
  cylinder's rim. It reads face loops' Newell normals and never checks a
  face is planar. A revolve's rim, with facets on both sides, is refused
  ("must share exactly one face"). A rim's bands have no torus ideal.
  `NurbsSurface::makeTorus` makes a whole torus only.
- **Booleans (K4)** clip every triangle of both operands through BSP
  trees whose splitting plane is the first polygon's own. A convex
  operand's tree is a chain as deep as its facet count, so the cost is
  O(n·depth). The only locality is the disjoint fast path. A balanced
  split was tried and was 8% slower (M11, Phase 142). `math::RTree` exists,
  and `DrawingProjection` has a file-local triangle BVH. There is no timed
  Boolean test; `tests/TimeLimits.h` gives the pattern for one.

## Phase 162: Hole and Mirror

In three PRs.

### 162a: mirrored bodies and the Mirror feature (as built)

- **A mirror keeps a solid facing out.**
  - `Mat4::reflection(point, normal)` and `Mat4::determinant3()`.
  - `cloneInto` (every `Pattern::transformed`/`append`), given a matrix
    that mirrors, reverses each loop as it copies it:
    - each half-edge starts at its old end, and its next and prev swap;
    - a vertex leaves by the half-edge that came into it;
    - an edge takes its old twin, so its curve still runs from its
      half-edge's origin;
    - each carrier and true surface is reversed in u
      (`NurbsSurface::reversedU`, moved from `StepFormat.cpp`), so a
      surface's normal is still its face's outward normal.
  - Tests mirror a box, a cylinder and a drilled block. Each keeps its
    signed volume and its carriers' sense, has its true surfaces facing
    out, and joins its original exactly. The three tests fail with the
    reversal switched off.
- **`MirrorFeature`**, about a point and normal, or a flat face of the
  part followed by its whole name (`planeFace`), as a sketch on a face is.
  - With no targets it mirrors the whole part and joins the image: a box
    in its own side is one solid twice its size, and still is when the
    box is made wider. In a plane clear of the part, the image is a
    second body.
  - With targets, each one's own body is mirrored and combined as the
    target combines, as `PatternFeature::executeIn` does. The image is
    named `child(featureID, 1)`.
  - A face that is gone or no longer flat fails the build: "the plane to
    mirror in: …".
- Model ▸ Mirror (ribbon "mirror-3d", with the patterns): the plane is a
  base plane or one of the part's flat faces (the one clicked, at
  first); the features are listed to mirror only them. The edit form
  shows the plane's point and normal and the face.
- **Files:** version 26, `"type": "mirror"`. A plane of nothing is a
  damaged feature: left out, and said.
- **A test chose the wrong face.** "facing (1, 0, 0)" matched the hole's
  wall before the box's side. The kernel was right: 1760 is the union of
  the part mirrored in the hole's wall. The test names the face by where
  it is, too.

### 162b: the Hole feature (as built)

- **`HoleFeature`**, a body cut from the part (`createsNewBody`, operation
  Cut), so the tree subtracts it, and a pattern or a mirror with it as a
  target repeats it.
  - It stores the face as a whole name (`face`), the point
    (`positionPoint`, projected onto the face where each build finds it),
    the type (simple, counterbore, countersink) and the extent (to the
    depth, through all, up to a face given by `upToFace`, parallel).
  - Sizes: `diameter`, `depth`, `boreDiameter`, `boreDepth`,
    `sinkDiameter`, `sinkAngle`, and `pointAngle` (118° by default; 0 is
    a flat bottom, and through all and up to a face are flat).
- **Built as a revolve.** A half-section is built in the plane through the
  axis, from a little above the face, and revolved a full turn into the
  part. `Revolve` gives a cylinder or cone to each line parallel or
  oblique to the axis.
  - The lines have fixed ids, and their faces and edges are renamed for
    what they are: `hole_1/wall`, `bore`, `boreFloor`, `sink`, `bottom`.
- **Exact against the faceted part.** Each type's volume matches its
  closed form over 32-gons.
  - A hole through a plate, and a counterbored one, go to STEP as
    designed: each wall is one cylinder, and the volume is exact.
  - So does the plate mirrored in its side, which checks 162a's surfaces
    through STEP.
  - A hole follows its face when the box grows, and a pattern repeats it.
- **Refused, and said:** no part before it; a face that is gone or not
  flat; a counterbore or countersink no wider, or no shallower, than the
  hole; an up-to face that is at a slant or not behind.
- **Model ▸ Hole** (in the ribbon's Features group). The face is the one
  clicked, else the first facing up. The position starts at its middle,
  and moves to the new face's middle when another face is chosen. The
  form offers only the sizes the type and extent use. The edit form shows
  every size, the point and both faces.
- **Files:** version 27, `"type": "hole"`. Its sizes are saved by name, and
  each is read back as an edit sets it. One the build refuses (a diameter
  of nothing) is a damaged feature: left out, and said.

### 162c: mirrored components (as built)

- **`ComponentInstance::mirrored`.** The part is mirrored in its own YZ
  plane (`ownMirror()`, S) and placed by `transform`, which stays rigid.
  A component at T mirrored in a world plane R is placed at R·T·S, the
  same for one already mirrored (which comes back unmirrored), since S is
  its own inverse.
  - `solid()` and `mesh()` give the mirrored solid and mesh. They are made
    from the part's the first time they are asked for, and again when the
    part's change (kept against the source's pointer). `ownSolid()` is the
    part's, unmirrored.
  - The mirrored solid goes through 162a's orientation-keeping clone. The
    mesh has its x negated, normals too, and triangles rewound.
  - Everything that reads a component's solid or mesh takes the mirror:
    the scene, bounds, mates, interference, and the merged mesh.
    `drawingSolid` is given parts as made, and mirrors a mirrored one's
    itself, since drawings read parts from their files.
  - A pattern's instances take their seed's mirror.
- **The bill of materials** lists a mirrored part as its own line ("block
  (mirrored)"). **STEP export** writes it as its own part, the mirror in
  its own frame, read back facing out.
- **Assembly ▸ Mirror Components**:
  - it works on the components checked (those chosen, at first);
  - the plane is YZ, ZX or XY through a point, at first the far corner of
    the chosen components;
  - it is one undo step;
  - a subassembly or a pattern's instance is refused, and the status bar
    says why. A mirrored subassembly would need a reflection in its
    children's STEP placements.
- **Files:** version 28, a component's `"mirrored"`.

## Phase 163: Shell, part 2 (as built)

- **The cavity is the part, each face moved.** `Shell::executeOffset`
  (`OffsetShell.cpp`) moves every face inward by the thickness, and each
  face to open outward by it, so the cavity comes out through the
  opening. The shell is the part less the cavity, by the Boolean.
  - The part keeps its faces, their names and their true surfaces. The
    cavity's faces are named `<shell>/inner:<face>` and lie on their
    offset ideals.
- **Each corner of the cavity** is where its faces' offset surfaces meet.
  They are grouped: the pieces of one plane are one surface, and the
  facets of one ideal are one surface.
  - A flat face's surface is its plane moved along its normal.
  - A cylinder or sphere is grown or shrunk by the thickness, by its
    sense: out of a boss, into a hole's wall.
  - A cone is slid along its axis by thickness / sin(half-angle).
  - Each corner is solved by Gauss–Newton with minimum-norm steps from
    where it is. So a corner on fewer than three surfaces (a cylinder's
    rim) keeps its place along the others, and stays on its radial line.
  - A corner whose offsets do not meet in a point (four faces at a
    pyramid's tip) is refused.
  - The offset ideals are exact: a rational circle's control points
    scale with it.
- **Refused, and said:**
  - a face on another kind of surface;
  - a face opened in part;
  - several bodies;
  - a face that is not there;
  - a wall too thick for the part: a face or hole of the cavity turns
    over or shrinks to nothing, an edge reverses, or a face's loop crosses
    itself (GeometryValidator on the cavity, before the cut). A fin
    thinner than two walls is caught this way.
  - Not caught: two faces of the cavity crossing each other past those
    local checks. No check in the kernel sees faces crossing yet. Review
    found the first version claimed the sewer's manifold check did, and it
    reads no coordinate.
- **Old files build as they did.** `ShellFeature::Method`: a new shell
  offsets (saved as `"method": "offset"`, format version 29). One read
  without it is built as the prism's, so its part and its names are
  unchanged.
- **Tests:**
  - a box opened at the top (424) and at the top and front (352);
  - a plate with a hole, keeping a wall round it, with the cavity's wall
    on a cylinder of r + 1;
  - a cylinder cup (A(5)·10 − A(4)·9) and a cone bowl;
  - refusals;
  - an offset shell keeping the box's names, with the cavity named after
    them;
  - the prism's names for the legacy method;
  - the round trip, with a pre-163 file building 14 faces as before;
  - the form, shelling a drilled box open at two faces.
- **Not done:** a vertex of four or more planes that do not meet (refused),
  tori and freeform faces (refused), several bodies, and a closed hollow
  (no open face).

## Phase 164: Fillets on curved faces (as built)

- **A revolve's flat end is one face.** In a full turn, stably named, a
  band square to the axis is one face (a disk, or a ring with its hole),
  not a facet per step. `ringstack` counts and turns holes too.
  - So a revolve's rim chain meets one face all round, as a cylinder's cap
    does.
  - That exposed a Boolean fault. `FragmentMerge` splits a merged face
    with a hole along a line through the hole's middle, and for an annulus
    that line cut chords of both circles mid-way. The counterbore floor's
    corners then left the bore's circle, and STEP wrote the bore in
    facets.
  - It now picks, per hole, the cut through the fewest chords of round
    loops (8 or more corners): a line through one of the hole's own
    corners. It keeps the old cut on a tie, so plain polygonal holes split
    as they did.
- **Fillet keeps a face's holes.** `FilletOp` walked only outer loops, and
  dropped a ring's hole ("open boundary"). Each loop is now rewritten the
  same way and built as an inner wire.
  - A solid of revolution's rim and a ring's outer rim both fillet,
    checked by Pappus.
- **A rim's exact section.** At a rim's mitered corners the miter plane
  holds the axis: it is the meridian plane. The section there is now the
  true fillet's, the ball touching both faces' lines in that plane. It
  used to be the chord's prism cut slantwise, an ellipse.
  - Consecutive sections are turns of one another. The bands stay flat,
    the part is a revolve in N steps, and every band corner is on the
    design torus (spine R − r, height h − r), to 1e-9.
  - `CylinderRimFillets` now checks against that closed form:
    (N/2)·sin(2π/N)·∫ρ² dz.
  - A radius-1.9 fillet on an 8-sided rim of radius 2 is now valid. It
    was refused only because a chord's prism ran backwards between the
    miters. A radius past the rim's own is still refused.
- **One torus per rim.** The bands' ideal is one torus through the exact
  sections' centres, matched by its geometry rather than by curve object.
  - The tangent lines (on the cap at R − r, on the side at h − r) record
    their circles.
  - `computeIdeal` measures the rounded cylinder with no parted edges,
    within 0.1% of Pappus.
- **STEP:** the pole check now asks whether the surface folds to a point:
  it compares the area element's size with the face's, not the angle of
  its tangents. A torus band's inside corners pass; a cone's apex still
  fails.
- **Not done:**
  - STEP export of a filleted rim. The torus band's seam is a chain of
    arc chords, and the writer closes a ring only along a single seam
    edge. It needs a synthesized arc edge. For now the rim goes out in
    facets, and says why.
  - A hole's rim. The Boolean splits the plate's top into pieces, so the
    rim chain changes faces at the seam. Keeping holes in Boolean output
    is wider work.
  - Cone rims have the same exact section, untested.
  - Chamfers on rims keep their prism sections.
  - A rim filleted in part (an open chain of its chords) fails with
    invalid geometry, as it did before this phase. `makeRimTorus` works
    from either mitered end, ready for when it can be.

## Phase 165: Booleans at scale (as built)

- **Where the time went.** Box less a 2,048-facet pin took 4.0 s in a
  Debug build (2.6 s optimised, per M11). Timing each stage showed the CSG
  itself took 1.7 s, and most of that was two tree builds:
  - the pin's own tree;
  - the last step, which merges the pin's surviving polygons into the
    box's tree and, at its leaf, builds them into a sub-tree.

  Each build split-tested every polygon against every plane before it, and
  allocated each time.
- **A convex set's tree in one pass.** When `build()` gives a new node a
  set of 32 or more polygons, `buildChain` checks it:
  - Each polygon, in order, joins the first plane (in order of first
    appearance) it lies in, as `splitPolygon` judges it, or begins the
    next.
  - The set is convex if no distinct point is in front of any of those
    planes.
  - If so, the tree is the chain `build()` would make: the same nodes,
    planes, polygons and order, so the results are identical. Otherwise
    nothing is made, and it is built as before.
  - The full suite, whose Boolean tests check exact volumes and names,
    passes unchanged.
- **The result:** the CSG went from 1.7 s to 0.4 s, and the whole Boolean
  from 4.0 s to 1.7 s (Debug). `AFinelyFacetedPinCutsQuickly` bounds it
  at 1 s optimised and 10 s Debug.
- **Not done:** clipping only what is near. A polygon far from the other
  operand still goes through the tree, and a non-convex operand's tree is
  still built by splitting. Drilling a plate that already has holes costs
  about 0.26 s more for each 256-facet hole already in it (Debug).
  - The plan's bounding-volume approach needs far polygons classified by a
    point-in-solid test, not by a tree of a partial surface, which
    misclassifies where it is open. That is a new Boolean core: pairwise
    intersection and classification, a mesh arrangement. It is left for a
    phase of its own.
  - The rest of this Boolean's time is FragmentMerge (0.6 s) and the
    sewer and names (0.3 s).

## Tracking

Each phase is its own PR, or one per part, with README rows and CHANGELOG
entries. A phase's section here is replaced by "as built" when it lands.
