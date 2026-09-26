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

### 162c: mirrored components

- **A mirrored component is its part mirrored in its own frame**
  (`ComponentInstance::mirrored`: the part reflected in its YZ plane),
  placed by a rigid transform. A reflection R about a world plane of a
  component at T is the rigid `R·T·S` of the part mirrored by `S`, the
  local reflection. So every placement stays rigid, and the mate solver,
  drags and exports keep working.
  - Its mesh is mirrored (x negated, triangles rewound, normals
    reflected), and its solid by the orientation-keeping reflection of
    162a.
  - The bill of materials lists it as its own line ("bracket, mirrored"),
    and STEP export writes the mirrored part once, as its own product.
- Assembly ▸ Mirror Components: the components checked, about a base
  plane, a datum plane or a component's flat face. One undo step.
- Files: a component's `"mirrored"`.

## Phase 163: Shell, part 2

- **Offset each face, not the profile.** Each face kept is moved inward by
  the thickness along its own plane: a flat face by `planeOf`, a facet of
  a curved ideal by its ideal (a cylinder's or a cone's radius less the
  thickness, from `frameForFace`).
  - Each inner vertex is where its faces' offset planes meet (three
    planes, `Mat3::inverse`; more, least squares). The inner faces are
    the offset loops, sewn (`SolidSewer`) with the outer shell less its
    open faces, and the rims between them.
  - The input's faces keep their names, surfaces and ideals. The inner
    faces are named after them (`/inner`), with ideals of the offset
    radius.
- **What it takes:** any number of open faces, on a single body whose
  vertices each join three faces (boxes, prisms, bosses, holes through
  or blind).
- **What it refuses, and says:** a vertex of four or more faces whose
  offset planes do not meet in a point; a wall that collapses (an inner
  edge reversing, found as now); a curved face that is not a cylinder or
  cone; several bodies.
- **Tests:**
  - a box opened at the top and front: 1000 − 8·9·9 = 352;
  - a plate with a hole through it, genus kept;
  - a boss on a block;
  - a cylinder cup, whose inner wall is a cylinder of r − t;
  - the old prism tests, as the parity net.

## Phase 164: Fillets on curved faces

- **A rim's bands are one torus.** When a filleted chain runs round a
  circle (a cylinder's rim, a revolve's rim, a hole's edge), each band's
  ideal is one shared torus (`makeTorus` about the circle's axis, ring
  radius R ∓ r, tube r) instead of a ruled patch per chord. Then STEP
  writes it as one face, and `computeIdeal` measures it exactly (Pappus).
- **A revolve's rim fillets.** The miter check is generalised for facets
  on both sides: a chain whose two faces at each vertex are each a facet
  of one ideal, taken as one face, and the turn measured between those
  ideals' normals.
- **Plane–cone** rims, by the same path, with the band's torus about the
  cone's axis.
- **Chamfer** of the same rims gives each band's ideal cone.
- **Tests:**
  - a revolve's rim filleted by name, checked by its Pappus volume;
  - a cylinder's rim fillet recording one torus, with `computeIdeal`'s
    volume to 1e-9;
  - a cone's rim;
  - STEP export of a filleted cylinder: one toroidal face;
  - `FilletCylinderToPlaneEdge` made to assert success.

## Phase 165: Booleans at scale

- **Clip only what is near.** A triangle of one operand whose box does
  not meet the other operand's box is outside it, and is kept or dropped
  by the operation without going through the tree (for Union and
  Subtract A kept, for Intersect dropped; B the other way for Subtract).
  Of the rest, only those near the other's triangles, found through a
  box tree of them, are clipped; the others are classified by one point
  against the other solid (a ray cast through the same tree).
- **A tree that is not a chain** where it is still needed: split planes
  chosen from a sample, not always the first polygon's.
- **Fewer allocations:** `splitPolygon`'s types in a reused buffer;
  `inheritEdgeIdeals` through a hash of the source segments.
- **Tests:**
  - a box less a 2,048-facet pin, exact, within an `HZ_TIME_LIMITS`
    bound;
  - a small box overlapping part of a 512- and a 2,048-facet cylinder,
    whose time ratio stays well below 16;
  - `FacesInExactContactAreSound` and the robustness suite unchanged, as
    the net.

## Tracking

Each phase is its own PR, or one per part, with README rows and CHANGELOG
entries. A phase's section here is replaced by "as built" when it lands.
