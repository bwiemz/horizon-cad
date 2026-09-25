# Milestone 15 — Curved geometry that travels (Phases 151–153)

Roadmap: [2026-09-25-professional-workflows-roadmap.md](../specs/2026-09-25-professional-workflows-roadmap.md).

## Baseline (checked in the code)

- **`StepFormat::toString` writes every face on its own `f.surface`**
  (`StepFormat.cpp:331`, `writeSolid`), with every edge on its own `e.curve`.
  - A faceted curved face is many faces, each on a flat facet plane.
  - A rim is many straight chords.
  - A 32-facet cylinder reaches another CAD system as a 34-face prism. It
    does not read it back as a cylinder either: its facets record the
    cylinder (`Face::analyticSurface`), but the writer never looks.
- What each facet and chord records:
  - Every facet records its ideal surface. A primitive's facets share one
    `analyticSurface` object; Extrude's and Revolve's do too.
  - Every chord records its ideal curve. A primitive's rim chords each record
    their own arc; Extrude's and Revolve's record one shared whole circle.
    Phase 147's `spanBetween` finds a chord's span on either.
  - Stable names say the same (Phase 139): a facet is `<face>/facet:k`, a
    chord `<edge>/chord:k`.

## Phase 151: STEP export as designed

### As built
- **One face per curved face.**
  - `planCurvedFaces` joins the facets that share an ideal surface over
    their shared edges.
  - Each group is written as one `ADVANCED_FACE` on that surface, as its
    rational B-spline.
  - `same_sense` compares the facets' normals with the surface's normal at
    the nearest point. Facets that face both ways are refused.
- **Its outline.**
  - The group's outline is walked into loops: half-edges whose other side
    is outside the group, going round each vertex over the group's own
    edges.
  - Each outline edge must lie on the surface:
    - a **circle**, written as a STEP `CIRCLE`, each with its axis turned so
      that the short way from the edge's start to its end runs
      anticlockwise (the reader trims a circle between an edge's vertices,
      but reads a B-spline curve whole);
    - a curve of the edge's own span;
    - a straight edge along the surface (a ruling);
    - a **rim** a Boolean left without its circle: where the cylinder meets a
      plane square to its axis, the circle through the rim's own corners,
      every corner checked against it. The cylinder's fitted centre is 5e-6
      off, so it is used only to check.
- **A seam.**
  - A group with two outlines (a cylinder's side) is cut along a joining
    edge that lies on the surface: a ruling, not a triangle's diagonal.
  - The seam is used once each way in one loop, as other systems and our
    reader expect.
- **The face across must agree.** An outline edge written as a curve must
  suit the face across it too: another face written as designed, or a
  plane the curve lies in (a cap). This is repeated until nothing changes.
- **Refused, and said.** Faces kept as facets are listed in
  `StepWriteReport`, and File ▸ Export ▸ STEP shows the list:
  - closed all round (a sphere, a torus);
  - coming to a point inside it (a cone's apex, a pole);
  - more than two outlines;
  - an outline off the surface;
  - facets facing both ways;
  - a face beside it kept in facets.
- **As modelled** (`StepWriteOptions{asDesigned = false}`) writes every facet
  as before, which reads back exactly.
- **Found on the way.** `Pattern::transformed`, and so every moved,
  patterned or assembled part, moved each facet's ideal surface separately.
  A moved cylinder's side was then 32 surfaces to everything that asks
  which facets are one (drawings, this export). `cloneInto` now moves each
  shared ideal once.

### Measured
- A cylinder: 3 faces, 64 circle edges. Read back, its ideal volume is πr²h
  to 1e-9, where it was a 34-face prism.
- A part's stably named cylinder, and one moved, are the same.
- A block with a boss joined on: exact, with the boss's foot circle
  recovered.

### Not yet
- **A hole cut by a Boolean** (and a blind hole) is kept in facets. The
  cutter's side is in triangles, and cutting a triangle's diagonal leaves a
  vertex on each rim chord, inside the circle. The plate's triangles share
  those vertices, so no one circle goes through the rim. The fix belongs in
  the Boolean: merge coplanar faces, and drop the in-line vertices.
- Spheres, tori and cones with an apex are kept in facets: a face with a
  pole needs a degenerate edge, or a split into two faces.

### Tests
- `StepCurvedTest`:
  - a cylinder as designed (faces, surface, circles, exact volume and
    area);
  - a part's cylinder and a moved one;
  - a boss on a block;
  - a sphere, a cone and a cut hole kept in facets with their reasons,
    each read back as the same solid; a box unchanged.
- `PatternTest`: a moved part's facets still share one surface.
- `ImportExportTest`: exporting a sphere part says which faces went out as
  facets; a box says nothing.
- The two tests of the exact faceted round trip ask for it
  (`asDesigned = false`).

## Phase 152: STEP import of trimmed surfaces (outline)
- A curved face with any boundary is built on its surface, trimmed in (u, v).
  Today only a (u, v) rectangle is (`asRectangle`); any other face becomes
  one flat facet.

## Phase 153: STEP assemblies (outline)
- `PRODUCT` / `NEXT_ASSEMBLY_USAGE_OCCURRENCE` read into an assembly of parts,
  and written from one.

## Tracking

Each phase is its own PR, with README rows and CHANGELOG entries. A phase's
section here is replaced by "as built" when it lands.
