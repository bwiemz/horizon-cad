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
- A plate with a hole cut through it as a part's features cut one (stably
  named): 9 faces, and the plate less the hole to 1e-9.

### Not yet
- **A hole cut by a Boolean named by position** (as older files are) is kept
  in facets.
  - Under stable naming, the Boolean puts each face's pieces back together
    (`mergeFragments`), and a part's hole goes out as designed.
  - By position, the cut's triangles stay. Cutting a triangle's diagonal
    leaves a vertex on each rim chord, inside the circle, and the plate's
    triangles share it.
  - The fix would be the same merge for positional names.
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

## Phase 152: STEP import of trimmed surfaces

### As built
- **Before this**, a curved face read from STEP was built on its surface only
  when its outline was a rectangle of the surface's (u, v). Any other
  outline, or any hole, became one flat facet, its outline. A cylinder cut
  on a slant, or one with a pocket in its side, measured wrong.
- **`loopInUV`**: any loop without a pole, unwrapped round the surface's
  seams, each half-edge placed where the one before ends. A loop that
  winds round the surface is refused.
- **`trimmedFacets`** cuts the region in (u, v) into triangles, scaled to the
  surface's own lengths in its middle:
  - Its holes are bridged to the outline. Each bridge goes to the nearest
    point it can see, crossing no edge of the outline, its hole or a hole
    still to join. The textbook choice, the far end of the edge a ray meets,
    ran across a seam to a far corner. Bridges are cut into points on the
    surface, since one straight chord across a curved face cuts through
    the part.
  - It is cut by ears. An ear is clipped only if no point is in it or on
    its edges: a point on a triangle's edge would be a vertex in the
    middle of its neighbour's edge.
  - It is made Delaunay by flips; the outline is never flipped. Ears leave
    long diagonals where no point goes in near them.
  - Points go in on a grid:
    - as close as the surface turns by `maxAngle` across the region each
      way (`piecesAcross`), and no further apart than the median of the
      outline's own (its average was stretched by a long straight seam,
      and the facets fell 1.4 % short);
    - only within the region: each column keeps the points where a line
      up it has crossed the outline and holes an odd number of times (a
      point in a hole was looked for in every triangle);
    - kept clear of the outline, each checked against the edges listed in
      its grid cell;
    - put in coarse to fine, every 2^k-th corner before those between, so
      each goes in among points about as far apart and changes a few
      triangles (in columns, each went in beside the unfilled region);
    - each located by walking from the last point's triangle across inner
      edges, else from the point beside it's; one on an edge cuts the
      triangles either side in two; then flipped Delaunay.
  - Its outline points are the edges' own, so the facets meet the faces
    beside them.
- **Still one facet:** a pole on a trimmed outline, and a hole across a seam.

### Measured
- A cylinder cut on the plane z = 1 + x/2, its side topped by an ellipse:
  nothing outlined, the facets within 1 %, and the ideal volume π to 1e-8.
- A cylinder of radius 2 with a pocket cut into its side (a face with a
  hole): nothing outlined, the facets within 1 %, the ideal volume
  16π − 0.525 to 1e-8, in half a second.
- A pocket over a third of the side, cut at 0.01 rad (55,000 points): 50 s
  before the points in the hole were left out and put in coarse to fine;
  under 5 s after (debug build), the same facets.


## Phase 153: STEP assemblies

### As built
- **Before this**, a STEP assembly came in with every part once, where it
  was drawn: a part used four times was one solid, and none was where the
  assembly put it. An assembly could not be exported.
- **`readStructure`** walks a file's product structure:
  - Each PRODUCT_DEFINITION's shape is its SHAPE_DEFINITION_REPRESENTATION's
    representation, and those related to it without a transformation (OCC
    relates a part's SHAPE_REPRESENTATION to the ADVANCED_BREP one holding
    its solids). A definition with solids is a part.
  - Each NEXT_ASSEMBLY_USAGE_OCCURRENCE places its part by the
    ITEM_DEFINED_TRANSFORMATION of its CONTEXT_DEPENDENT_SHAPE_REPRESENTATION:
    A(item2)·A(item1)⁻¹ takes rep_1's coordinates into rep_2's, each item's
    origin in millimetres by its own representation's length unit; turned
    round when the part is rep_2, as some writers have it.
  - From each top assembly down, placements compound. A loop is followed
    once; the walk stops at 20,000 placements or 1,000,000 assemblies, and
    says so. A use with no placement is placed where drawn, and said.
  - A file whose structure places nothing reads exactly as before, names
    and order alike.
- **`fromString`** returns each placement of each part. A solid's first
  keeps its names; each copy after it is named as a solid of its own, so a
  face of one bracket is not taken for another's.
- **`assemblyFromString`** returns the parts once each and their placements,
  named by the uses down to them ("Sub:1/Bolt:2").
- **`assemblyToString`** writes a part product for each part (its solids and
  an origin) and an assembly product whose uses place them the same way. A
  placement that is not rigid, or of no part, is left out and said.
- **Names** travel as Part-21 text: quotes and backslashes doubled, all else
  `\X2\`/`\X4\` escaped; those and `\X\`, `\S\` read back.
- **The app:**
  - File ▸ Import ▸ STEP as a New Part places each part.
  - File ▸ Import ▸ STEP as an Assembly (`saveStepAssembly`) writes each part
    as a part file, in "<assembly> parts" beside the assembly, never over
    another file, and opens the assembly.
  - File ▸ Export ▸ STEP from an assembly tab writes a STEP assembly, each
    part once from its file; components whose parts cannot be read are
    listed.
- **Not read:** placements by MAPPED_ITEM (the other way AP214 allows).

### Measured
- A nested fixture (an assembly within the assembly, in centimetres; a part
  written OCC's way; a placement written the other way round; an escaped
  name): each tetrahedron's centroid where the assemblies together put it,
  to 1e-9.
- An assembly written and read back: its parts, names and placements to
  1e-9; exported from an assembly tab and imported as files, the same.

## Tracking

Each phase is its own PR, with README rows and CHANGELOG entries. A phase's
section here is replaced by "as built" when it lands.
