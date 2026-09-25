# Milestone 11 — Kernel depth (Phases 139–142)

The roadmap (`docs/superpowers/specs/2026-09-24-product-completeness-roadmap.md`,
§1.5) lists the kernel's known limits:
- Fillet and chamfer take only 90° convex edges between planar faces.
- Names of edges after a fillet, and of Revolve, Loft, Sweep and Chamfer
  faces, follow storage order.
- Curved faces are measured and cut as vertex polygons.
- Tolerances are fixed absolute values.

This milestone takes them on. Each phase measures before and after, and
refuses what it still cannot do (Principle 7).

A survey of the kernel on 2026-09-24 set the baseline below. It also
corrected two records:
- The ChamferOp volume defect was fixed in Phase 82.
- Booleans take torus and revolve operands since Phase 84's faceting.

**Old files keep working.** A feature records its naming scheme. A file
without one loads as `Positional` (NativeFormat.cpp), so every new name below
applies to `Stable` features only, which is what new features are. A saved
`"box/edge0"` still finds `box/edge0`.

## Phase 139: Stable names

### As built

- **A new naming scheme, `Stable` (3), for new features.** Documents saved
  with FromGeometry (2) or none (Positional) keep exactly the names their
  references were made against.
  - The file format is now version 19: an older build refuses a document
    with naming 3, rather than build it under names nothing refers to.
  - `namesFromGeometry` covers 2 and 3 wherever geometric naming applies.
- **Primitives are named after their feature** (`scopeToFeature`):
  `primitive_3/top`, where every box's top was `box/top`. Their edges are
  named from their faces, not storage order.
- **Fillet and chamfer keep the names of edges they did not touch**
  (`keepEdgeNames`).
  - An edge between two faces that exactly one input edge joined keeps that
    edge's name. The rest are named from their faces.
  - A second fillet on an edge named before the first now finds it.
- **Shell's names are scoped to its feature**, not every shell's
  `shell/…`. Its outer faces are still renamed by position (see Not done).
- **Revolve, Sweep and Loft name each face after the profile element it
  comes from**, in facets:
  - `revolved:<source>/facet:<step>`;
  - `swept:<source>/facet:<path segment>`;
  - `lofted:<source>/facet:<level>`, from the first section.
  A ring turned round, in Sweep and Loft, is mapped back to the profile's
  own order. The names hold when the facet count changes.
- **A curve is one logical edge, and a curved face one logical face.**
  - Facets of one ideal surface are `<face>/facet:<k>`
    (`nameFacetsLogically` for primitives).
  - Edges are named logically (`nameEdgesLogically`): chords of one curve
    are `<edge>/chord:<k>`, and seams between facets of one face are
    `<face>/seam:<k>`.
  - `logicalFace` and `logicalEdge` remove only the facet or chord
    component, so a pattern copy's curve stays its own (`…/pattern:1`).
  - Fillet, chamfer and mate lookups prefer a name's own chords or facets
    over its descendants (a copy's).
- **The viewport picks and highlights the whole curve or curved face.** The
  fillet, chamfer and shell lists show one row per curve or face, and no
  seams.
- **A pattern whose copies meet joins them at Stable names.** It joined them
  by position, which lost every name.
- **After review:**
  - A fillet's or chamfer's faces are one face for each curve they round
    (`nameBlendFaces`), `<feature>/fillet/<curve>/facet:<k>`. Named after
    each chord and band, a rounded cylinder rim was 256 faces. The edges
    between them carried the chord's `/chord:`, and `logicalEdge` then cut
    it from the wrong place: some names covered two or three edges.
  - A piece of a facet that a Boolean split belongs to its face,
    `<face>/facet:<k>/piece:<n>` as well as
    `<face>/facet:<k>/pattern:<n>/piece:<j>`. The pieces were grouped by piece
    number across facets, so a cylinder cut in two was two faces, each made
    of halves from both sides.
  - An older loft's twisted level has always been cut into triangles named
    `<side>/facet:<k>`, in every scheme. The UI now groups them as their
    side. That is kept: each triangle carries the side's ruled patch as its
    ideal, so a mate on the side finds the same frame. Renaming them would
    orphan references saved in older files.

### Tests

15 new, 1 changed:
- Integration (PersistentNaming):
  - two primitives named apart, with a fillet on the second by name;
  - an older (positional) box keeps `box/top`;
  - each scheme round-trips through a file;
  - fillet and chamfer keep untouched edge names, and a second one finds
    its edge;
  - a shell named after its feature;
  - a cylinder rim filleted whole by one name, its volume checked by
    Pappus's theorem;
  - revolve faces named after their profile, stable across the facet count;
  - sweep and loft named after their profile, including a turned ring;
  - a pattern copy's curves are its own;
  - a rounded and a chamfered rim are one face, with one curve each side;
  - a cylinder cut in two is one side, and so is a pattern copy's;
  - an older loft's twisted side is one face, one ideal.
- Window: a click on a cylinder's rim picks the whole curve, and Fillet
  rounds all of it.
- Changed: the sides-by-source test now checks both schemes' names.

### Not done

- A shell's outer faces are renamed by position (`outer_wall_<i>`), not
  kept from its input: it rebuilds the part as a ring stack.
- A flat face a Boolean splits into separate regions keeps a name for each,
  `<face>/piece:<k>`, numbered in storage order.
- Vertices are still unnamed.

## Phase 140: Fillet and chamfer at any angle

### As built

- **A fillet at any angle, convex or concave** (`computeFilletFrame`).
  - The ball rolls in the wedge between the faces' in-face directions: in
    the material for a convex edge, on the empty side for a concave one.
  - It touches each face a setback r·cot(θ/2) from the edge. Its centre is
    one radius off faceA into the wedge. The arc spans π − θ, with middle
    weight sin(θ/2), and its middle control point is on the edge.
  - A concave blend adds material, through the same construction.
  - At a convex right angle every formula reduces to the old one.
  - Faces within a degree of continuing one another, or of a knife edge,
    are refused.
  - The capacity check compares the setback, not the radius.
- **A blend's end lies on its end face**, square to the edge or not. The end
  section is carried along the edge onto the end face's plane, as a miter
  carries it onto the bisector. The bands stay planar, because each lies
  between two rulings parallel to the edge.
- **Fillets on parts of several bodies** (`perBody`).
  - Each body with edges to round is filleted alone, and the bodies are
    collected again with their names.
  - Every face was put in one shell, and Euler's check failed.
  - After review: the second and later bodies rounded name what they make
    under `<feature>/body:<k>`. Each rebuild numbers its corner blends and
    new edges from 0, so two bodies had one `<feature>/blend/corner:0`.
- **Chamfer.**
  - At any angle it needed no change: the plane clip is general. Its header
    said otherwise.
  - Its end on an oblique end face was not manifold. The chamfer face's
    corners are now carried onto the end face, where the clipped side faces
    put theirs.
  - Multi-body chamfers already worked, since the sewer makes a shell of
    each piece.
- **Refused by name:**
  - "A corner blend of three fillets needs square, convex edges";
  - "A chain of fillets across corners of different angles".

### Tests

8 new, 2 replaced. The volumes are exact to 1e-9, against closed forms:
- the kite less the n-chord sector for a fillet, and the triangle for a
  chamfer;
- the swept section less twice its first moment, for an end on 45° faces.

Cases:
- A fillet on a 45° edge, and on all three edges of a triangular prism
  (45°, 45°, 90°).
- A fillet ending on 45° end faces.
- A concave fillet that adds material.
- One body of two filleted, and two bodies filleted at once, named apart.
- A chamfer on a 45° edge.
- A concave chamfer.
- A chamfer ending on 45° end faces.
- One body of two chamfered.
- Replaced: `NonOrthogonalDihedralRefused` and `ConcaveEdgeRefused`, whose
  refusals are lifted.

### Not done

- A chain of chords that share no face, both sides faceted (a revolve's
  rim), is refused. It needs a miter at a vertex of four edges.
- An oblique or concave three-edge corner blend, and a chain across corners
  of different angles, are refused by name.
- Faces with holes, which only STEP import makes, still lose their inner
  loops in both ops.
- A tolerance-driven chord count assumes a quarter arc. An acute corner's
  arc is longer, so it meets the tolerance less closely.

## Phase 141: Curved faces measured as curved

### Baseline
- Mass properties integrate the vertex polygons (MassProperties.cpp),
  never the ideal surface. A cylinder of 32 facets reads 0.6 % low.
- No "ideal" or "as modelled" report exists.
- STEP import keeps no ideals. It reads cones, spheres and tori as
  failures. An OCC-style cylinder (V=2, E=3, F=3) has one-vertex cap loops,
  which the boundary mesh skips, so its volume and Booleans are wrong.

### As built
- **Ideal properties** (`MassPropertiesCalculator::computeIdeal`).
  - Each edge is cut into m pieces on its ideal curve, or where its faces'
    ideals meet (projection onto each in turn). Each face with a curved
    ideal is triangulated, each triangle cut into m², and its points put on
    the surface. Flat faces keep their plane. Shared edge points keep the
    refined boundary closed.
  - Measures at m = 1, 2, 4, ... are extrapolated (Romberg) until the last
    correction is under the tolerance (1e-10 by default). The estimate was
    within a factor of 3 of the true error for a torus, sphere and cone.
  - `exact` holds unless a face without an ideal bends by under 30° from a
    neighbour without one (such faces are named, and measured as modelled),
    or two ideals part at an edge (counted; refinement stops at m = 8).
  - Three things made the sequence settle in powers of 1/m²:
    - Curve points are nearest to the chord's points, as face points are.
      Points evenly spaced in a full circle's parameter left an O(1/m)
      error.
    - A triangle at an apex or pole is cut as a quad collapsed there, not
      barycentrically: log(m)/m².
    - No search starts at, or steps onto, a pole.
      `NurbsSurface::project` halves a step that would leave the domain;
      `locate` starts from cell middles.
- **NurbsSurface:**
  - `evaluateWithDerivatives`: exact first derivatives, no allocation;
  - `closedU`/`closedV`;
  - `project`: seeded Gauss–Newton, round seams;
  - `locate`: a fast global search. The old `closestPoint`, with numerical
    derivatives and allocation, took 25 s for a sphere's vertices.
- **The dialog reports both**, as modelled and ideal, and whether the ideal
  is exact. Parts with 100 or more curved faces are measured on a worker
  (in Auto); closing the dialog stops it. After review: a cancel is seen
  along every edge and every row of every triangle; between faces alone,
  closing the window waited for a large face to finish.
- **STEP curved faces** (`model::facetCurved`, applied when an imported
  body is built, so the file's exact B-Rep is what a document saves):
  - Edges are cut into equal chords by turning (32 per circle), each
    recording its curve.
  - Flat faces keep their holes (`SolidSewer` takes inner loops now).
  - A curved face whose outer loop, mapped into (u, v), is a rectangle is a
    Coons grid of facets. The mapping handles seams, including one met
    twice, and poles or an apex, where the side is degenerate. The face's
    loop orientation sets the facets' orientation. A face that is all of its
    surface (a sphere, closing both ways round) is taken along the
    surface's normal.
  - Any other curved face is one facet, its outline, and the import report
    says so.
  - Cone, sphere (in its placement's frame, turned to face out) and torus
    surfaces are read. Plane angle units are read, so a cone's semi-angle
    in degrees is right.

### Tests
- Ideal (modeling): cylinder, cone, sphere, torus, revolve ring and
  filleted box against closed forms (volume, area, centroid, inertia) to
  1e-9. A part with nothing curved is as modelled. Facets without ideals
  are named, and parted ideals counted.
- NurbsSurface: exact derivatives against differences; closedness;
  projection to the foot, round a seam, beside a pole, onto a seam.
- STEP: an OCC-style cylinder, cone, sphere and torus, a cone in degrees,
  and a plate with a bore, each right as modelled and exact ideally. An
  imported body is built in facets named for their face.
- UI: a cylinder's dialog shows both values; on a worker the ideal comes in
  when measured, and closing the dialog stops it.

### Not done
- A curved face trimmed other than by a rectangle of its (u, v), such as a
  cylinder cut by a slanted plane, is one facet, its outline. That needs a
  constrained triangulation in (u, v).
- No test file has a B-spline face. They go through the same rule, but
  only the analytic surfaces are exercised.
- Export writes the facets, not the curved faces they approximate.

## Phase 142: Boolean robustness

### Baseline
- The BSP is fully recursive: build, clip, invert, allPolygons and the
  node destructor. Its splitting plane is the first polygon's, so its depth
  is O(triangles) for prismatic solids (MeshCsg.h:43-49).
- Tolerances are absolute and inconsistent:
  - `kCsgPlaneEps = 1e-6`;
  - the weld tolerance, 1e-7;
  - the T-junction tolerance, 8 × the weld tolerance;
  - FragmentMerge gives up past 60 holes;
  - only FeatureTree's `solidProblem` is relative to size.
- Robustness tests pass at thresholds: 5 of 20 random transforms, 3 of 10
  fuzzed Booleans. Nothing tests far-from-origin coordinates, very small or
  large scales, exact face contact, or deep BSPs.

### As built
- **Held to strict terms, Booleans failed three ways**, all now fixed. The
  new tests require every case to give a valid solid that conserves volume,
  or to refuse with a reason.
  - **A million out, a box came back inside out.**
    `extractFacePolygons` judged orientation by a signed volume summed about
    the origin. A triple product's rounding grows as the cube of the
    distance; there it was larger than the box, so the sign was chance.
    Subtract returned the intersection. Every such sum is now taken about a
    point of the solid's own: BoundaryMesh, RingStack, PrimitiveFactory,
    FilletOp, MeshCsg and topology's `signedVolume`. Newell normals are
    likewise (MeshCsg, FragmentMerge, SolidSewer, BoundaryMesh,
    `loopNormal`). About the origin, a triangle there tilted by 2e-10 rad,
    which moved its plane 2e-4.
  - **Far out, a face's fragments did not merge.** FragmentMerge grouped
    them by plane offset n·p. Two normals' rounding times a lever of 2.7e6 is
    0.016. It now measures a fragment's distance from the plane at a point
    of the plane.
  - **A part a hundredth of a millimetre across did not sew.** The
    tolerances were absolute (1e-6). `CsgTolerance` takes them from the
    operands: 1e-8 of their extent, never under 64 ulps of their largest
    coordinate. The one value serves as the BSP's on-plane band,
    FragmentMerge's tolerance and the sewer's weld.
- **An iterative BSP.** The tree is a flat node array, and every traversal
  is a loop over an explicit stack. Its output is bit-identical to the
  recursive one's across 75 mixed Booleans. A Boolean whose tree is 1,024
  deep runs on a 128 KB stack; the recursive version crashed there.
- **FragmentMerge cuts through any number of holes.** A bit per hole in as
  many words as it takes. It gave up past 60, leaving a drilled plate in
  fragments.
- **The balanced split was tried and left out.** A plane chosen from 16
  candidates, to split least and balance most, was 8% slower on a
  2,048-facet cylinder and no faster on a plate of 81 pins. A convex
  solid's own planes each have all its other faces behind them, so its tree
  is a chain whatever the choice.
- **The validators already had a relative tolerance.** The feature tree's
  gate uses 1e-9 of the part's size, with a floor of 1e-7.

### Tests
- `BooleanRobustness`:
  - 20 random rigid placements of a cylinder against a box, all three
    types;
  - the same a million millimetres out;
  - the same at 1e-3 and 1e5 scale;
  - face-to-face contact, contact over part of a face, a face shared in
    part, and flush faces;
  - a plate cut by 81 pins at once;
  - a 1,024-deep tree on a 128 KB stack.
- The old threshold tests are strict: all 20 random transforms, and all
  18 random overlaps. They asked for 5, and for 1.

### Not done
- Building the tree of a convex solid is quadratic in its facet count: 2.6
  s for a 2,048-facet cylinder in a debug build. A tree that is not a chain
  needs planes other than the polygons' own, and then leaves that record
  inside or out.
- Solids of 100,000 triangles were not tested, as the plan asked. They run
  but take minutes, for the reason above.

## Tracking

Each phase is its own PR, stacked as before, with README rows and CHANGELOG
entries. The phase's section here is replaced by "as built" when it lands.
