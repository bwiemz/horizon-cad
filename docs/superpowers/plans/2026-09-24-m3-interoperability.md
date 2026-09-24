# Milestone 3 — Interoperability & 2D fidelity (Phases 107–110): Implementation Plan

Roadmap: [2026-09-23-production-readiness-roadmap.md](../specs/2026-09-23-production-readiness-roadmap.md)

## Goal

A user can get work in and out: import a STEP part or a DXF drawing, export
STEP, STL, glTF or DXF, and be told plainly what a file held that was left
out or approximated. The 2D tools behave at every zoom.

## Phase 107 — Import/Export reachable

### What the audit found

- The File menu has Open and Save As, which reach the native formats and
  DXF by extension. **STEP import and export, glTF export and BOM export
  exist in code, but nothing in the window reaches them**, and there is no
  STL export at all.
- An imported solid has nowhere to live. A part is its feature tree, and no
  feature holds a solid that did not come from a sketch.
- Since Phase 100, loading a file skips a malformed item instead of failing
  the whole file. That skip is silent, so the next save drops the item for
  good without the user knowing it was there. DXF import skips entity types
  it does not read, also silently.

### Design

- **`io::ImportReport`**: what a load left out (`skipped`) or changed
  (`approximated`), in the user's terms, plus a one-line summary. Filled by:
  - `NativeFormat` load: each skipped entity, layer or feature, with its
    reason;
  - `DxfFormat` load: unread entity types, counted by type;
  - STEP import: solids that could not be read.
- **`doc::ImportedBodyFeature`**: a feature holding a solid brought in from
  a file. It builds a new body like any other and takes the usual body
  operation.
  - It is saved as embedded STEP text, so a part keeps what it imported
    without the original file.
- **`io::StlExport`**: binary STL of a part's tessellation.
- **File ▸ Import:**
  - STEP… opens a new part with one imported body per solid.
  - DXF… adds the drawing's entities to the active drawing, as one
    undoable step.
- **File ▸ Export:** STEP…, STL… and glTF… for a part, DXF… for a drawing.
  Each item is enabled only when it applies.
- **The report is shown.** After an open or import that left anything out,
  a dialog lists it before the user can save over the file.

### Tests

- Native load of a file with a malformed entity and a malformed feature: the
  load succeeds, and the report names both.
- DXF with unread entity types: the report counts them by type.
- STL: header, triangle count, and unit normals; the volume from the
  triangles equals the part's.
- An imported body round-trips through save and load, keeps its volume, and
  takes Cut / Join.
- Window (offscreen):
  - Export STL / STEP / glTF through the menu writes the file.
  - Import STEP makes a part with the solid.
  - Import DXF adds its entities and one undo removes them.
  - Opening a file with a malformed item shows the report.

**As built.**
- **What is reported:** `ImportReport` names each skipped item as "<kind> <n> (<type>): <why>".
  - A feature whose sketch is gone now fails with that reason. It used to vanish without one.
  - A DXF load counts what it did not read, by type, including inside blocks.
- **When it is shown:** the report dialog appears after Open (native and DXF) and after Import DXF. It does not appear when an assembly resolves its components.
- **`ImportedBodyFeature`:**
  - Names its faces `face:<k>` in the imported solid's order (which never changes) and its edges after its faces.
  - Saved as `"type": "imported"` with the STEP text inline.
- **`StlExport`** writes little-endian binary STL through the atomic writer. Its header does not begin with "solid".
- **Export** items are enabled when the menu opens: STEP, STL and glTF for a part with a body; DXF for a document with something drawn.

## Phase 108 — DXF fidelity

LWPOLYLINE bulges (import and export), OCS extrusion, partial ELLIPSE,
non-uniform and mirrored INSERT scale, nested INSERTs, POLYLINE/VERTEX,
POINT, MTEXT chunk order and `\P` / `%%` codes, the full ACI colour table,
`$INSUNITS`, `$DWGCODEPAGE` → UTF-8, and escaped string output. Each
entity gets a DXF fixture authored to the spec. Anything still unread goes
into the 107 report.

This is split in two: **108a** for geometry and **108b** for text and
metadata (MTEXT codes, `%%` codes, the ACI table, `$INSUNITS`,
`$DWGCODEPAGE`, escaped output).

**108a as built.** Import now reads every section's raw entities before it
builds any of them, so a POLYLINE can take its VERTEX entities and a block
can insert one defined after it.
- **Polyline arcs.** An LWPOLYLINE or POLYLINE segment with a bulge comes
  in as an arc. The polyline's pieces are kept together as one group, since
  a polyline here has straight segments only; it is reported as
  approximated.
- **The old POLYLINE form.** POLYLINE / VERTEX / SEQEND is read. A 3D
  polyline or mesh is reported as not read, not flattened.
- **Object coordinate systems.** An extrusion of (0, 0, −1) is mirrored
  exactly for CIRCLE, ARC, LWPOLYLINE, POLYLINE, HATCH and INSERT.
  - TEXT keeps its position mirrored but cannot be drawn mirrored; this is
    reported.
  - Any other extrusion is out of the drawing's plane and is reported.
- **Partial ELLIPSE** (41/42) becomes a polyline on the curve, which is
  reported. A whole one stays an ellipse.
- **INSERT scales.** Unequal scales used to be averaged, so (2, 1) drew at
  1.5, and a negative one lost its mirror. Such an insert is now placed piece
  by piece:
  - exactly for a similarity;
  - for unequal scales, lines, polylines, splines and hatches stay exact and
    a circle becomes an exact ellipse, while arcs and ellipses become
    polylines and text keeps its proportions.

  Pieces on layer 0 take the insert's layer and, when they have no colour
  of their own, its colour. Pieces on a named layer keep that layer's
  colour.
- **ELLIPSE** centre and major axis are in world coordinates, as the DXF
  reference defines them; a reversed extrusion only reverses the direction
  its parameter runs.
- **Nested INSERTs** are flattened into their block. A block that contains
  itself is refused.
- **POINT** is reported as not read: there is no point entity yet.
- **Found while testing:** `DraftArc::mirror` read its end points after
  moving its centre. Every arc mirrored in an axis that does not pass
  through its centre came out wrong, and the 2D Mirror tool is among the
  callers. Fixed, with the drafting module's first transform tests.

**108b as built.**
- **Text.**
  - *MTEXT:* chunks (3) are now joined in order; they used to be joined
    back to front. Each line (`\P`, `\N`, `\X`, `^J`) becomes a text of
    its own, grouped with the others. Lines are placed from the attachment
    point (71) and spaced 5/3 of the height apart, times factor 44.
  - *MTEXT formatting codes* (`\f \H \C \p`… up to `;`, and `{}`) are
    dropped. A change of height or colour, or an underline, is reported as
    approximated. A stacked fraction `\S1/2;` is written inline as "1/2"
    and reported. A `\p` is paragraph formatting, not a line break; the
    old reader broke the line there.
  - *TEXT* reads `%%d %%p %%c %%% %%nnn`, `\U+XXXX` (including surrogate
    pairs), and `^I ^J ^M` and `^ `. Only those three carets are decoded,
    so "2^N" stays as it is. Text values keep their spaces.
  - *Placement:* TEXT with any alignment but left-on-baseline stands at its
    second point (11/21), moved to the baseline for bottom, middle and top.
    The old reader always used the first point, which other writers leave
    at 0, 0, and which AutoCAD writes after its own layout. MTEXT takes its
    direction (11/21), else its angle (50), in degrees as AutoCAD writes
    it.
- **Colour.** The full 256-entry index replaces the ten named colours:
  index 12 used to come in white. It is generated from its structure:
  - 24 hues;
  - five levels, 1, 0.65, 0.5, 0.3 and 0.15;
  - a pale version of each level at half saturation;
  - six greys.

  True colour (420) is read on entities and layers and wins over 62. It
  is written whenever the index does not hold the colour exactly, so
  every colour round-trips. Black is written as index 7 and true colour 0.
- **Code pages.** Every value is turned into UTF-8 as it is read.
  - A value that is valid UTF-8 is kept, which is how AutoCAD 2007 and
    later write every file.
  - Otherwise the value is read in `$DWGCODEPAGE`: Windows-1252 (the
    default) or Windows-1251.
  - Any other code page's bytes become U+FFFD and are counted in the
    report.

  A byte-order mark is skipped. A binary DXF is refused with a reason.
- **Units.** `$INSUNITS` scales the drawing into millimetres.
  - A block is scaled once, in its definition; an insert of it only moves.
  - The conversion goes in the report's new `converted` list, which the
    window shows in the status bar, not as a warning.
  - Saved files now declare `$INSUNITS` 4 (mm), `$MEASUREMENT` 1 and
    `$DWGCODEPAGE`.
- **Writing.** TEXT values are encoded so they read back:
  - the degree, plus-minus and diameter signs are written as `%%` codes;
  - a `%` that would start a code is written as `%%%`;
  - control characters are written as `^` codes, and a caret that could
    be read as one as `^ `.

  No value can carry a line break into the file.
- **Not done:** MTEXT word wrap to its box width (41). Text in a
  multibyte code page, such as `\M+` or ANSI_932, is reported, not
  decoded.

## Phase 109 — STEP fidelity

`LENGTH_UNIT` conversion, faces with inner loops (now that Phase 105
accepts them), per-solid partial import instead of all-or-nothing, and
round-trip-exact real formatting.

**As built.**
- **Units.** Each solid is scaled into millimetres by the LENGTH_UNIT of
  its shape representation's context: an SI unit of metres with any prefix,
  or a conversion-based unit (an inch is `LENGTH_MEASURE(25.4)` of the
  millimetre). The conversion goes in the report's `converted` list, and
  the window shows it in the status bar. A unit that cannot be read is
  taken as the millimetre and reported as approximated.
- **Partial import.** One solid that cannot be rebuilt no longer refuses
  the file. It goes in the report's `skipped` list as "solid N (#id): why",
  and the others come in. A solid's topology names use its index in the
  file, so they do not shift when an earlier solid fails.
- **Faces with holes.** The reader already built their inner loops, and
  Phase 105 validates them, but nothing downstream read them:
  - mass properties counted a hole as solid;
  - a face was drawn over its hole;
  - a Boolean cut through it.

  Now `BoundaryMesh::extractFacePolygons`, which the tessellator, the CSG
  and interference all read, gives a face with holes as one keyhole
  polygon, each hole bridged in from the outer loop. A hole's points
  follow its curved edges, so a round hole bounded by two arcs is a hole,
  not a line. The ear clipper no longer lets a bridge end block an ear.
  Mass properties triangulates the same keyhole polygon.
  - Tested with two hand-authored fixtures, a plate with a square hole and
    one with a round hole: the volume, the area and the drawn faces are
    right, the plate round-trips, and a Boolean cut into it is exact.
- **Reals** already round-tripped exactly (`std::to_chars` since Phase 97;
  pinned by `CoordinatesRoundTripExactly`).
- **Not done:**
  - Curved faces are still measured by their vertex polygons, so a round
    hole's wall (two arcs, four vertices) adds nearly nothing to the
    volume. This needs per-face surface integration.
  - BREP_WITH_VOIDS is still refused.
  - Review notes, not defects: a hole that no bridge can reach without
    crossing an edge is left out of the keyhole silently, and the crossing
    test has no tolerance. Rightmost-first bridging makes both unlikely for
    a simple face, and no input that triggers either has been found.

## Phase 110 — 2D correctness pass

- Snap tolerance in screen pixels.
- Hidden and locked layers left out of snapping and of Trim's cutting edges.
- Correct Midpoint and Center snap types, and an intersection snap.
- Trim keeps line type and group.
- Pick tolerances that do not depend on zoom.

**As built.**
- **Screen-pixel reach.**
  - `ViewportWidget::snap(worldPos)` sets the snap tolerance to 10 screen
    pixels at the current zoom. It used to be a fixed 0.5 world units: at
    high zoom that reached thousands of pixels, and zoomed out it reached
    under one.
  - `ViewportWidget::pickTolerance(pixels)` replaces the 16 copies of
    `std::max(10 px, 0.15)`, whose world floor made picks reach far past
    the cursor when zoomed in.
  - Every tool now snaps and picks through these two.
- **Layers.** The viewport's snap passes a filter that leaves out entities
  on hidden or locked layers. Trim's cutting edges skip them too. They used
  to include every entity in the drawing.
- **Snap kinds.**
  - Entities offer `typedSnapPoints()`. A line's midpoint is a snap point:
    it was none before.
  - Arcs, circles, ellipses and rectangles report centres; circles and
    ellipses report quadrants.
  - A block's content keeps its kinds.
  - Every snap used to be reported as "endpoint".
  - The overlay has markers for the new kinds: a diamond for a quadrant, an
    X for an intersection, and a plus for the grid, which used to have the
    X.
- **Intersection snap.** It is found among the entities that pass within
  the reach of the cursor, at most 32 of them.
- **Priority.** The nearest object snap wins, and on a tie the more
  specific kind. The grid is used only when no object snap is in reach; a
  nearer grid point used to beat an endpoint.
- **Performance.** The spatial-index snap looked each candidate up by
  scanning the whole entity list, a scan per candidate on every mouse move.
  It now makes one pass.
- **Keeping style.** `DraftEntity::copyStyleFrom` gives a piece the layer,
  colour, width, line type and group of its source.
  - Trim uses it; its pieces used to lose line type and group.
  - Break, Extend, Chamfer and Fillet used to drop the group.
  - A chamfer or fillet's joining piece joins a group only if both lines
    are in it.
