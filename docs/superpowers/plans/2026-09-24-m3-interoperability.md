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

## Phase 109 — STEP fidelity

`LENGTH_UNIT` conversion, faces with inner loops (now that Phase 105
accepts them), per-solid partial import instead of all-or-nothing, and
round-trip-exact real formatting.

## Phase 110 — 2D correctness pass

- Snap tolerance in screen pixels.
- Hidden and locked layers left out of snapping and of Trim's cutting edges.
- Correct Midpoint and Center snap types, and an intersection snap.
- Trim keeps line type and group.
- Pick tolerances that do not depend on zoom.
