# Milestone 8: A Drawing You Can Hand Over (Phases 127–130): Implementation Plan

Roadmap: [2026-09-24-product-completeness-roadmap.md](../specs/2026-09-24-product-completeness-roadmap.md)

## Goal

A drafter can draw to exact sizes, dimension and annotate the drawing, and
hand it over on paper or as PDF, without leaving Horizon CAD.

## Order

Phase 128 comes first. Phase 127 needs the owner's answer on Qt
PrintSupport, since printing needs it. PDF (`QPdfWriter`, in QtGui) and SVG
(written directly) do not, and can go ahead either way.

## Phase 127: Print and PDF

### What the audit found

- **A drawing could not leave Horizon CAD except as DXF** (P1). There was
  no printing, no PDF, no SVG and no image.
- **Blocks drew only part of what they held.** The viewport drew seven
  kinds of entity from a block (lines, circles, arcs, rectangles,
  polylines, splines, ellipses). Text, hatches, dimensions and blocks within
  the block were not drawn (found mapping the renderer for this phase).

### As built

- **`PlotScene`** (drafting, no Qt) is what a drawing plots: strokes and
  text in world coordinates, built by `buildPlotScene`.
  - It includes entities on visible layers, locked ones too.
  - ByLayer and ByBlock are resolved.
  - Blocks are expanded to 16 levels, every kind of entity, text scaled and
    turned with its block.
  - Circles and arcs go in steps of a degree or less, ellipses and splines
    are evaluated, and hatches bring their boundary and clipped lines.
  - Dimensions bring their extension lines, dimension lines, arrowheads and
    value.
- **On paper:**
  - `plotTransform` places the scene: fitted to the printable area, or at a
    scale, centred, the page's y down. It says whether the drawing fits.
  - `plotWeightMm`: widths are millimetres, as DXF has them; the default
    width 1.0 plots at 0.25 mm. Clamped to 0.05–2.11 mm.
  - `plotDashMm`: the viewport's dash patterns in centimetres.
  - `plotColor`: white, drawn on the dark viewport, plots black; or
    everything black.
- **`io::SvgExport`**: a page the paper's size in millimetres, a polyline
  or polygon per stroke with weight, dashes and colour, and escaped text at
  its height. Numbers are written in the C locale, so a comma locale does
  not garble coordinates. It is written atomically.
- **`ui::exportPdf`**: a one-page vector PDF (`QPdfWriter`, QtGui) of the
  same scene, through `QSaveFile`. Pens are in millimetres, and dash
  patterns are in pen widths, as Qt wants. It is guarded by
  `QT_CONFIG(pdf)`: a Qt without PDF gets a clear error, not a build
  failure.
- **File ▸ Export ▸ PDF… / SVG…** ask for the paper (A0–A4, Letter, Legal,
  Tabloid), the orientation, the scale (fit, or 1:200 to 10:1) and the
  colours. At a scale the drawing does not fit, a warning comes before
  anything is cut off (Cancel is the default).
- **The viewport draws block references through `plotBlockReference`**,
  the same expansion, so everything a block holds shows on screen. This
  replaces 90 lines of per-type code.
- Checked by eye: a sample drawing exported to PDF and SVG, rendered with
  `pdftoppm` and `rsvg-convert`, came out the same and right. It had lines,
  a dashed red line, a circle, an arc, text, a dimension, and a turned,
  scaled block with text.

### Tests

10 new.
- Plot scene:
  - layer styles, and hidden and locked layers;
  - blocks expanded, nested, with ByBlock and text;
  - a dimension's lines and value;
  - weights, dashes and colours;
  - the drawing on the paper, fitted and at a scale that does not fit.
- SVG:
  - the page, strokes, weights, dashes and escaped text;
  - monochrome;
  - numbers under a comma locale (it skips where no de_DE locale exists).
- Window:
  - a drawing plots to PDF and to SVG from the menu;
  - a plot that does not fit is warned about, and Cancel writes nothing.

### Not done

- **Printing** (`QPrinter`, Qt PrintSupport) waits on the owner's
  decision.
- No paper space or layouts, and no title block on the plot.
- Plot styles (weight by colour) are not supported.
- Dimension text sits on its line rather than above it.
- Fonts are the system's sans serif. `DraftText` has no font of its own.

## Phase 128: Precise input

### What the audit found

- **Nothing could be drawn to a size.** No draw tool took a typed point or
  length. Only Fillet and Chamfer read numbers, and Rotate and Scale each
  had their own parser.
- **Line did not chain.** Each line took two clicks, and the next started
  from nothing.
- **Snaps could not be switched off.** The status bar said "SNAP GRID"
  whatever was on. The snap engine had no switch, and there was no ortho or
  polar tracking.
- **The property panel showed no geometry.** It had no fields for a line's
  ends, a circle's centre and radius, or an arc's angles.
- **Typing went nowhere after a ribbon click.** Picking a tool from the
  ribbon left the keyboard focus on the ribbon.
- **The property panel edited on selection** (found while building this
  phase). `refreshLayerList()` cleared the panel's "updating" flag halfway
  through `updateForSelection()`. So every field filled in after it (text,
  spline, hatch, ellipse) ran its edit slot. Selecting a second ellipse of a
  different size pushed "edits" onto the undo stack and marked the drawing
  modified.

### As built

- **`TypedPoint`** reads a point typed while a drawing tool waits for one:
  - `x,y`, absolute;
  - `@dx,dy`, relative to the tool's last point;
  - `@length<angle`, polar from the last point, the angle in degrees
    counter-clockwise from +X (`length<angle` is polar from the origin);
  - a length alone: that far from the last point toward where the cursor
    points (with its snaps and tracking).

  What is none of these is refused whole, with a reason in the prompt. Keys
  are read by code, not text: digits, `.`, `-`, `,`, `@`, `<`, Backspace.
- **Routing** (`ViewportInputHandler::handleKeyPress`):
  - A tool that places points says so (`Tool::acceptsTypedPoints`) and names
    its base point (`Tool::basePoint`).
  - Its keys build the typed text, and Enter gives the point to the tool as
    a click there. `ViewportWidget::applyTypedPoint` makes `snap()` return
    it exactly, with no snap or tracking.
  - Escape drops what is typed first, then cancels the tool.
  - Enter with nothing typed goes to the tool.
  - Seven tools take typed points: Line, Polyline, Rectangle, Circle, Arc,
    Ellipse and Spline.
  - Choosing a tool gives the viewport the keyboard focus.
- **Line chains.** Each click ends a line and starts the next one there, and
  Enter or Escape finishes. A click on the start point adds nothing.
- **Drafting aids** (`DraftingAids`, `SnapEngine::setObjectSnapEnabled` and
  `setGridSnapEnabled`), each a checkable action with its own key:
  - object snap (F3);
  - grid snap (F9);
  - ortho (F8);
  - polar tracking (F10), in 15° steps by default.

  They are under Tools ▸ Drafting Aids, and are toggles in the status bar in
  place of the fixed "SNAP GRID". They are saved in the preferences. Ortho
  and polar tracking are never on together. Tracking projects the cursor onto
  the nearest allowed direction from the tool's base point, and an entity
  snapped to wins over it.
- **Property panel geometry:**
  - fields for a line (start, end, length, angle), a circle (centre, radius)
    and an arc (centre, radius, start and end angle);
  - each field is taken when it is entered (keyboard tracking off), so
  typing 12.5 is one edit;
  - only the field entered changes; the others come from the entity, not
    their four-decimal fields;
  - the change goes through `GripMoveCommand`, as a grip drag does: one undo
    step, with constraints solved after.
- **The panel's guard** is restored, not cleared, by `refreshLayerList()`.
- **Found in review:**
  - The ellipse tool's last click used the cursor's last position, not the
    point given. A typed minor axis was lost, since a typed point does not
    move the cursor.
  - A Length or Angle entered for a line of no length (a grip dragged onto
    its other end) did nothing. It now takes its direction, or its length,
    from the other field.
  - Arc angles entered in the panel are normalized to [0, 2π) where they
    are set. The arc placed in the drawing was already normalized, being a
    copy.

### Tests

14 new.
- `TypedPoint`:
  - the four ways to type a point;
  - what is refused, with reasons;
  - keys, Enter and the prompt.
- Snapping: object and grid snaps switched off.
- Window, through the viewport's own event handling:
  - typed points draw a chain of lines, and Enter ends it;
  - a length goes toward the cursor;
  - a refused point is shown and changes nothing, and Escape drops typing
    first;
  - ortho and polar tracking hold the direction, one at a time;
  - snaps switched off do not pull a click that they reached a moment
    before;
  - geometry edited in the panel, one undo step each;
  - selecting shows and does not edit. This one fails on the old guard;
  - an ellipse takes its typed last point;
  - a line with no length takes one from the panel;
  - an arc's angles stay in range when edited.

### Not done

- Typed points in the dimension, text, leader, move/copy, mirror and array
  tools.
- Rotate and Scale still have their own number parsers.
- Rectangle, polyline and ellipse geometry in the panel.
- Choosing the polar step in the preferences (it is saved, but only
  settable in the settings file).
- Object-snap tracking lines (extension and alignment from snap points).

## Phase 129: Dimensions and text

Dimension style editor; display units in dimensions; baseline and continue
dimensions; multi-line text.

## Phase 130: Layers and blocks

### What the audit found

- **A layer could not be renamed**, though the README said it could, and its
  line weight could not be changed from the panel.
- **Create Block had four problems:**
  - It put the block's base point at the average of its entities' centres,
    with no way to choose it.
  - Undo put the entities back at the end of the drawing order.
  - Redo built a new block reference, under a new ID, so whatever named the
    old one no longer found it.
  - The originals were removed one at a time.
- **The README claimed things the product did not do:**
  - custom hatch patterns;
  - polylines closed by the tool;
  - the system clipboard;
  - a configurable dimension style;
  - shortcuts for all tools;
  - format v16 (it is v18);
  - a list of ribbon tabs that has changed.

### As built

- **Layer rename:**
  - `LayerManager::renameLayer`, and `RenameLayerCommand`, which carries the
    entities on the layer, in the drawing and in block definitions, and the
    current layer.
  - It refuses the default layer 0, a name already taken, and an empty
    name, changing nothing (`applied()`).
  - The layer panel has Rename…, which also refuses characters DXF does not
    allow in a layer name.
- **Layer line weight:** a double-click on the layer's Width column sets it
  (`ModifyLayerCommand`).
- **Create Block:**
  - It asks for the name and the base point in one form. The base point
    defaults to the centre of the selection's bounds (`FeatureForm::text` is
    new).
  - `CreateBlockCommand` takes the base point.
  - It removes the originals in one pass, and undo puts them back where
    they were (`removeEntities`/`restoreEntities`).
  - The block and its reference are made once, so redo restores the same
    reference under the same ID.
- **The README's Features** now say what the product does, including Phase
  128's input and this phase's layers and blocks.
- **Found in review:**
  - Create Block put its reference on layer 0 whatever the current layer,
    where every other way of making an entity, Insert Block among them,
    uses the current layer. It now takes the current layer.
  - The line-weight dialog's 0.1–10 range clamped what it showed of a
    thinner weight (DXF's 0.05 mm), so OK changed a layer that was only
    looked at. It now takes 0.01–25.

### Tests

4 new.
- Document:
  - a rename carries entities (drawing and block), properties and the
    current layer, and undoes;
  - refused renames change nothing;
  - Create Block keeps drawing order on undo and the reference's ID on
    redo.
- Window: Create Block offers the selection's centre and takes the base
  point typed.

### Not done

- Picking the base point in the viewport. The form takes typed
  coordinates.
- Explode still recreates its entities on redo.
