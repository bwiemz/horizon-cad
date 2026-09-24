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

Print and PDF/SVG export of the drawing at a chosen scale and paper size,
with real plot line weights and text to scale (P1). Printing waits on the
PrintSupport decision.

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

### Tests

11 new.
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
  - selecting shows and does not edit. This one fails on the old guard.

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

Layer rename and line weight; block base point; README claims matched to the
product.
