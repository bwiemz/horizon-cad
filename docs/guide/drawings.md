# Drawing sheets

A drawing sheet (`.hzdwg`) is a sheet of paper with views of a part or an
assembly, dimensions and a title block. It refers to the model file and
projects its views from it each time it opens, so it is always up to date.

## Making a sheet

**Drawing ▸ New Drawing from Part or Assembly...** uses the part or assembly
you are working on (it must be saved), or asks for one. The sheet starts on
A3 landscape with four views: front, top, right and isometric, at the
largest standard scale that fits. A sheet of an assembly has a parts list
and numbered balloons.

The AI assistant makes one too, when asked for a drawing, and on a sheet's
tab it changes the sheet as the Drawing menu does: see
[the AI assistant](ai-assistant.md).

## The sheet

- **Title Block...** fills in the title, part number, revision, author,
  date, material, company and sheet, and chooses the paper (A0 to A4, or
  ANSI A to D) and its orientation.
- **Scale...** chooses a standard scale, or the largest that fits.
- **Update from Part** draws the views again from the model now. A sheet is
  drawn again by itself when the model's file changes.

## Views

- **Add Section View...** cuts a front, top or side view vertically or
  horizontally, and shows the cut as a new, lettered view.
- **Add Detail View**: click the centre on a view and then the radius, and
  choose the detail's scale; it shows that circle enlarged.
- **Move View**: click a view, then where it goes.
- **Remove View...** removes a view, and the views made from it.
- **View Properties...** shows or leaves out a view's hidden edges, tangent
  edges and centre lines.

## Dimensions

**Add Dimension**, then click edges on the views: a circle gets its
diameter, an arc its radius, a straight edge its length. **Remove
Dimension**, then click an edge, removes its dimension.

**Add Hole Callout**, then click an edge of a hole made with **Hole**: the
hole is called out on a leader, its size and depth as the Hole feature says
them ("⌀6.6 THRU", "⌀5 × 10 DEEP"), a counterbore's or countersink's on a
second line ("CBORE ⌀11 × 6.4 DEEP", "CSK ⌀12 × 90°"), and how many of it the
part has when a pattern or mirror repeats it ("3× ⌀6.6 THRU"). It follows the
hole when the part changes. Call a hole out in the view that looks along it,
where it is a circle. **Remove Dimension** on the hole's edge removes its
callout. A sheet with hole callouts is saved in a format older versions of
Horizon CAD do not open.

## Geometric tolerances and datums

**Add Datum Feature...**, then click an edge: the edge's face becomes a
datum, drawn as a triangle on the edge and its letter in a box at the end of a
short leader, as ASME Y14.5 and ISO 1101 draw it. The form offers the next
free letter (A, B, C..., never I, O or Q); each further edge clicked becomes
the letter after. A letter is used once on a sheet.

**Add Geometric Tolerance...** asks for a feature control frame:

- its **characteristic**: position, flatness, straightness, circularity,
  cylindricity, perpendicularity, parallelism, angularity, profile of a
  surface, circular runout, concentricity or symmetry;
- its **tolerance**, in the sheet's unit;
- its **zone**: between two lines or planes, or a cylinder (⌀), for an axis
  or a hole's position;
- its **material condition**: regardless of feature size, maximum (Ⓜ) or
  least (Ⓛ);
- up to three **datums** it refers to, primary first, from those on the sheet.

Then click edges: each gets the frame, a row of boxes on a leader from the
middle of the edge, out of the view. The form refuses what the standard does
not allow, and says why: a form tolerance (flatness, straightness,
circularity, cylindricity) refers to no datum; orientation, position,
runout, concentricity and symmetry need one; only those that control an axis
or a plane of a feature of size take MMC or LMC. Two frames on one edge stack
on one leader. A hole's frame, from any edge of a hole the view calls out,
hangs under its callout, as a hole's position is given under its size, and
the hole's datum under the frame.

The characteristic symbols are drawn in lines and circles rather than as
characters, so they print, and export to DXF, whatever the fonts. A frame and
a datum follow their edge as the part changes; one whose edge the view no
longer shows is said when the sheet is opened. **Remove Dimension** on an
edge removes its frames and its datum too; a frame elsewhere that still
refers to that datum is said. Frames and datums go on the side of the edge
facing out of its view: when views are close, move them apart to make room. A
sheet with frames or datums is saved in a format older versions do not open.

## Notes and exporting

The 2D tools draw notes on the sheet. They are kept with the sheet and move
with their view. The sheet's own layers (its views, border and title block)
are locked.

**File ▸ Export ▸ PDF...** or **SVG...** writes the sheet on its paper at 1:1.

Each change to a sheet (its title block, paper, scale, a view added, moved
or removed, a view's settings, a dimension, a frame or a datum) is one step
for **Edit ▸ Undo**
and **Redo**. A view's notes, drawn by hand inside it, move back with it.
Drawing a sheet again from its part (**Update from Part**, or the part
changing) is not a step: it changes nothing of the sheet's own.
