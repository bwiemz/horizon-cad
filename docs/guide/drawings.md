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

## Notes and exporting

The 2D tools draw notes on the sheet. They are kept with the sheet and move
with their view. The sheet's own layers (its views, border and title block)
are locked.

**File ▸ Export ▸ PDF...** or **SVG...** writes the sheet on its paper at 1:1.

Changes to a sheet are saved with it, but cannot be undone with Undo.
