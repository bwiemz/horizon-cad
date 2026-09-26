# Getting started

## The window

- **The ribbon** across the top has seven tabs: Home, Draw, Modify,
  Annotate, Constrain, Block and 3D. The menus above it hold the same
  commands and more. Every tab and menu is there whatever you are working
  on; a command that does not apply says so in the status bar.
- **The Feature Tree** (on the left) lists a part's features. The
  **Assembly** tree, tabbed with it, lists an assembly's components, mates
  and patterns.
- **The viewport** in the middle shows the drawing or the model.
- **Properties** (on the right) shows and changes what is selected.
  **Layers** is tabbed with it.
- **The status bar** at the bottom shows the cursor's coordinates, what the
  active tool is asking for, the drafting aids (SNAP, GRID, ORTHO, POLAR)
  and how many things are selected.
- **Documents** open in tabs. There is always at least one tab.

Show or hide the panels from the **View** menu. Their layout is kept when
you close Horizon CAD.

## Finding a command

Press **Ctrl+K** for the command palette, type part of a command's name
("extrude", "export", "line") and press **Enter**. It searches every menu
command and shows its shortcut.

## Moving around the view

| To | Do this |
|---|---|
| Orbit | Drag with the middle mouse button |
| Pan | Hold **Shift** and drag with the middle mouse button |
| Zoom | Turn the mouse wheel; it zooms about the cursor |
| See everything | Press **F** (Fit All) |
| Look along an axis | Click a face of the ViewCube (top right), or use View ▸ Front, Top, Right, and so on |
| Go back to the isometric view | Click **HOME** under the ViewCube |

View ▸ Orthographic switches between perspective and orthographic views.
View ▸ Display chooses Shaded, Shaded with Edges or Wireframe.

## Tools

The left mouse button works the active tool. Choose a tool from the ribbon,
a menu, or its key (**L** for Line, **C** for Circle; see
[Keyboard shortcuts](shortcuts.md)). **Space** goes back to the Select tool.
**Escape** cancels the tool's current step and keeps the tool; press Space
to leave it.

## A first part in five steps

1. Choose **File ▸ New Part**.
2. Choose **Model ▸ New Sketch ▸ On the XY Plane**. The view turns to look
   straight at the sketch.
3. Press **R** for Rectangle. Click one corner, then type `@40,20` and press
   **Enter** for the opposite corner, 40 by 20.
4. Choose **Model ▸ Finish Sketch**.
5. Choose **Model ▸ Extrude** (**Ctrl+Shift+E**), set the Distance to 10,
   and click OK.

The part now has a sketch and an extrusion in the Feature Tree. Double-click
the extrusion there to change its distance. See [Parts](parts.md) for what
comes next.

## Samples

**File ▸ Open Sample** lists the samples that come with Horizon CAD: an
L-shaped bracket, a plate with a row of counterbored holes, a pin, an
assembly of the plate and the pin held by mates, a drawing sheet of the
bracket, and a 2D gasket.

A sample opens as a copy, in the "Horizon CAD Samples" folder in your
documents, so you can change it and save it. A copy you have changed is
kept: opening that sample again opens your copy.
