# 2D drafting

A drawing (File ▸ New Drawing, **Ctrl+N**) holds 2D geometry: lines,
circles, arcs, rectangles, polylines, splines, ellipses, text, hatches,
dimensions and blocks. The same tools draw into a sketch while one is being
edited (see [Sketches and constraints](sketches.md)).

## Placing points

Every drawing tool takes points by clicking. You can also type a point and
press **Enter**; it counts as a click there. Tools that take typed points:
Line, Circle, Arc, Rectangle, Polyline, Spline, Ellipse, and the linear
dimensions.

| Type | To place the point |
|---|---|
| `x,y` | at x, y |
| `@dx,dy` | dx, dy from the last point |
| `@length<angle` | at a distance and angle from the last point (degrees, counter-clockwise from +X; add `rad` for radians) |
| `length<angle` | at a distance and angle from the origin |
| a length alone | that far from the last point, towards the cursor |

A length may carry a unit: `2in`, `50 mm`, `1' 6"`, `3/4 in`. A bare number
is in the document's unit (see [Settings](settings.md)). What you type shows
in the status bar; **Escape** clears it, and a second **Escape** cancels the
tool's step.

## Drafting aids

Turn these on and off in the status bar, from **Tools ▸ Drafting Aids**, or
with their keys:

- **Object snap** (**F3**) pulls the cursor to endpoints (square marker),
  midpoints (triangle), centres (circle), quadrants (diamond) and
  intersections (cross). How near the cursor must be is the Snap reach in
  Preferences.
- **Grid snap** (**F9**) pulls it to the grid when no object snap is near.
- **Ortho** (**F8**) holds the next point to horizontal or vertical from the
  last one.
- **Polar tracking** (**F10**) holds it to 15° steps. Ortho and Polar are
  one or the other; an object snap wins over both.

Nothing on a hidden or locked layer is snapped to, selected or changed.

## Drawing

| Tool | Key | How |
|---|---|---|
| Line | **L** | Click the first point, then each next point; each segment starts where the last ended. **Enter** or **Escape** ends the chain. |
| Circle | **C** | Click the centre, then a point on the circle, or type the radius and press **Enter**. |
| Arc | **A** | Click the centre, then the start (this sets the radius), then the end. The arc runs counter-clockwise. |
| Rectangle | **R** | Click one corner, then the opposite one (or type `@width,height`). |
| Polyline | **P** | Click the vertices; finish with **Enter** or a double-click. **Escape** discards it. |
| Spline | **S** | Click four or more control points; finish with **Enter** or a double-click. |
| Ellipse | **E** | Click the centre, the end of the major axis, then a point that sets the minor radius. |
| Text | **T** | Click where it goes and type it. Change its height, rotation and alignment in Properties. |
| Hatch | **H** | Click a rectangle, a closed polyline or a circle to fill it. Change its pattern (Solid, Lines, CrossHatch), angle and spacing in Properties. |

## Selecting

With the Select tool (**Space**):

- Click to select; **Shift**+click adds or removes.
- Drag left to right for a window (blue): only what is wholly inside.
  Drag right to left for a crossing (green): anything it touches.
- **Delete** deletes the selection.
- Drag a selected entity's grips (its ends, centre, corners or vertices) to
  change it.

## Changing geometry

| Tool | Key | How |
|---|---|---|
| Move | **M** | Select first. Press on a selected entity, drag, release. |
| Rotate | **Shift+R** | Select first. Click the centre, then click a point or type an angle and press **Enter**. Makes rotated copies. |
| Scale | **Shift+S** | Select first. Click the base point, then click or type a factor. Makes scaled copies. |
| Mirror | **Shift+M** | Select first. Click two points on the mirror line. Makes mirrored copies. |
| Offset | **O** | Click a line, circle, arc, rectangle, polyline or ellipse, then click on the side and at the distance for the copy. |
| Trim | **X** | Click the piece of a line, circle or arc to remove, between the places other geometry crosses it. |
| Extend | **Shift+E** | Click a line or arc near the end to extend to the next geometry. |
| Break | **B** | Click a line or arc to split it where it crosses something, or where clicked. Click a circle to split it into two arcs where two things cross it. |
| Fillet | | Click two lines. For a radius other than the one shown (1 at first), type it and press **Enter** first. |
| Chamfer | | Click two lines. For a distance other than the one shown, type it and press **Enter** first. |
| Stretch | **W** | Click two corners of a crossing window, then a base point and where it goes. Points inside the window move. |
| Polyline Edit | | Click a polyline. Drag a vertex to move it; press **A** and click a segment to add a vertex, **D** and click a vertex to remove it, **C** to close or open it, **J** and click another polyline to join them. **Enter** finishes. |

**Rect Array** and **Polar Array** copy the selection in rows and columns,
or around a centre, from a dialog.

On the Edit menu: **Duplicate** (**Ctrl+D**) copies the selection beside
itself. **Copy** and **Paste** place copies with each click until you change
tool. **Group** (**Ctrl+G**) makes the selection select as one;
**Ungroup** (**Ctrl+Shift+G**) undoes that.

## Dimensions and text

| Tool | How |
|---|---|
| Linear (**D**) | Click two points, then where the dimension line goes. **Tab** chooses horizontal, vertical or aligned. |
| Radial | Click a circle or arc, then where the text goes. **Tab** switches radius and diameter. |
| Angular | Click two lines, then where the arc goes. |
| Continue, Baseline | Each click adds the next dimension in a chain, or from a common base, after the last linear dimension. |
| Leader | Click points, press **Enter**, then type the note. |

**Dimension ▸ Style...** sets the text height, arrow size, extension lines,
decimal places and unit of every dimension. To show other text on one
dimension, type it in Properties.

These dimensions describe the drawing; they do not drive it. To make a size
drive the geometry, use a constraint (see
[Sketches and constraints](sketches.md)).

**Measure ▸ Distance, Angle** and **Area** show their results in the status
bar without adding anything.

## Layers

The Layers panel lists each layer with its visibility (V), lock (L), colour,
line width and line type (LT). New geometry goes on the current layer, shown
in bold; double-click a layer's name to make it current. Double-click its
width or line type to change them. Layer "0" is always there; deleting
another layer moves its geometry to "0".

An entity's colour, width and line type are ByLayer (taken from its layer)
until you set them in Properties.

## Blocks

- **Block ▸ Create Block...** turns the selection into a named block, with a
  base point, and puts one copy of it where the geometry was.
- **Block ▸ Insert Block...** places copies of a block, at a rotation and
  scale, with each click.
- **Block ▸ Explode** turns selected blocks back into geometry.
