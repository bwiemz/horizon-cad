# Sketches and constraints

A sketch is 2D geometry on a plane in a part. Extrude, Revolve, Loft and
Sweep build solids from sketches.

## Making a sketch

- **Model ▸ New Sketch ▸ On the XY Plane** (or XZ, or YZ) makes a sketch and
  opens it for editing. The view turns to look straight at it.
- **On a Face...** puts it on a flat face of the part: click the face first,
  or choose it from the list. The sketch follows the face when the part
  changes.
- **On a Datum Plane...** puts it on a datum plane (see [Parts](parts.md)).

While a sketch is being edited, the [2D drafting](drafting.md) tools draw
into it. **Model ▸ Finish Sketch** closes it, puts the view back, and makes
it the profile that the next Extrude or Revolve uses.

To change a sketch later, double-click it in the Feature Tree's Sketches
list, or use **Model ▸ Edit Sketch...**. A single click in that list chooses
the sketch as the profile.

While editing:

- **Model ▸ Project Edges...** draws edges of the part onto the sketch, as
  construction geometry or as part of the profile. They follow the part and
  are drawn in violet.
- **Model ▸ Construction** makes the selected geometry construction geometry
  (dashed, and not part of the profile), or back.

## Constraints

Constraints hold geometry in relation to other geometry, and the drawing is
solved to keep them. Choose one from the **Constrain** tab or the
**Constraint** menu, then click what it applies to:

| Constraint | Click |
|---|---|
| Coincident | two points (line ends, centres, arc ends, polyline vertices) |
| Horizontal, Vertical | two points |
| Perpendicular, Parallel | two lines |
| Tangent | a line and a circle or arc, or two circles or arcs |
| Equal | two lines, or two circles or arcs |
| Fixed | one point |
| Distance | two points; you are asked for the distance |
| Angle | two lines; you are asked for the angle |

Distance and Angle constraints are driving dimensions. To change one,
double-click the geometry with the Select tool; the drawing is solved again.

Once a drawing has constraints, geometry is coloured by how far it is held:
green where it can still move, red where it is over-constrained. The
Properties panel lists the selected geometry's constraints, with a button to
delete one.

## Variables

**Edit ▸ Variables...** lists a part's variables: a name and an expression
such as `3 mm`, `10 * wall` or `4`. Expressions can use `+ - * / ^`,
brackets, `pi`, and `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2`,
`sqrt` and `abs`.

To drive a feature by a variable, open the feature's Edit dialog
(double-click it in the Feature Tree) and type `=` and an expression in a
length or angle field, for example `=wall * 2`. The field then keeps the
expression, and the feature follows when the variable changes.

## Configurations

**Edit ▸ Configurations...** is a design table: a row for each
configuration, a column for each variable. A cell sets that variable in
that configuration; a blank cell keeps the variable's own value. Choose a
configuration in the Feature Tree's Configuration list to build the part as
it.
