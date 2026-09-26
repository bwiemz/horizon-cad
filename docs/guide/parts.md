# Parts

A part (File ▸ New Part) is built from features, in order: sketches made
solid, holes drilled, edges rounded, and so on. The Feature Tree lists them.
Change any feature and the part is built again from it.

## How features are made

Choose a feature from the **Model** menu or the ribbon's **3D** tab. A form
asks for its values; lengths and angles take units ("2 in", "0.5 rad") and
show the document's unit.

Faces and edges clicked with the Select tool before you choose a command are
checked in its lists already. Click a face or edge on the model to select
it; **Shift**+click adds more.

**Result** says what a feature does to the part: Join the part, Cut from the
part, Keep the intersection, or New body. A feature that would leave nothing
(a cut through everything) is not added, and the status bar says why.

## Features

| Feature | What it asks |
|---|---|
| **Box, Cylinder, Sphere, Cone, Torus** | Their sizes; where they stand (x, y, z) and which way they point. |
| **Extrude** (**Ctrl+Shift+E**) | The distance, and how far it goes: to the distance, both ways, through all, or up to a face. Which way (out of the sketch or reversed). |
| **Revolve** (**Ctrl+Shift+R**) | The angle (up to 360°), and the axis: the sketch's vertical (Y) or horizontal (X) axis through its origin. |
| **Loft...** | Two or more sketches, in order; the solid passes through each. |
| **Sweep...** | A profile sketch and a path sketch; the profile is carried along the path. |
| **Hole** | The face and the point (the point is moved onto the face), the type (simple, counterbored or countersunk), how far (to a depth, through all, or up to a face), the diameter and depth, and the drill point's angle (0 for a flat bottom). |
| **Fillet** (**Ctrl+Shift+F**), **Chamfer** (**Ctrl+Shift+C**) | The radius or distance, and the edges. |
| **Shell** (**Ctrl+Shift+H**) | The wall thickness, and the faces to leave open (none makes a closed hollow). |
| **Draft** | The pull direction, the neutral plane, and the angle. |
| **Linear Pattern** | The direction, the spacing and the number of instances, and the features to repeat (none repeats the whole part). |
| **Circular Pattern** | The axis, the number of instances, and the angle they span. A full turn spaces them evenly. |
| **Mirror** | The plane (YZ, ZX, XY, or a flat face), and the features to mirror (none mirrors the whole part). |
| **Union, Subtract, Intersect** | No form: they combine the part's bodies (made with Result: New body). Subtract cuts every later body from the first. |

**Model ▸ Datum** adds reference geometry: a plane (from a face or a
principal plane, offset and turned), an axis or a point. Datums are drawn
dashed in orange, and sketches and mates can use them.

## The Feature Tree

- The Status column says OK, Suppressed or Rolled back. A feature that
  failed is shown in red, with the reason in its tooltip.
- **Double-click** a feature (or **Edit...** in its right-click menu) to
  change any of its values. A direction can be set from a face or edge
  clicked in the view.
- **Right-click** to Suppress or Unsuppress a feature, Roll Back to Here
  (build the part only up to it), Roll Forward to the End, or Delete.
- **Drag** a feature to move it in the order.
- Every change is one step of **Undo**.

A part that takes a while to build is built in the background: you can keep
working, and the status bar shows its progress with a **Cancel** button.

## Measuring

**Model ▸ Mass Properties...** (**Ctrl+Shift+M**) shows the part's volume,
surface area, centre of mass, and, for a material you choose (steel,
aluminium, titanium, ABS), its mass and moments of inertia.

**View ▸ Section Plane...** cuts the view across X, Y or Z at a position,
keeping one side; **View ▸ No Section** removes it.
