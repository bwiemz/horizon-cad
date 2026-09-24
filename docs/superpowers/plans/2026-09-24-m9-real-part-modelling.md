# Milestone 9: Real Part Modelling (Phases 131–135): Implementation Plan

Roadmap: [2026-09-24-product-completeness-roadmap.md](../specs/2026-09-24-product-completeness-roadmap.md)

## Goal

A designer can model a real part in the window: sketch on any plane or face,
with holes in the profile; pick faces and edges by clicking; reach every
modelling command from a menu; change a feature's definition afterwards;
and see the part the way they need to.

## Starting point (mapped for this plan)

- **Profiles are the whole drawing.**
  - Extrude and Revolve take every top-level entity in the drawing, on XY.
    `resolveProfileSketch` wraps them in a `doc::Sketch` that shares the
    entities.
  - `ProfileValidator` accepts exactly one closed loop. So a note, a
    dimension or a second loop anywhere in the drawing makes both fail, and
    a plate with a hole cannot be modelled.
  - Nothing ever makes a sketch active: `setActiveSketch` is only ever
    called with `nullptr`.
- **`doc::Sketch` exists but is not used.** It has a plane (by value),
  entities in local 2D, constraints and a spatial index.
  - Sketches are saved without their constraints.
  - Drawing tools cannot write into one, and the renderer never draws one.
  - `draft::SketchPlane` has `localToWorld` / `worldToLocal` and
    `rayIntersect`.
  - `model::DatumPlane::toSketchPlane()` exists. Datums can be made only
    from scripts.
- **No 3D picking.**
  - `MeshData` has no face ids. `SolidTessellator` drops the `topoId` that
    each face polygon carries.
  - The edge overlay draws every triangle's edges, not the model's.
  - GPU colour picking exists but is unused. It picks nodes, not faces, and
    cannot run in the GL-less window tests.
  - Fillet, chamfer and shell choose from text lists, and a faceted
    cylinder lists every facet. Mates choose from raw tag strings.
- **Features cannot be changed where it matters.**
  - The edit dialog shows raw parameter names and angles in radians (a
    full revolve shows as 6.2832).
  - Directions, axes and positions are fixed: Extrude along the sketch
    normal and blind, Revolve about world Y, primitives at the origin.
  - Loft, Sweep and datums have a kernel, a feature and a file format, but
    no command.
  - Rollback has no control. It is not undoable, not saved, and the
    background rebuild ignores it.
  - A pattern repeats the whole solid.
- **Finding and seeing.**
  - The 16 modelling commands are on the ribbon only: no menu, so not in
    the Ctrl+K palette, and no shortcuts.
  - Fit All ignores solids.
  - The camera is always perspective (`resizeGL` resets it).
  - There are no display modes, no mass-properties dialog, and no section
    plane in the UI (`GLRenderer::setClipPlane` is never called).

## Order

131 → 132 → 133 → 134 → 135. Sketching on a face (131) lists planar faces
until 132 lets them be clicked. 132's picking is what 133's and 134's
definitions choose directions and faces with. 135 is independent and last.

## Phase 131: Sketches on planes

### As built

- **A sketch owns a drawing.** `doc::Sketch` holds a `draft::DraftDocument`
  (`drawing()`): its entities, index, blocks and dimension style, in the
  plane's coordinates.
- **The document has an active drawing.**
  - `Document::editSketch(sketch)` / `editedSketch()`.
  - `activeDrawing()` and `activeConstraints()` are the edited sketch's, or
    the top level's.
  - Every drawing tool, the selection, the property panel, the clipboard,
    arrays, blocks, groups, the dimension style and the constraint colours
    use them.
  - DXF import and export, plotting and the whole-drawing profile keep the
    top level.
  - `removeSketch` stops editing the sketch it removes.
  - `drawings()` lists the top level and every sketch. Renaming or
    removing a layer carries what is on it in all of them, blocks included.
- **Sketch mode:**
  - Model ▸ New Sketch ▸ on the XY, XZ or YZ plane, on a face (a list of the
    part's flat faces, through the face's middle, facing out), or on a
    datum plane.
  - Model ▸ Edit Sketch… and Finish Sketch.
  - While a sketch is edited, the view works in the sketch's coordinates:
    it looks straight down on its plane, and the solid is drawn placed in
    that frame. So every tool, snap and typed point works unchanged.
  - The camera is saved on entering and restored on leaving.
  - Undoing the new sketch leaves it. Each tab keeps what it is editing.
  - The feature tree panel lists the sketches above the features, with
    what uses each and which is being edited. A double-click edits one,
    and the one chosen is what Extrude and Revolve take.
- **Profiles:**
  - Extrude and Revolve take the sketch being edited (and finish it), else
    the one chosen, else the drawing as shown (hidden layers left out).
  - `ProfileValidator::regions()` finds every closed loop and nests them by
    the even-odd rule: outer loops with holes, islands in holes as regions
    of their own. Text, dimensions, leaders and hatches are passed over; a
    block reference is named ("explode it first").
  - Loops that cross or touch are refused, saying where.
  - Extrude and Revolve build each region as its outer loop less its holes:
    - the hole's cutter reaches past the ends, a tenth further for Extrude
      and a twentieth of the angle for Revolve;
    - through a full turn it is a cavity;
    - the regions are joined through the exact Boolean;
    - a single plain loop is built exactly as before, under the same names.
  - Revolve turns about the sketch's own vertical or horizontal axis (it
    was always world Y, which a sketch on XZ could not use).
- **Files:**
  - Sketches are saved with their constraints; they were lost.
  - There is no default sketch any more. Every document had an empty
    "Default Sketch", and `m_defaultSketch` went stale after a load. One in
    an old file that nothing uses is dropped on reading.
- **Found building it:** the face lists named each face by the way its loop
  wound, and a box's loops wind inwards. "Facing (0, 0, 1)" was the bottom,
  so Shell's list opened the wrong face of anything not symmetric, and a
  sketch "on the top face" went on the bottom. Normals now face out of the
  part (by the sign of its volume).
- **Found in review:** a tool mid-way through something (a line's first
  click) kept its point when an undo took the sketch away, so its next click
  made a line from a point in the sketch's frame to one in the drawing's.
  The tool now starts again whenever the frame changes, not only for Edit
  and Finish Sketch.

### Tests

30 new, 2 changed.
- Profile regions:
  - holes and islands;
  - separate regions;
  - notes passed over;
  - refusals with where;
  - a plate with two holes extruded (genus two, holes named);
  - regions extruded together;
  - a window revolved a quarter and a full turn.
- Document:
  - a sketch's own drawing;
  - editing makes it active, and only this document's;
  - layers renamed and removed across drawings.
- Files:
  - a sketch keeps its constraints;
  - an old empty default sketch is dropped unless used or drawn on.
- Window:
  - a sketch on XZ drawn and extruded where it should be;
  - a plate with a hole from a sketch, with a note on it;
  - a note in the drawing no longer stops Extrude;
  - a boss on a box's top face;
  - undoing a new sketch leaves it, and a line begun in it starts again;
  - editing again from the list, with undo in the sketch;
  - a revolve about the sketch's own axis.
- Shell opens the face that faces up: it opened the bottom. This and the
  face sketch fail with the old face normals.
- Changed: the DXF-free kernel tests still pass through the new dispatch;
  `SolidCommandsTest.ShellOpensTheChosenFace` gained the check above.

### Not done

- A sketch on a face does not follow the face when the part changes: its
  plane is copied.
- Sketch geometry cannot yet use the part's edges (projection).
- The top-level drawing is not shown while a sketch is edited.
- Sketches cannot be renamed or deleted from the list.
- Loft and Sweep still take one loop.

## Phase 132: 3D picking

### As built

- **The mesh says what it shows.** `geo::MeshData` gains, for a mesh
  tessellated from a solid:
  - `triangleFaces`, an index per triangle into `faceTags`, the faces'
    persistent names;
  - `edges`, the part's edges as named polylines (curved ones sampled).
    Seams between two facets of one curved surface are left out: the same
    ideal surface, or two ideal surfaces bending by less than 30 degrees, as
    a patterned cylinder's copies do. So are lines across one flat face.

  Meshes read from a file's cache carry none of this, and are not picked.
- **The part's edges are drawn,** not every triangle's. Faces are pushed back
  a little (a fill polygon offset) so the edges and highlights on them show.
  A mesh without edges keeps the triangle wireframe.
- **`render::MeshPicker`** picks on the CPU, so the window's GL-less tests
  pick as a user does:
  - `pickFace`: the nearest triangle the ray through the cursor hits
    (Möller–Trumbore), and its face.
  - `pickEdge`: the edge drawn nearest the cursor, within the pick distance
    on screen. An edge further along the ray than the face in front is
    hidden, with a slack of a few pixels' width at that depth, since an
    edge a pixel from the cursor sits a little behind the face beside it.
- **In the viewport:**
  - `pickModel` gives an edge near the cursor, else the face under it, as a
    `ModelPick` (owner: a component's id, or 0 for the part; the tag; face
    or edge). Nothing while a sketch is edited.
  - The Select tool: a click on no drawing entity chooses what is under it,
    Shift adds or takes away, and a click on nothing clears. Moving the
    cursor highlights what a click would choose.
  - The choice is drawn in orange and the hover in blue, faces filled and
    edges thick. Rebuilding the model clears both.
- **The commands take what was clicked:**
  - Fillet and Chamfer check the clicked edges in their lists, and Shell the
    clicked faces.
  - New Sketch on a Face takes a clicked flat face without asking.
  - Add Mate starts from faces clicked on two components: scene nodes
    carry their component's id.
- **Found in review:**
  - A highlighted edge is drawn over the part's own line for it, at the same
    depth, and the depth test (less-than) rejected it: only its fringe
    showed. It is drawn with less-or-equal, as face highlights are.
  - The hover stayed drawn after switching to another tool. Changing the tool
    clears it.
  - Sketch on a Face now takes the first flat face among those clicked.

### Tests

15 new.
- Picker:
  - the nearest face, from above and below, and a miss;
  - a moved mesh picked where it is;
  - an edge near the cursor, too far, and hidden.
- Tessellator:
  - triangles name their faces, and a box has its twelve edges;
  - a faceted cylinder shows its rims, not its seams.
- Window, clicking at points on screen:
  - clicks choose faces, with Shift to add and take away, and nothing
    clears;
  - near an edge a click takes it, but not a hidden one;
  - the hover, and another tool dropping it;
  - Shell opens the clicked face;
  - Fillet rounds two clicked edges;
  - a sketch goes on the clicked face.

### Not done

- Picking tests every triangle of every solid on each mouse move. A large
  model will want bounds per node, then a BVH.
- Components shown from a file's cache are picked only once resolved: their
  cached mesh names no faces. No window test drives Add Mate by clicking:
  it needs saved part files, and is covered by the list path.
- Dragging a box to choose faces, and choosing a whole loop or chain of
  edges.

## Phase 133: The missing commands

### As built

- **Loft, Sweep and datums in the Model menu:**
  - Loft… takes two or more sketches from a checklist, in order.
  - Sweep… takes a profile sketch and a path sketch.
  - Datum ▸ Plane… is offset from XY, XZ, YZ or a clicked flat face, and
    can be turned about its x axis.
  - Datum ▸ Axis… runs along X, Y, Z or a clicked face or edge, through a
    point.
  - Datum ▸ Point… is at typed coordinates.
  - The viewport draws datums dashed: a plane as a square, an axis as a
    line, a point as a cross. Suppressed and rolled-back ones are not drawn.
  - New Sketch ▸ On a Datum Plane (Phase 131) now has datums to use.
- **Loft and Sweep say why they fail.** The kernels returned nullptr with
  no reason, and the part said only that it could not be built. Now:
  - a loft names the section and its corner count against the first's;
  - a sweep names the path that doubles back, turns too sharply, sweeps
    along the profile's plane, or runs into itself.
- **Editable definitions:**
  - Each parameter has a kind (`Feature::parameterKind`): length, angle,
    count or choice.
  - The edit form labels the fields, shows angles in degrees (a circular
    pattern's step too), counts as whole numbers, and a Boolean's operation
    by name.
  - Features have named vectors (`vectors()`/`setVector`): Extrude's
    direction; Revolve's axis point and direction; Draft's pull direction
    and neutral point; a pattern's direction or axis.
  - The form offers each direction as it is, one of the six axes, or along
    or against a face or edge clicked on the part. Each point is three
    coordinates.
  - `EditFeatureCommand` carries vectors, and undo puts them back. Refused
    ones (no length, or along the sketch) are left out and named.
- **Rollback:**
  - The tree's menu has Roll Back to Here and Roll Forward to the End.
  - Each is one undo step (`SetRollbackCommand`). It was written straight
    to the tree.
  - The rollback point is saved in the file, so the background rebuild,
    which builds from a saved copy, honours it too.

### Tests

11 new, 1 changed.
- Document:
  - parameter kinds;
  - a direction refused when it has no length or lies along the sketch,
    and kept as a unit vector;
  - a direction edit undone;
  - rollback as an undo step;
  - Loft and Sweep failure reasons.
- Files: the rollback point is kept, and a stale one ignored.
- Window:
  - a revolve's angle edited in degrees;
  - an extrusion reversed;
  - a loft from XY to a sketch on a datum plane (a frustum, exactly);
  - a sweep of a square along a line;
  - rolling back from the tree, and undo.
- Changed: a refusal now names the field's label ("Segments per turn"), not
  its code name.

### Not done

- The rollback point is set from the tree's menu, not by dragging a bar.
- A datum is its coordinates when made: it does not follow a face it was
  made from.
- Loft sections must still have as many corners each.

## Phase 134: Extrude and pattern options

- **Extrude:**
  - reverse, symmetric (both ways, half each) and through-all (as far as
    the body reaches along the direction);
  - and to a distance from a face.
- **Pattern the selected features:** a linear or circular pattern repeats
  the tool bodies of the chosen features (a hole's cut, a boss's join), not
  the whole solid.
- **Placed primitives:** a primitive has a position and an orientation,
  editable afterwards.
- **Tests:**
  - symmetric and through-all volumes;
  - a patterned hole cuts N holes;
  - a placed box saves and reloads where it was.

## Phase 135: Finding and seeing

- **A Model menu** with every modelling command. These reach the Ctrl+K
  palette, and the common ones get shortcuts. The 2D Fillet and Chamfer
  are renamed in the palette so the two are told apart.
- **Fit All includes solids,** and has one action with its shortcut.
- **View:**
  - orthographic or perspective, kept across a resize;
  - Back, Bottom and Left views;
  - display modes: shaded, shaded with edges (the model's edges, 132), and
    wireframe.
- **Mass properties:** Model ▸ Mass Properties shows the volume, area,
  centre of mass and inertia for a chosen material.
- **Section plane:** a clip plane on X, Y or Z at an offset, with edges
  clipped too. No cap yet; that is recorded as not done.
- **Tests:**
  - every modelling command is in a menu and the palette;
  - Fit All frames a solid;
  - ortho survives a resize;
  - the mass-properties dialog's numbers for a box.
