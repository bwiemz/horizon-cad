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

- **A sketch owns a drawing.** `doc::Sketch` holds a `draft::DraftDocument`
  (entities, spatial index, block table, dimension style) in place of its
  own entity vector and index.
- **The document has an active drawing.**
  - `Document::activeDrawing()` is the sketch being edited, or the
    top-level drawing.
  - The window's editing code (tools, selection, property panel, clipboard)
    uses it. File formats, plotting and export keep using
    `draftDocument()`, the top level.
  - Constraints follow the same way.
- **Sketch mode:**
  - Model ▸ New Sketch on XY / XZ / YZ / a datum plane / a planar face. The
    face is chosen from a list until 132.
  - Model ▸ Edit Sketch and Finish Sketch. While a sketch is edited, the
    viewport looks along its normal and works in its coordinates, so every
    drawing tool, snap and typed point works unchanged. The solids are
    shown placed in the sketch's frame.
  - Sketches appear in the feature tree above the features that use them.
- **Profiles from the sketch:**
  - Extrude and Revolve use the selected, or last edited, sketch. The
    old whole-drawing path stays for drawings without sketches, and ignores
    text, dimensions, hatches and hidden layers.
  - `ProfileValidator` returns regions: an outer loop with any inner loops
    (holes), from nested closed loops.
  - Extrude and Revolve build a region with holes as the outer loop's solid
    minus each hole's, through the exact Boolean pipeline. Persistent names
    come from the Boolean's face pieces.
- **Files:** sketches are saved with their constraints. The stale
  `m_defaultSketch` after a load (it can point at a sketch no longer in the
  list) is fixed.
- **Tests:**
  - an extrude of a plate with two holes, checked by volume;
  - a revolve with a hole;
  - a sketch on XZ and on a face;
  - a note in the drawing no longer breaks Extrude;
  - tools draw into the sketch and undo there;
  - sketches round-trip with their constraints.

## Phase 132: 3D picking

- **Tessellation keeps its topology.** `MeshData` gains per-triangle face
  indices and a face table of `TopologyID`s. The tessellator records edge
  polylines, sampled along their curves, with their ids.
- **The edge overlay draws the model's edges,** not triangle wireframe.
- **CPU ray picking** (`render::MeshPicker`):
  - `Camera::screenToRay` (Qt-style y) is tested against each node's
    bounds, then its triangles (Möller–Trumbore), giving the nearest face.
  - Edges are hit by screen distance to their projected polylines, within
    the pick tolerance.
  - It works without GL, so the window tests drive it.
- **Highlight:** the face or edge under the cursor, and the chosen ones,
  drawn in the highlight colour.
- **Choosing by clicking:** Fillet, Chamfer and Shell open a pick mode that
  collects edges or faces by click (Shift adds, a second click removes)
  and then asks for the size. The lists stay as a fallback. Add Mate picks
  a face on each component. "New sketch on a face" takes the clicked face.
- **Tests:**
  - picker units: the nearest face through a box and a cylinder; the edge
    nearest the cursor; a miss;
  - through the window: a fillet by clicking two edges; a shell by
    clicking the top face; a mate by clicking.

## Phase 133: The missing commands

- **Loft, Sweep and datum commands.**
  - Loft takes two or more sketches. Sweep takes a profile sketch and a
    path sketch.
  - Datums: a plane offset or at an angle, through three points, or a
    midplane; an axis; a point.
  - Datums are drawn, can be picked (132), and sketches can be placed on
    them.
  - Loft and Sweep failures say why. The kernel returns nullptr with no
    reason today.
- **Editable definitions:**
  - Parameters carry a kind (length, angle, count, choice). The edit
    dialog labels them and shows angles in degrees.
  - Directions, axes and positions can be edited: Extrude's direction,
    Revolve's axis, the pattern direction and axis. They are picked (132)
    or chosen from the axes.
- **Rollback bar:**
  - The feature tree gets a draggable marker.
  - Moving it is an undo step.
  - It is saved in the file.
  - The worker rebuild honours it.
- **Tests:**
  - loft and sweep from the menu;
  - datum then sketch on it;
  - angles shown and taken in degrees;
  - a changed extrude direction;
  - rollback undone, saved and reloaded.

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
