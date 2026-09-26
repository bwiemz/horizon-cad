# Assemblies

An assembly (File ▸ New Assembly) puts parts, and other assemblies, together
and holds them in place with mates. It refers to the part files; it does not
copy them. When a part changes on disk, the assemblies that use it are
updated.

## Components

- **Assembly ▸ Insert Component...** adds a part (`.hzpart`) or a whole
  assembly (`.hzasm`) as a component, beside the others.
- **Drag a component** with the Select tool to move it; its mates are kept
  as it moves. **Escape** puts it back.
- A selected component shows arrows and rings: drag an arrow to move it
  along that axis, a ring to turn it.
- **Move Component...** and **Rotate Component...** move it exactly.
- **Rename**, **Suppress** (leave it out for now) and **Remove** are in the
  Assembly menu and the Assembly tree's right-click menu.
- **Open Part** opens a component's part in its own tab.

## Mates

**Assembly ▸ Add Mate...** Click a face on each of two components first, and
they are filled in.

| Mate | Holds |
|---|---|
| Coincident | two faces in one plane, or two points together |
| Concentric | two round faces on one axis |
| Distance | two faces at a distance (or between limits) |
| Angle | two faces at an angle (or between limits) |
| Parallel, Perpendicular | two faces or edges |
| Tangent | a round face touching a flat or round face |
| Fixed | one component where it is |

A mate can be on a face, a straight or round edge, or a datum plane, axis or
point. A mate that cannot be satisfied is not added, and you are told why.
**Edit Mate...** changes a distance or angle mate's value; **Remove
Mate...** deletes one.

## The Assembly tree

It lists the components, the mates (with their values) and the patterns.
Double-click a mate to edit it, a component to rename it, a pattern to edit
it. Selecting a row highlights the component in the view. **Delete** removes
the selected row.

## Exploded views

**Assembly ▸ Explode Components...** moves chosen components along an axis
by a distance, as a step of a named exploded view. Each use adds a step.
**Show Exploded View...** shows one (or none: the components where they
are); **Remove Exploded View...** deletes one. Components cannot be dragged
while an exploded view is shown.

## Patterns and mirrors

**Pattern Components...** repeats components in a line or around an axis;
you can leave out instances by number. Instances follow their seed: to move
or change them, change the seed. **Mirror Components...** adds mirrored
copies across a plane.

## Checking

- **Check Interference** lists every pair of components that overlap, with
  the volume they share.
- **Bill of Materials...** lists the parts: top level, indented, or parts
  only. **Export CSV...** saves it.
