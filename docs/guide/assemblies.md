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

## Motion

**Assembly ▸ Motion Study...** moves a mechanism through its range: a hinge
turning, a slide sliding, a linkage working. Choose the mate that drives it,
a Distance or an Angle mate (a hinge is a Concentric and a Coincident mate,
and an Angle mate between two of its faces turns it, from 0 to 180 degrees:
the angle between two faces), the values to drive it from and to, how many steps, and whether to check each position for
collisions.

Every other mate is solved at each step, from where the last one left the
components, so the mechanism moves on as it would by hand. The status bar
says what it came to: through its whole range, or where the mates bind and
it stops, and where parts first clash.

The positions are played in the **Motion Study** window, beside the main one,
so you can turn the view while it moves: drag the slider, or **Play**. Each
says the driving mate's value and which parts clash there. Nothing is moved
until you choose **Keep This Position**, which leaves the components there,
and the driving mate at its value, as one undo step. Editing the assembly,
dragging a component or showing another tab closes the window and draws the
components where they are.

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
