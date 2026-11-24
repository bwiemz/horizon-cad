# Assembly motion — design

Item 4 of `docs/superpowers/notes/2026-11-23-competitive-gaps.md`: motion for
assemblies. SolidWorks calls it a motion study: a motor on a mate turns or
slides a component, the other mates carry the rest of the mechanism along,
and the study shows where it binds and where parts collide.

## Intent

A user who has mated a mechanism (a hinge, a slider, a linkage) wants to see
it move through its range, find where it stops, and find where parts hit
each other, before making it. The AI assistant wants the same answers in
words ("the arm clashes with the base from 80° on").

## What it does

- **A driver.** One Distance or Angle mate is driven from one value to
  another in steps. Those are the mates that carry a value; a hinge is a
  Concentric and a Coincident mate, and an Angle mate between two faces turns
  it. The driver's limits, if it has any, are set aside while it drives.
- **Each step solved.** At each value the assembly's mates are solved from
  where the last step left the components, as a drag is, so the mechanism
  moves continuously and does not jump to another way of meeting its mates.
- **Where it binds.** The first value the mates cannot reach ends the study:
  "the mates do not hold past 40°". What it reached is kept, and can be
  played.
- **Where it collides.** Each position is checked for interference (shared
  material, as the Interference check measures it); the first one with a
  clash is named, with its components.
- **Nothing changed.** The study does not move the components. Its positions
  are shown (a pose drawn in place of the placements: not an edit, not
  saved), played through with a slider, and one can be kept, as one undo
  step.

## Approaches considered

1. **Drive a mate and solve per step (chosen).** Uses the mate solver every
   drag already uses, with its limits; mechanisms of any shape that mates
   describe; no new solver.
2. **Kinematic chains** from `hz::kin` (serial forward/inverse kinematics).
   Only serial chains, and the joints would have to be inferred from mates;
   a four-bar linkage is not serial.
3. **Rigid-body dynamics** (forces, masses, contact). What SolidWorks Motion
   Analysis adds on top of motion; far larger, and not what most users of a
   motion study need first. Not now.

## Design

- `hz::doc::MotionStudy` (document layer):
  - `prepareMotion(assembly, documents, mate, why)` on the GUI thread: parts
    read whole, mates gathered, the driver found (refused unless Distance or
    Angle), and a copy of each component's solid for the interference check,
    so the run can go to a worker while the assembly is edited.
  - `runMotion(input, study, cancelled)` on any thread: the steps, each
    solved from the last; pattern instances placed from their seeds; each
    position checked for interference when asked; stops where the mates
    bind, or when cancelled.
- `AssemblyDocument::setPose` / `pose()`: placements drawn in place of the
  components' own (`displayTransform`), not an edit and not saved.
- UI: Assembly ▸ Motion Study... (driver, from, to, steps, check for
  collisions), run on a worker; then a motion bar: a slider over its
  positions, Play, the value and any clash at each, Keep This Position (one
  undo step), Close.
- AI: `study_motion` (mate, from, to, steps): how far it went, where it
  bound, and where parts clash, in words and numbers.

## Not now

Saving studies with the assembly; more than one driver; time, speed and
forces; exporting the motion as video. Each can follow once the study exists.
