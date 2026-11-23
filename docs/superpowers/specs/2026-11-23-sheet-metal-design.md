# Sheet metal: a base flange from an open sketch

## Why

Sheet metal is one of the first things a SolidWorks user reaches for
(brackets, enclosures, covers), and a shop needs the flat pattern to cut.
Horizon has the maths (bend allowance, K-factor, a folded strip extruded to
a width, its developed length) but only as a library: no command, no file
support, nothing the AI can call. See
`docs/superpowers/notes/2026-11-23-competitive-gaps.md`.

## Goal

**Base Flange** in the Part workbench, as SolidWorks has it: an open sketch
of lines (an L, a U, a Z, a hat section...) becomes a sheet of a given
thickness, each corner a bend of a given inside radius, extruded to a width
along the sketch's normal. It is a feature in the history like any other:
edited, saved, undone, built on (holes, cuts, patterns). Its flat pattern
(the developed length by the width, with its bend lines) exports to DXF for
cutting. The AI assistant can add one.

## Non-goals (for now)

Edge flanges on an existing body, hems, jogs, relief cuts, bend tables,
unfolding a part that later features cut (the flat pattern is the base
flange's own), arcs in the sketch (a bend comes from the radius).

## Geometry

- The sketch's lines, end to end, are the **virtual sharps of one surface
  of the sheet**: the surface the thickness grows from (the strip's bottom).
  By default the material grows to the left of the lines as they run from
  the first point; `flip` puts it on the right.
- At each corner the turn is a bend of the same angle. On the bottom
  surface a bend's radius is the inside radius `r` when the bend turns away
  from the material, `r + t` when toward it; its tangent points lie
  `R·tan(θ/2)` from the corner along each line. A flat length is a line's
  length less those setbacks; a line too short for its bends is an error that
  says which.
- The strip folds as `SheetMetalSolid::fold` does (start at the origin along
  +x, material toward +y, extruded along +z), then is placed so its start is
  the sketch's first point, +x along its first line, +z along the sketch
  normal.
- Bend lines in the flat pattern run across it at the middle of each bend's
  allowance.

## Parameters

`thickness` (length), `bendRadius` (length, inside), `kFactor` (a plain
number 0-1: a new parameter kind, Number), `width` (length),
`segmentsPerBend` (count), `flip` (choice: left, right).

## Left to the builder

Names, where the polyline is chained from the sketch, the DXF layer names
for the flat pattern, the command's form.
