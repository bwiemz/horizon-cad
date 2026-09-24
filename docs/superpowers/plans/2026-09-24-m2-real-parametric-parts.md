# Milestone 2 — Real parametric parts (Phases 102–106): Implementation Plan

Roadmap: [2026-09-23-production-readiness-roadmap.md](../specs/2026-09-23-production-readiness-roadmap.md)

## Goal

A user can model a plate with a hole, fillet an edge, undo it, save and reopen
it — through the UI, with every failure explained.

## Phase 102 — Multi-body regeneration

### The defect

`Document::rebuildModel()` replays the tree through `buildWithDiagnostics()`,
which threads one solid through the features. Every feature that creates
geometry (Extrude, Revolve, Loft, Sweep, Primitive) ignores the solid it is
handed, so each one *replaces* the part: a second Extrude discards the first,
and a hole cannot be cut. `buildBodies()` models separate bodies but is
called only from tests, and `BooleanFeature` is a no-op on the product path.

### Design

- **Body operation.** `enum class BodyOperation { NewBody, Join, Cut,
  Intersect }` on the `Feature` base class, meaningful for features whose
  `createsNewBody()` is true. The tool body the feature builds is combined
  with the part so far:
  - *New body* — kept as a separate body: its shells are added to the part's
    solid alongside the existing ones (no Boolean). A part is one
    `topo::Solid` that may hold several shells, as a spaced pattern already
    does, so rendering, the tessellation cache, mates and mass properties need
    no change.
  - *Join / Cut / Intersect* — `BooleanOp` Union / Subtract / Intersect of the
    part and the tool.
  - Cut or Intersect with no part yet fails with a reason; a Cut that removes
    everything, an Intersect of disjoint bodies, or a Boolean that fails, fail
    with a reason — never an empty part presented as success.
- **One regeneration rule** in `FeatureTree`, shared by `build()` and
  `buildWithDiagnostics()`; `buildBodies()` applies the same operations to
  its body list so the two paths cannot disagree.
- **Persistence.** `"bodyOperation"` on each creating feature (`"new"`,
  `"join"`, `"cut"`, `"intersect"`; `"operation"` is already the Boolean
  feature's type). A file without it — every file written before this
  phase — loads as *New body*, so every body it describes appears (before,
  only the last creating feature's result survived the rebuild).
- **UI.** Extrude and Revolve ask for the operation, defaulting to Join when
  the part already has a body and New body otherwise. `BooleanFeature` stays
  readable for old files.
- **Utility.** `Pattern::collect(a, b)` — both solids' shells in one solid —
  exposes the shell cloning Pattern already does.

### Tests

Closed-form volumes, as the kernel tests do:

| case | expected volume |
|---|---|
| 10×10×2 plate, cut by an r2 circle extruded 2 at 32 segments | 200 − 2·(16·4·sin(π/16)) |
| two 10³ boxes 5 apart, Join | 1500 |
| two disjoint boxes, New body | 2000, two shells |
| two 10³ boxes overlapping by half, Intersect | 500 |
| Cut with no body / Cut removing everything | failure with a reason |
| file without "operation" | New body; round trip keeps each operation |

## Phase 103 — One result contract for kernel ops

`OpResult<T>` (value + status + reason) modelled on `SolveResult`, adopted by
Extrude, Revolve, Sweep, Loft, Shell, Draft, BooleanOp, Pattern, Fillet and
Chamfer; the feature tree surfaces the reason instead of "Feature 'X' failed
to execute", and the Extrude command stops guessing "profile is not a closed
loop" for every failure.

## Phase 104 — Real 3D commands, undoable

Primitive / Boolean / Fillet / Chamfer / Shell / Pattern / Loft / Sweep ribbon
commands create document features (dialogs + selection) instead of fixed
demos outside the document. Feature add / edit / delete / reorder / suppress
and assembly component / mate edits become undo-stack commands, which also
retires the explicit `setDirty(true)` calls Phase 98 kept for them.

## Phase 105 — Topology correctness gates

Euler–Poincaré with ring and genus terms (valid iff `V−E+F−R` is even and at
most `2S`); `GeometryValidator` gates every solid-producing op; profile
validation rejects self-intersection and zero-length extrudes.

## Phase 106 — Persistent naming, first cut

Names derived from generating geometry (source profile segment + side) rather
than storage order, so an upstream edit that adds a vertex does not retarget a
downstream fillet; `TopologyID::resolve` wired into feature execution; unique
face IDs after Booleans.
