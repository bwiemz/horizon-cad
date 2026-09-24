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

**As built.** An optional `std::string* reason` out-parameter rather than a
new return type — the same contract the file readers took in Phase 99, and one
that leaves every existing caller and test compiling. It is on
`Feature::execute` (every feature fills it), `Extrude`, `Revolve` and
`BooleanOp`; Loft, Sweep, Draft, Pattern and the primitives report a reason
from their feature, since their kernel ops do not yet distinguish their
failure modes. `ProfileValidator`'s messages now name coordinates.

Writing those messages exposed that the profile reader accepted only lines,
arcs and a lone circle: a Rectangle- or Polyline-tool shape (and an imported
LWPOLYLINE) could not be extruded at all. It now reads them as their line
segments and names any kind it still cannot use.

## Phase 104 — Real 3D commands, undoable

Primitive / Boolean / Fillet / Chamfer / Shell / Pattern / Loft / Sweep ribbon
commands create document features (dialogs + selection) instead of fixed
demos outside the document. Feature add / edit / delete / reorder / suppress
and assembly component / mate edits become undo-stack commands, which also
retires the explicit `setDirty(true)` calls Phase 98 kept for them.

Large, so two PRs: **104a** makes every model edit an undoable command; **104b**
replaces the demo ribbon commands with ones that add features.

### What the audit found

- Feature edits bypass the undo stack. Add and reorder call
  `setDirty(true)`; **editing a feature's parameters (double-click) does not
  mark the document modified at all**, so closing discards the edit without a
  prompt. Undo never touches the model.
- There is no way to delete or suppress a feature. The panel labels features
  past the (unreachable) rollback index "Suppressed".
- Assembly edits (insert component, add mate) set the assembly's own dirty
  flag and are not undoable; the mate solve moves components with no way back.
- Every 3D ribbon command except Extrude/Revolve is a fixed demo that adds a
  mesh to the scene graph outside the document (never saved, never undone);
  the Boolean demos `const_cast` a box's vertices. Shell, Pattern, Loft,
  Sweep and Draft have no command at all.
- The scripting API adds features straight to the tree: not undoable, and
  the document is not marked modified.

### 104a design — undoable model edits

- **Feature suppression.** `Feature::isSuppressed()` / `setSuppressed()`; every
  build path skips a suppressed feature exactly as it skips a datum. Stored as
  `"featureSuppressed": true` (`"suppressed"` is already a pattern's list of
  skipped instances, renamed in code to `suppressedInstances()`).
- **Format version 17.** A build that predates body operations (102) or
  suppression would silently build a different part from a file that uses
  them; the version gate refuses instead of misreading.
- **Tree primitives.** `FeatureTree::insertFeature`, `takeFeature`, `indexOf`,
  and a `revision()` counter bumped by every mutation (and `markChanged()` for
  edits made through a `Feature*`), so the window knows when an undo changed
  the model. Removing or inserting before the rollback index shifts it.
- **Commands** (`FeatureCommands.h`, `hz::doc`): `AddFeatureCommand` (with the
  wrapper sketch it was made from), `RemoveFeatureCommand`,
  `MoveFeatureCommand`, `EditFeatureCommand` (parameters and body operation),
  `SetFeatureSuppressedCommand`, `AddSketchCommand`. Commands find their
  feature by identity, not by index, so an edit made outside the stack
  (a script) cannot make undo act on the wrong feature. Each restores the
  rollback index it found.
- **Assembly edits.** `AssemblyDocument::State` (components + mates) with
  `snapshot()` / `restore()`; `AssemblyEditCommand(before, after)` pushed on
  the assembly tab's undo stack after an insert or mate. The tab is modified
  when either the assembly flag or the undo stack says so; saving marks both.
- **Window.** Add (Extrude/Revolve/box from the empty state), edit, reorder,
  delete and suppress push commands; undo/redo rebuild the model when the
  tree's revision moved and the assembly scene when an assembly is active.
  The feature panel gets a context menu (Edit, Suppress/Unsuppress, Delete)
  and the Delete key; features can no longer be dropped *onto* each other
  (which nested them in the panel without changing the tree).
- **Scripting.** `ScriptContext` pushes the same commands.

### 104b design — real 3D ribbon commands

- Primitives ask for dimensions and a body operation and add a
  `PrimitiveFeature`.
- Union / Subtract / Intersect add a `BooleanFeature` that combines the
  part's bodies (its single-solid `execute` is a no-op today); Subtract keeps
  the first body and removes the others.
- Fillet / Chamfer / Shell / Pattern / Loft / Sweep / Draft ask for their
  inputs (edges and faces by name until the viewport can pick them) and add
  features. The demos and their scene-graph side channel are removed.

**As built.** One form builder (`FeatureForm`) serves every command. Edges are
listed by their end points and faces by which way they face and where their
middle is (the TopologyID in the tooltip), until the viewport can pick them.
Combine splits the part into its bodies (`Pattern::separate`, one solid per
shell) and folds them in body order; with one body the ribbon command refuses,
and the feature passes the part through. New commands: Shell, Draft, Linear
and Circular Pattern, with icons.

**Deferred: Loft and Sweep commands.** Both need sketches on more than one
plane — a loft's sections, a sweep's profile across its path — and the window
has no way to make one: every profile is the top-level drawing on XY, wrapped
in a sketch. They wait for a *sketch on a plane* command (XY / YZ / XZ, an
offset, a datum or a planar face), added to the roadmap as Phase 104c.

### Requirement → task map

| Roadmap requirement | Where |
| --- | --- |
| Feature add undoable | 104a `AddFeatureCommand`; `addBodyFeature` |
| Feature edit undoable | 104a `EditFeatureCommand`; double-click edit |
| Feature delete | 104a `RemoveFeatureCommand`; panel menu + Delete key |
| Feature reorder undoable | 104a `MoveFeatureCommand` |
| Feature suppress | 104a `Feature::setSuppressed`, `SetFeatureSuppressedCommand`, `"featureSuppressed"` |
| Assembly component / mate edits undoable | 104a `AssemblyEditCommand` |
| Retire `setDirty(true)` for these edits | 104a (the undo stack's clean index carries it) |
| Primitive commands create features | 104b |
| Boolean commands create features | 104b |
| Fillet / Chamfer / Shell / Pattern commands create features | 104b (plus Draft) |
| Loft / Sweep commands create features | 104c — needs sketches on planes |

### Tests (104a)

- Commands: add → undo → redo keeps the same feature object and the model's
  volume; remove/move/edit/suppress undo to exactly the prior tree; a feature
  added outside the stack is not the one an undo removes; rollback index
  follows inserts and removals.
- Suppression: a suppressed Cut leaves the plate whole; round trip keeps it;
  a pattern's instance suppression still round-trips.
- Window (offscreen): editing a parameter marks the tab modified and undo
  clears it; delete and suppress through the panel; undo after Extrude
  removes the body; inserting a component into an assembly marks it modified
  and undo removes it.

## Phase 105 — Topology correctness gates

Euler–Poincaré with ring and genus terms (valid iff `V−E+F−R` is even and at
most `2S`); `GeometryValidator` gates every solid-producing op; profile
validation rejects self-intersection and zero-length extrudes.

**As built.**
- **Euler–Poincaré.** `checkEulerFormula()` now checks `V − E + F − R = 2(S − G)`, and `genus()` reports G.
  - The old form was `V − E + F = 2(S − R)`, with the rings on the wrong side and doubled. It rejected every torus and every part with a hole through it.
  - Two of those rejections reached users: **STEP import refused them** ("failed validation"), and **FilletOp refused every such part**, since it gates its result on `isValid()`.
  - The count is global, not per shell, because the Euler operators build intermediate states, such as a lone vertex on a face, that no loop walk reaches.
- **One gate in the feature tree.** It is not a per-op change: every feature's result, and every Join / Cut / Intersect result, must pass the manifold, Euler and geometric checks. The tolerance scales with the part's size. Otherwise the feature fails with a reason in the user's terms, for example "the result is not a valid solid: a face's boundary crosses itself".
  - No existing test produced a solid the gate refuses.
  - The whole suite runs in the same 4 seconds as before.
  - **Cost on large parts.** On a 10,000-box pattern, `GeometryValidator` took 4.3 s of a 5.4 s debug rebuild. Almost all of it went to sampling each face's surface normal at nine points. Two changes brought it to 0.23 s, so the rebuild now takes 1.3 s:
    - A bilinear carrier is decided from its four control points instead.
    - A `FailingOnly` scope skips the two report-only checks.
  - The sampling had also called about one face in nine of that pattern "curved", so those faces were never checked for flatness. They are checked now.
- **Profiles that cross or touch themselves are refused**, with the point named. Arcs are sampled for the test. Zero-length extrudes were already refused in Phase 103.
- **Found, not fixed:** a fillet on a Boolean result fails with "non box-like corner". FilletOp supports only three-edge corners, and the sewn faces of a Boolean can meet in more. That belongs with persistent naming and rebuilding the fillet and chamfer ops on SolidSewer (see the kernel findings note).

## Phase 106 — Persistent naming, first cut

Names derived from generating geometry (source profile segment + side) rather
than storage order, so an upstream edit that adds a vertex does not retarget a
downstream fillet; `TopologyID::resolve` wired into feature execution; unique
face IDs after Booleans.

### The defect

An Extrude named its side faces `lateral_0 … lateral_n` in profile order, and
its edges `edge0 … edgeN` in storage order. A fillet stores the name of its
edge, so any upstream edit that changes the count retargets it silently:
another vertex, or an arc's facet count. The fillet then rounds whichever edge
now carries the old number. Fillet and Chamfer look their edges up by exact
name only. A Boolean gives every piece of a split face the same name.

### Design (first cut)

- **Versioned naming.** `ExtrudeFeature` records the scheme it names with:
  `Positional` (the old rule) or `FromGeometry`.
  - Files written before this load as Positional, so every saved fillet, chamfer and mate keeps its reference. New features use FromGeometry.
  - The feature stores `"naming": 2`, and the format goes to **version 18**, so an older build refuses a file it would misname.
- **Faces from their source.** Each side face is named after the sketch entity that made it:
  - `side:e<entity id>` for a line or arc;
  - `side:e<id>.<k>` for side k of a rectangle or polyline;
  - `.f<j>` appended for facet j of an arc or circle.

  Caps stay `cap_bottom` / `cap_top`. Entity ids are saved with the document, so the names survive reopening. `ProfileValidator` reports each ordered edge's source, and `sampleProfile` reports each chord's.
- **Edges from their faces.** An edge is named after the two faces it separates, as `edge:<a>|<b>` with the face roles sorted. Editing another part of the sketch leaves it alone.
- **Resolution.** Fillet, Chamfer and Shell take an exact match first, else every descendant (`TopologyID::resolveAll`). A reference to a face or edge that an operation split or patterned still finds its pieces.
- **Unique pieces after a Boolean.** When a face comes out in several pieces, each is named `<face>/piece:<k>`, and references to the face resolve to all of them.

Revolve, Loft, Sweep and the primitives keep their names for now. Each needs the same source mapping, and Extrude is where sketch edits happen most.

### Tests

- Inserting a vertex in the sketch (splitting a line) leaves a fillet on an untouched vertical edge on the same edge. Under Positional naming, the same edit moves the fillet.
- Names survive a save and reload.
- A version-17 file keeps Positional naming, and its fillet still resolves.
- The side faces of a rectangle and an arc are named as above.
- A face split by a Cut gets unique pieces, and a Shell referencing the unsplit name removes all of them.

**As built.**
- **Every feature records its scheme.** `Feature::naming()` is saved as `"naming": 2`. New features use `NamingScheme::FromGeometry`, and a file without the key loads every feature as `Positional`. The format is at version 18.
  - Extrude names its own faces and edges by the scheme.
  - BooleanOp renames its result only when the feature doing the Join / Cut / Intersect (or Combine) is FromGeometry.
  - The review caught an earlier draft that renamed every Boolean result. That broke references saved against the sewer's numbering in files from Phase 102 onward.
- **Edge-naming rule.** `nameEdgesByFaces` gives `<src>/edge:<a>|<b>` when both faces come from the same feature, and `edge:<faceA>|<faceB>` otherwise. The rule is shared, so an edge between the same two faces has the same name whether Extrude built it or a Boolean re-sewed it.
- **Where the new names apply.** Boolean results made by FromGeometry features are named by faces and pieces (`nameFacePieces`). The sewer's own positional edge names stay for the other ops that sew: cylinders, spheres, cones, tori, Revolve, Loft and Chamfer. Documents saved with fillets on those parts made their references against those names.
- **Resolution.** Fillet and Chamfer resolve a reference to the edge itself or all of its pieces, counted once each. Shell takes the face or its first piece, and mates already did the same.
- **Found, not fixed: Boolean fragmentation.** The BSP CSG splits faces along the other operand's planes even where the cut never reaches, and does not merge the fragments back. A groove across a 10 × 10 × 5 block leaves 30 faces where about 10 would do; the bottom face, which the cut never touches, comes out in two pieces.
  - So an edge next to such a face is renamed by an unrelated Boolean, even though both faces are still there.
  - The extra vertices on those edges are very likely why Fillet refuses Boolean results ("non box-like corner", Phase 105).
  - Merging each face's coplanar fragments after the CSG would fix both. It is added to the roadmap as **106b**.

## Phase 106b — Boolean fragments merged

**As built.**
- **`mergeFragments`** (`FragmentMerge.cpp`, internal to the CSG) runs on a Boolean's fragments before sewing, for FromGeometry features. Old documents' Booleans are left alone, as their names depend on the fragment count.
  1. Fragments are grouped by source face, operand and plane.
  2. Vertices are welded at the CSG plane tolerance.
  3. Every edge is split at vertices lying on it, and cancels against its reverse. What is left chains into loops.
  4. Straight-run vertices are dropped; the sewer puts back any a neighbour needs.
  5. A pinch, an overlap, or a hole the cuts cannot resolve leaves that face's fragments as they were.
- **Holes.** A face here has one loop, so a merged region with holes is cut through each hole, along a line through its middle parallel to the region's longest outer edge, and each side is merged on its own. The cuts meet the outer boundary part-way along an edge, so the outer corners of a plate with a hole stay where three faces meet.
- **Results:**
  - A groove across a block leaves 10 faces, where there had been more than 20 triangles.
  - A plate with a square hole leaves 12 faces.
  - An edge a cut never touches keeps its name.
  - **Fillet accepts Boolean results.** The Milestone 2 goal passes end to end through the window: a plate with a hole, filleted, undone and redone, saved and reopened.
