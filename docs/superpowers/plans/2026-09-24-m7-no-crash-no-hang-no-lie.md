# Milestone 7: No Crash, No Hang, No Lie (Phases 122–126): Implementation Plan

Roadmap: [2026-09-24-product-completeness-roadmap.md](../specs/2026-09-24-product-completeness-roadmap.md)

## Goal

Everything the audit found that crashes, hangs, or gives a wrong answer
without saying so is fixed or refused. Each fix comes with a test that fails
without it.

## Phase 122: 2D crash and hang fixes

### What the audit found

- **Insert Block started twice crashed.** `onInsertBlock` registered a new
  `InsertBlockTool` under the same name. `ToolManager::registerTool`
  replaced, and so destroyed, the tool that was active. `setActiveTool` then
  called `deactivate()` on it, and so did the viewport, which held its own
  pointer to it. Under AddressSanitizer, the new test fails on the old code
  with a heap-use-after-free at `ToolManager.cpp:24`.
- **Removing an entity rebuilt the whole R-tree.** `RTree::remove` collected
  every entry, cleared the tree and inserted them all again. The following
  each removed entities one at a time, O(n log n) per entity:
  - Delete, Cut and Create Block;
  - undo of a DXF import, a duplicate, a mirror, a rotate, a scale or an
    explode.

  Undoing a 100,000-entity import was about 10¹⁰ insertions.
- **Undo moved deleted entities to the end.** `RemoveEntityCommand::undo`
  used `addEntity`, so a restored entity was drawn over everything else.
- **Finding an entity by id scanned the drawing:**
  - once per selected entity in box selection, group expansion and the
    grip hit test;
  - every frame in `renderGrips`, and in the constraint labels;
  - 49 times in the commands, and on every mouse move in snapping.
- **The spatial index went stale.**
  - Moves and block edits rebuilt the whole index after each change.
  - Eight property commands never updated it: text content, height,
    rotation and alignment; spline closed; ellipse semi-axes and rotation.
    Picking and box selection then worked from the old outline.

### As built

- **R-tree deletion** (`RTree::remove(value, where)`):
  - The entry is found by searching only the nodes that intersect the box
    it was inserted with, and taken out of its leaf.
  - Underfull nodes on the way up are dissolved and their entries inserted
    again (Guttman's CondenseTree), and the root shrinks.
  - Freed nodes are reused.
  - O(log n). A wrong box costs a full search, never a wrong result.
  - `remove(value)` still removes every copy.
  - A randomized test checks 6,000 inserts, removals (with right, wrong and
    no box) and queries against brute force.
- **`SpatialIndex`** remembers the box each entity is indexed under. It can
  remove or update an entity that has moved since it was indexed, and it
  keeps one entry per id.
- **`DraftDocument`:**
  - an id→entity map: `findEntity` / `sharedEntity`, O(1);
  - `removeEntity` returns the entity's position, and `insertEntity`
    inserts at one;
  - `removeEntities` / `restoreEntities` remove or restore a batch in one
    pass, with positions;
  - `replaceEntity` swaps in a new object for grip edits;
  - `updateEntityBounds` re-indexes one entity.

  `rebuildSpatialIndex()` rebuilds both indexes, so it stays a full resync
  point.
- **Commands:**
  - All 49 id scans are lookups.
  - `RemoveEntityCommand::undo` restores the position.
  - A new `RemoveEntitiesCommand` backs Delete and Cut.
  - Moves, block edits and grip edits update only the entities they change.
  - The eight property commands update the index.
- **Selection, grips and snapping:**
  - Box selection, group expansion, the grip hit test, `renderGrips`, the
    constraint labels and the array commands use the lookup.
  - Snapping has an overload that takes the drawing: index, then lookup, so
    it no longer scans the entity list on every mouse move.
- **Insert Block:**
  - `ToolManager::registerTool` stops pointing at a tool it is about to
    destroy.
  - `onInsertBlock` releases the viewport's tool before replacing it.
- **Tests:** 15 new.
  - R-tree:
    - random churn against brute force;
    - node reuse;
    - duplicate values.
  - `DraftDocument`:
    - lookups;
    - order through remove and insert;
    - batch round trip;
    - replace;
    - bounds update;
    - 20,000 removals in reverse order within a time budget.
  - Commands:
    - positional undo;
    - batch undo;
    - a move re-indexes;
    - a text-height change re-indexes.
  - Window:
    - Insert Block three times;
    - delete a box selection, then undo and redo, in order.
- **Not done:**
  - Removing an entity from the middle of the list is still an O(n) shift.
    A batch pays it once; one at a time, from the end, as undo removes, it
    is O(1).
  - The constraint solver's full index rebuild after a solve is left to
    Phase 138.

## Phase 123: Kernel honesty

Shell refuses what it cannot shell and offsets by loop winding. Draft and
Chamfer take their direction from winding. The last good solid is kept, and
the failing feature marked. Surface area on non-convex faces and the stray
sphere in STL/glTF exports are fixed.

## Phase 124: Hostile files, round 2

A DXF expansion budget; multi-loop HATCH import with tests; valid
lineweights; an entity budget on every reader.

## Phase 125: Background work that stops

Cooperative cancel for STEP import and interference; wedged-job and
trial-build exception safety; autosave failures shown to the user; a
recovery crash-loop guard; a GL version guard; swallowed errors logged.

## Phase 126: A safety net that catches

A TSan job; coverage; a Clang libFuzzer job and new fuzz targets; `/W4`
measured; translations in packages; CI cache and timeouts.
