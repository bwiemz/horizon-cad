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

### What the audit found

- **Shell built a different part.** It made a new cup from two faces: the
  removed cap and an opposite cap with the same number of corners.
  - Everything else on the part was dropped: holes, bosses, pockets.
  - Only the first of several open faces was used.
  - "Inward" was the side facing the profile's centroid. On an L-shaped cap
    that turned the inner-corner edges outward, and the cavity broke through
    the walls.
  - The cavity's walls were always straight down, so a tapered part got the
    wrong wall thickness.
  - The thickness limit was a distance from the centroid, which refused
    walls the profile could take.
- **Draft and Chamfer compared with a centroid too.** Draft judged each face
  against its body's centroid, so hole walls and the inner faces of an L
  leaned the wrong way. Chamfer judged each face against its own centroid,
  which is wrong on non-convex faces.
- **A failing feature emptied the part.** The build returned no solid, the
  viewport went blank, and `rebuildScene` then rebuilt the failing model on
  the GUI thread at every redraw, tab switch and save.
- **Surface area was too high on non-convex faces.** A face without holes
  was fanned from its first corner and the triangles' areas added unsigned.
  An extruded U counted 116 for its 52-square cap.
- **Each three-edge fillet corner carried a whole sphere.** The tessellator
  drew all of it, so every STL and glTF export had seven eighths of a ball
  inside each blended corner.

### As built

- **Shell:**
  - It opens one face. More are refused.
  - It hollows a right prism only: N+2 faces, 2N vertices, quad sides, no
    holes, the base straight below the top. Anything else is refused with
    one message saying why.
  - The inner profile is offset by winding, `axis × edge` on the
    counter-clockwise cap.
  - The profile is then checked: every edge keeps its direction, the polygon
    stays simple, and it stays inside the outer profile. This replaces the
    centroid "inradius". An L with 3-wide arms now takes a 1.4 wall and
    refuses 1.5, where its arms close.
  - The header now describes what Shell does.
- **Draft** takes each face's outward normal from its loop's winding, turned
  over for a body wound inside out. That is judged by the body's signed
  volume, now `topo::signedVolume`, which Pattern shares.
- **Chamfer** uses FilletOp's rule: walking a loop with its normal up, the
  face lies to the left (`topo::loopNormal`). This holds however the body is
  wound.
- **A failed build keeps the part as it stood.**
  `FeatureTree::buildWithDiagnostics` replays the features up to the failing
  one, so a failed build pays twice and a successful one nothing.
  `Document::needsBuild()` counts a failed build as built until the tree
  changes. `rebuildScene` builds only a model that has no solid and has not
  been built.
- **Save caches the part as it is.** Save builds any model that is behind its
  features before writing the tessellation cache. It used to build only when
  there was no solid, so during a rebuild on a worker it cached the part as
  it was before the edit, and lightweight assembly loads showed that. (Found
  in review; the gap predates this phase.)
- **Mass properties** triangulate every face of more than three corners
  (`BoundaryMesh::triangulatePolygon`), not a fan. Volume is unchanged.
- **Fillet corners** carry `NurbsSurface::makeSphereOctant`, an exact
  rational biquadratic eighth of the ball facing the vertex. The whole
  sphere remains the face's analytic surface, which mates and dimensions
  read.
- **The Boolean header** no longer says holes are ignored and split faces are
  never merged back.
- **Tests:** 11 new.
  - Shell:
    - an asymmetric L, with its exact cup volume;
    - the thickness limit, 1.4 accepted and 1.5 refused;
    - a drilled plate refused;
    - two open faces refused.
  - A failing feature:
    - leaves the part before it;
    - leaves nothing when it is the first feature;
    - is not rebuilt until the tree changes.
  - Mass properties: a U-channel's area.
  - A corner blend's mesh area matches the part's.
  - The octant patch itself.
  - Save during a rebuild on a worker caches the part as it is.
- **Not done** (Milestone 11):
  - a real offset-based shell;
  - Draft on selected faces;
  - curved STEP faces are still drawn untrimmed;
  - the validator still cannot see faces crossing each other.

## Phase 124: Hostile files, round 2

### What the audit found

- **Hatch boundaries were garbled.** `parseHatch` took every 10/20 pair in
  the entity after the first as one polygon. That merged the outer boundary,
  its islands, and the seed points after them into one outline, and turned
  edge-defined boundaries (lines, arcs, ellipses, splines) into their
  endpoints. It said nothing, and no test read a hatch.
- **Nested blocks multiplied without bound.** Blocks inserted into blocks are
  flattened. The depth was capped at 16, but not the breadth: ten inserts of
  a block of ten inserts, eight levels deep, is 10⁸ entities from a few
  kilobytes, made on the GUI thread.
- **Lineweights were not DXF lineweights.** Group 370 was written as
  `width × 100` (1.5 as 150), not one of the 24 values DXF allows, and the
  layer table wrote none, though the reader takes it.
- **A duplicate ID hid an entity** (from the Phase 122 review). A native file
  with two entities under one ID gave the second the first's place in the id
  lookup. The first could then be neither selected nor deleted.

### As built

- **Hatch boundaries are read path by path**, in the file's order:
  - 91 paths, each starting at 92;
  - a polyline path's 72/73/93 and vertices, with their bulges followed as
    arcs;
  - an edge path's 93 edges, each by its 72 type: lines; circular and
    elliptic arcs, including clockwise ones, sampled every π/16; splines,
    by their control polygon.

  The hatch keeps the path the file marks as outer (else the largest). The
  report says "islands inside it left out" when there were more, and
  "curved boundary brought in as segments" when a curve was sampled.
- **Flattening has a budget** of 2,000,000 placed entities per import. It is
  `DxfFormat::setMaxFlattenedEntities`, for tests and embedding applications.
  Past it, flattening stops, and the report lists what was left out.
- **Lineweights:** `dxfLineweight` writes the nearest allowed value. Layers
  write theirs: a default-width layer is written as -3 (default), which reads
  back as the default.
- **A duplicate native ID** gives the later entity a new one
  (`DraftEntity::newId()`), and the report says so.
- **The other readers' output is proportional to their input.** The native
  format's counts are capped (Phase 100), and the STEP reader has no
  instancing. So the DXF flattening budget is the entity budget that was
  missing.
- **Tests:** 5 new.
  - DXF:
    - a hatch with an island and a seed point;
    - an edge boundary with a half circle;
    - nested blocks cut at a budget of 1,000;
    - every 370 written is valid, and the layer widths round-trip.
  - Native: a duplicate ID.

  Two new fuzz seeds cover every hatch path and edge type, and nested
  inserts.
- **Not done:**
  - A hatch still has one boundary. Holding islands needs `DraftHatch` to
    take more than one loop.

## Phase 125: Background work that stops

Cooperative cancel for STEP import and interference; wedged-job and
trial-build exception safety; autosave failures shown to the user; a
recovery crash-loop guard; a GL version guard; swallowed errors logged.

## Phase 126: A safety net that catches

A TSan job; coverage; a Clang libFuzzer job and new fuzz targets; `/W4`
measured; translations in packages; CI cache and timeouts.
