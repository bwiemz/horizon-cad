# Milestone 10 — Scales to real models (Phases 136–138)

The product-completeness roadmap
(`docs/superpowers/specs/2026-09-24-product-completeness-roadmap.md`)
answers the audit's scaling findings here:

- **S1**: the 2D view rebuilds every vertex every frame, with one draw call
  per circle and arc, and re-paints and re-sends a full-window text image.
- **S2**: adding a feature builds the model two or three times, the first on
  the GUI thread; open always builds on the GUI thread; tessellation is never
  cached.
- **S3**: the DOF analysis is a dense SVD on the GUI thread after every edit.
- **S4**: the undo stack is unbounded; move commands clone every constrained
  entity; resolved assembly parts are never released.

Each phase measures before and after, and its tests count work done, not
time alone. Timing tests run only where timing means something
(`HZ_SKIP_WITHOUT_TIME_LIMITS`).

## Phase 136: A 2D view that scales

### As built

- **A drawing cache** (`ui::DrawingCache`). The 2D view draws from it: the
  drawing's lines, circles, arcs, polylines, splines, ellipses, hatches,
  dimensions and placed blocks as line vertices, batched by pen (colour,
  width, line type), and its texts.
  - It is built once, then kept while nothing it is built from changes. A
    frame that only moves the view or the cursor builds nothing.
  - Circles and arcs go into their pen's batch with the lines. Each was a
    draw call of its own.
  - It covers everything `renderEntities` drew, with the same colours:
    ByLayer, the selection's orange, the DOF colours and hidden layers.
- **What tells it the drawing changed** (the stamp `update` compares):
  - `DraftDocument::revision()`. It moves with every change made through the
    drawing: add, remove, replace, restore, clear, re-index, set the
    dimension style, and `updateEntityBounds` after a change in place.
    - Values come from one process-wide counter, and a copy takes a new one.
      So no two drawings ever share a revision, even one made at the address
      of a closed one.
  - The document's undo revision, for commands that change an entity in
    place without re-indexing it (a colour, say).
  - `SelectionManager::revision()`, new. It moves only when the selection
    changes.
  - The count of DOF analyses.
  - The drawing and document themselves, so editing a sketch or switching
    tabs is a change.
  - The layers and the dimension style are small, and are handed out by
    mutable pointer. The cache keeps a copy of each and compares it every
    frame (`LayerManager::operator==`, new).
- **The live drags re-index.** A sweep of every in-place change found three
  tools that change the drawing mid-drag with no command and no signal:
  MoveTool, StretchTool and Polyline Edit.
  - Each now calls `updateEntityBounds` as it drags. That also keeps
    snapping and picking right during the drag.
  - The Select tool's grip drag already replaced the entity, which moves
    the revision.
- **On the GPU.**
  - Each batch has a buffer (`GLRenderer::LineBuffer`). It is sent once
    after a build (`uploadLineBuffer`), not every frame.
  - A frame draws the batch's runs for the chunks in view
    (`drawLineBuffer`).
  - The buffers are released with the GL context.
- **View culling.**
  - The cache cuts the drawing into chunks: a grid over its entities'
    middles, up to 32 × 32 and about 256 entities each.
  - Each batch keeps its vertices in chunk order. Each chunk's extent is
    grown from the vertices actually emitted, so an entity reaching out of
    its chunk still counts.
  - A frame leaves out a chunk only when all its corners lie beyond one and
    the same side of the view, tested in clip space. That holds for any
    camera, a perspective one looking across the plane included. The runs
    of the chunks in view are joined where they meet.
- **The text overlay is painted only when what it shows changes.**
  - The stamp: the view-projection matrix, the size, the device pixel ratio,
    the document and the cache's build count. Annotations follow the
    selection through the cache.
  - The image is reused at the same size, and the texture is updated in
    place (`glTexSubImage2D`) only after a paint.
  - Texts well off the view are skipped.
  - A font is made only when the size changes; it was made for every text.
- **Found on the way, and fixed.** A new GPU test paints a real viewport and
  checks its pixels. It found two faults as old as the code:
  - **The grid hid the drawing.**
    - The grid writes the depth of its plane, and the drawing lies on that
      plane. So wherever the grid's lines were close together, the drawing
      lost the depth test to it: on this Linux/NVIDIA machine, all of it.
    - The grid is a reference, and no longer writes depth. The axis
      indicator in the corner, hidden the same way, shows again.
  - **The text overlay was upside down.**
    - The quad sampled the image's first row, its top, at the bottom of
      the view. Texts showed mirrored across the view from what they
      label, and the view cube and view name were upside down at the
      bottom.
    - The view cube's click targets were always at the top, so a click on
      the cube as drawn missed it.

  The sweep for in-place changes found three more:
  - **Stretch undid itself as it was made.** It put the entities back, then
    pushed grip moves whose first execute takes the state as it finds it. A
    stretch showed only after an undo and a redo.
  - **Another tool chosen mid-drag** left the Select tool's grip drag or
    Polyline Edit's vertex drag in the drawing, with no command to undo it.
    - The Select tool had no `deactivate`.
    - Polyline Edit's `deactivate` said it restored, and did not.
    - Both now cancel, as Escape does.
  - StretchTool looked for each stretched entity in the whole drawing on
    every mouse move. It now looks each one up by id.

### Measured

A drawing of 100,000 lines, 20,000 circles and 20,000 arcs, in a Debug build:

| | Before | After |
|---|---|---|
| Built per frame | every frame: 0.37 s, and 3.56 M vertices sent to the GPU | once, until it changes |
| A frame that changes nothing, zoomed into a corner | the same 0.37 s | 0.08 ms; 26,000 of the 3.56 M vertices drawn, nothing sent |

A frame of it with nothing changed must take under 1 ms in Release (20 ms
in Debug).

### Tests

7 new:
- The cache:
  - it builds once for many frames, and once for each change: a line moved
    in place, an add, an undo, a selection, a layer colour, the dimension
    style, the DOF analysis, a sketch edited, another document;
  - circles and arcs go in their pen's batch; the selection's colour; a
    hidden layer;
  - only the chunks in view are drawn, and all that is in them, in
    orthographic and perspective, and none looking away;
  - a frame of the 140,000-entity drawing, timed.
- The overlay is painted once over several frames, and again on a pan, an
  edit, a resize and a selection.
- `DraftDocument`'s revision moves with each change, and no two drawings
  share one.
- On a real OpenGL (`ViewportGraphicsTest`, run with a display, skipped on
  the offscreen platform):
  - the line is drawn over the grid, where it is;
  - it follows a pan and a drag;
  - a text is drawn where it is, not mirrored;
  - a chunk not drawn before is drawn once in view.

Also:
- Stretch is kept, and undone (`ToolEditsTest`).
- Another tool mid-drag puts the drag back, for grips and polyline vertices.

### Not done

- Circles and arcs keep 64 segments a turn at any zoom; a large circle,
  zoomed into, shows its facets.
- Text is still painted by QPainter on the CPU whenever the view moves.
  Panning a drawing full of text repaints it every frame. Text on the GPU
  would end that.
- The DOF colours are still those of before a live drag, until it ends
  (Phase 138 moves the analysis off the paint path).
- The cache keeps a CPU copy of the vertices (16 bytes each) beside the
  GPU's.

## Phase 137: The GUI thread stays free (planned)

- **One build per added feature.**
  - Count the builds (`Document` gets a build counter for tests).
  - The audit found an add builds two or three times, the first on the GUI
    thread; the count will say where. It should build once, on the worker
    when the last build was slow, as `rebuildFeatureTree` already decides.
- **Open and save off the GUI thread.**
  - Opening a part parses and then builds on the GUI thread
    (`rebuildScene` builds a model that was never built). Parse and build on
    the worker; show the document as building meanwhile.
  - The audit found save serialises on the GUI thread. Snapshot there, then
    serialise and write on the worker.
- **Cached tessellation.**
  - `rebuildScene` tessellates the solid every time it runs: every
    selection change, every sketch-mode change.
  - Keep the mesh with the solid it came from, and tessellate only a new
    solid.
- **Tests:**
  - the build count of an add;
  - a timer that fires while a large part opens;
  - the tessellation count across scene rebuilds that do not change the
    solid.

## Phase 138: Bounded memory (planned)

- **An undo limit.**
  - `UndoStack::setLimit` by count, from a preference, with a sensible
    default.
  - The oldest steps are dropped. When the saved state's step is dropped,
    the document stays modified until the next save.
- **The DOF analysis off the paint path.**
  - `recomputeDOF` runs the solver's dense SVD inside `paintGL`, after
    every edit.
  - Run it after a change, off the paint path: sparse where the system is
    large, on a worker past a size. The view draws the last result.
- **Moves that clone only what the solve changes.** Move commands snapshot
  every constrained entity (`ConstraintSolveHelper`). Keep only those the
  solve moved.
- **Shared meshes between assembly instances.** Each component's scene node
  copies its part's mesh. Instances of one part should share one mesh.
- **Released parts.**
  - `DocumentManager` keeps every resolved part for good.
  - Hold them weakly, so a part no assembly uses is released.
- **Tests:**
  - the undo stack stays within its limit and the modified state stays
    right;
  - the analysis does not run in a paint;
  - instances share a mesh;
  - a part is released when no assembly holds it.
