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

## Phase 137: The GUI thread stays free

### As built

- **A feature added is built once.**
  - `addModelFeature` built the whole model on the GUI thread to try the
    feature, took it out again, pushed its command, then built again in
    `rebuildFeatureTree`: two builds for every add, the first always on the
    GUI thread.
  - Now the command goes in at once and the model is built once, by
    `rebuildFeatureTree`, which puts a slow build on the worker. The
    command appends and clears the rollback, as the trial did, so that one
    build is the model the trial built.
- **A feature that fails itself is withdrawn** (a Cut that would leave
  nothing, an Intersect of bodies that do not touch).
  - The add leaves a pending mark: its document (held weakly), its step,
    its feature, and the undo revision it left.
  - When that document's build is shown or applied (`settlePendingAdd`, from
    `showBuildResult` and `onRebuildFinished`), the feature is refused if
    nothing has been done since and it is the failing one. It is taken back
    with `UndoStack::withdraw`, new: undo the newest step and forget it,
    with nothing left to redo. The part as it was is then built again.
  - This works on a worker too: the refusal comes when the build does.
  - Anything done before the build is seen (an undo, another step) settles
    it: a feature still there stays, failing like any other.
- **A part opened is built on a worker.**
  - A part came in unbuilt, and `rebuildScene` built it on the GUI thread
    as the tab opened.
  - Now its tab's build time is marked unknown (`kBuildTimeUnknown`). Auto
    treats that as slow, so the first build goes to the worker, and
    `rebuildScene` no longer builds a model the worker is building.
- **A large file is read on a worker.** A part or DXF drawing of at least
  1 MB (`kWorkerImportBytes`, as STEP import uses) is read on a worker
  (`readFile`, `startOpen`), into a document of its own.
  - When it has been read, its tab is added. `DocumentManager::adoptPart`
    registers a part as `openPart` would, or returns the one opened
    meanwhile. `adoptDocument` registers a drawing.
  - One file is read at a time. Cancel drops the result once it is read,
    because reading cannot be stopped midway.
  - 100,000 lines took 3.1 s to read in a Debug build.
- **Tessellated once for each build.** `rebuildScene` tessellated the solid
  every time it ran: every tab shown, every sketch opened or closed, every
  undo. Each tab now keeps its mesh with the build it came from
  (`Document::builds()`, new), and tessellates only a new build.

### Tests

7 new:
- `UndoStack::withdraw`: undone and forgotten; only the newest step can be
  withdrawn; a saved state that had the step cannot be reached again.
- In the window (`GuiThreadFreeTest`):
  - an add builds once;
  - a feature that fails itself is withdrawn, with nothing to redo, here and
    on a worker (where it is in the tree while its build runs);
  - a part opened is built on a worker, and at once with RebuildMode Never;
  - the model is tessellated once for each build, not again for a sketch
    opened and closed;
  - a large part and a large DXF are read on a worker, their tabs added when
    read, one at a time, and a file already open shows its tab.

Each new window test was seen to fail on the code before it, bar the
withdrawal here, which keeps what the old refusal did.

### Not done

- **Save stays on the GUI thread.** For 100,000 lines in a Debug build:
  building the JSON tree takes 916 ms, dumping it 477 ms, writing it 5 ms.
  - The tree reads the document, so it can only be built off the GUI thread
    from a copy, and making the copy costs about as much.
  - Moving only the dump saves a third, and needs the saved state recorded
    at the undo depth the save began from. It is left for when saving gets
    faster itself.
- Reading a file cannot be cancelled while it runs.
- A refused feature is built away again: two builds for a refusal, as
  before, one for an add.

## Phase 138: Bounded memory

### As built

- **The DOF analysis, sparse and by cluster** (`SketchSolver::analyzeDOF`).
  - It built the whole Jacobian dense (m × n) and took its SVD after every
    edit. For a chain of 400 lines that was 33 s in a Debug build; for 3,000
    constrained lines the matrix alone would be 860 MB.
  - Now each constraint's Jacobian rows are taken in its own entities'
    columns only, into a small scratch buffer, and their non-zeros kept.
  - Parameters that one equation ties together are joined (union-find), by
    equation rather than by constraint: a coincidence's x and y fall in two
    clusters, so a chain of lines breaks into many small clusters.
  - Each cluster's rank comes from a dense SVD when it has up to 96
    parameters, and a sparse QR (COLAMD) above that.
  - The 400-line chain now takes 7 ms, and 3,000 lines with 12,000
    parameters take 84 ms, both in Debug.
  - **Each cluster has its own status.** An over-constrained corner no
    longer turns the whole sketch red, nor a free one the whole sketch
    green. An entity is:
    - over-constrained when an equation on it is in an over-constrained
      cluster, or cannot be met;
    - free when a parameter of it is tied by nothing, or is in a cluster
      with freedom left;
    - fully constrained otherwise.
  - The freedom left matches the whole Jacobian's rank, tested on chains of
    every length and mix of constraints.
  - `ParameterTable` now looks entities up in a hash (it searched them
    all), and `applyToEntities` makes one pass, not one per entity.
- **Off the paint path.** `paintGL` no longer runs the analysis. When it is
  due (`ViewportRenderer::dofStale`), a paint queues it to run just after,
  and paints again. The frame shows the change at once; the colours follow.
- **An undo limit.**
  - `UndoStack::setLimit` drops the oldest steps past the limit, at once
    and on every push. A saved state among the dropped steps can no longer
    be reached, so the document stays modified.
  - Preferences ▸ Undo steps: default 1,000, 0 for no limit. It applies to
    every open document and to each new one.
- **Solves keep only what they moved.**
  - `ConstraintSolveHelper::solveAndApply` cloned every constrained entity
    before and after every solve, and each move kept both copies for undo.
    Now it compares the parameters before and after (within rounding) and
    clones, applies and re-indexes only the entities the solve moved.
  - Its restore on a failed solve went. The entities are untouched until a
    solve succeeds, so it had nothing to restore.
  - Its after-snapshot loop, quadratic in the entities, went too.
- **Shared meshes.**
  - A `SceneNode` shows a `shared_ptr<const MeshData>` (`shareMesh`).
  - `GLRenderer` keeps one GPU buffer for each mesh, not each node. Each
    entry holds its mesh, so the address it is found by cannot become
    another mesh's while the entry exists.
  - Every instance of an assembly's part shares one mesh, tessellated once
    (`DocumentManager::sharedMesh`, weak, versioned by the part's build or
    the file's time). Each instance had tessellated its own, and the scene
    copied it again for every node.
  - A part tab's mesh (Phase 137) is shown as it is, not copied.
- **Released parts.**
  - A part opened only for an assembly's components is held by them alone.
    It is found by its path while one holds it, and released when none
    does.
  - It is no longer among `documents()`. `DocumentManager` kept every
    resolved part for good.
  - Opened in a tab too, it is kept. Its file is still watched for changes.

### Tests

10 new or extended:
- The DOF analysis:
  - each cluster's own status: pinned, free and over-constrained lines
    side by side;
  - the freedom agrees with the whole Jacobian's rank, over 24
    configurations;
  - 3,000 lines, timed.
- `dofStale` before and after an edit.
- The undo limit: the oldest steps go, and the saved state stays right.
  The preference is kept and applied to the open document.
- A solve keeps only the moved entity.
- A shared mesh is listed once by the scene.
- Two instances of a part share its document and its mesh. The part is
  released when they let go, and kept when a tab opens it (the unit test
  and the multi-document integration test).
- On a real OpenGL: the solid is drawn, and drawn again from the same mesh
  after the scene is rebuilt.

### Not done

- The solver itself still builds a dense Jacobian and a dense QR every
  iteration. A move in a large constrained sketch is slow; the solve should
  go sparse the same way.
- A single huge cluster (one densely constrained mechanism) still runs a
  sparse QR on the GUI thread, after the paint rather than in it.
- Assembly instances share a mesh, but each is still its own draw call;
  instanced drawing (the InstanceBatcher) is not wired to the viewport.
