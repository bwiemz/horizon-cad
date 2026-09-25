# Changelog

All notable changes to Horizon CAD are recorded here. The project was built
phase-by-phase against the roadmap in
[docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md](docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md);
this file summarizes that work by era. Each phase shipped as an honest core
slice — where the roadmap named a heavy third-party dependency, an in-house
implementation was built instead to keep CI lean and the code testable
headless. Those deviations (STEPcode/OCCT, Embree, OpenCAMLib) are documented
in [the era findings note](docs/superpowers/notes/2026-07-03-era2-roadmap-findings.md).

**Versions.** The version is set in one place, `project()` in the top-level
`CMakeLists.txt`, and everything else reads it from there: the About box,
`horizon --version`, the log and the installer. It is **0.1.0**, and no
version has been released yet. The 80-phase roadmap was recorded here under
"1.0.0", but nothing was ever built or shipped as 1.0.0: that section now
carries the version the code had, and the work after it is post-roadmap
work, not "post-1.0".

## Unreleased — Professional workflows, Milestones 13–14 (Phases 145–149)

- **The Linux build opens a window (145).** It was built with a Qt that had
  no platform plugin and no font engine: every headless test passed, and the
  application could not start. Qt is now built with FreeType, HarfBuzz and,
  on Linux, fontconfig and xcb. The release starts the application with
  `horizon --self-test` before packaging it.
- **`horizon --self-test`** checks that the window opens and the viewport
  draws, and says what it found.
- **Polyline Edit's Join** no longer leaves a segment of no length where
  the two polylines met.
- **Tab cycles the kind of dimension** being placed (horizontal, vertical,
  aligned; radius or diameter), as the tools always meant it to. The view
  no longer loses the keyboard to it.
- **Clean drawing views (147).** A drawing of a part no longer draws the
  seams between a curved face's facets. It draws a curved face's outline as
  its silhouette, a rim on its circle, and marks where a fillet meets a face
  as a tangent edge. Views of finely faceted parts are drawn up to 40 times
  faster. The Front view now looks from the front: it showed the part from
  behind, mirrored.
- **The assembly commands are their own workbench (146).** They moved out
  of the 5,600-line main window into `AssemblyWorkbench`. It reaches the
  window through a narrow `WorkbenchHost` and is tested without one. No
  behaviour changes. The part commands follow.
- **CI now runs what it claimed to.** It ran no window test (181), none in
  the Linux Release job, and no OpenGL test. All of them run now, each job
  counts what it ran, and coverage floors hold.
- **Drawing sheets (148).**
  - Drawing ▸ New Drawing from Part opens a tab with the part's four
    standard views on paper, at the largest standard scale that fits, with
    a border and a title block.
  - Save it as a `.hzdwg`. It names the part, so it opens again drawn from
    the part as it is then.
  - Set the title block, the paper and the scale in forms.
  - An open sheet is drawn again when its part is saved, or changed by
    another program.
  - PDF and SVG export of a sheet start on its paper at 1:1.
  - A view drawn at a scale states its lengths at 1:1.
- **Sections and details on a sheet (149).**
  - Drawing ▸ Add Section View cuts through a view where you say. The
    section is hatched, captioned "SECTION A-A", and its cut is drawn on the
    view with arrows and letters.
  - Drawing ▸ Add Detail View: click the centre and the radius on a view.
    The detail is circled there and drawn larger, at a standard scale.
  - New views go where the sheet has room. Move View moves one by two
    clicks. Remove View removes one, with the views taken from it.
  - Choosing another scale keeps the sections and details.
- **Dimensions and centre lines on a sheet (149).**
  - Drawing ▸ Add Dimension: click edges to dimension them. You get a
    circle's diameter, an arc's radius, or an edge's length, stated at full
    size.
  - They are measured from the part each time the sheet is drawn. When
    the part changes, they change with it. One whose edge is gone is said.
  - A partly hidden edge is dimensioned end to end. The dimension used to
    cover only its first visible run.
  - Holes and bosses get centre lines: a cross seen end-on, the axis seen
    side-on.
  - Drawing ▸ View Properties shows or leaves out each view's hidden edges,
    tangent edges and centre lines.
## Unreleased — Product completeness, Milestone 12 (Phases 143–144)

- **Placing components (143).**
  - An Assembly menu, and an assembly tree beside the feature tree, listing
    the components and mates.
    - From the tree: remove, suppress, rename; edit a mate's distance or
      angle, or remove it.
    - Removing a component takes its mates with it. A mate left behind made
      every later move fail.
  - Move and Rotate Component, each one undo step, with the mates solved
    again. A component held by a Fixed mate stays put, and says so.
  - A component is placed beside the others when inserted, not on top of
    them at the origin.
  - A component can be clicked as soon as it is inserted. Add Mate lists
    each face once, said by what it is, and takes the faces clicked. A
    clicked cylinder's side used to be matched to whichever of its facets
    came first.

- **Living assemblies (144).**
  - A part saved in its tab, or changed on disk by another program, shows
    changed in every open assembly that places it. The mates are solved
    again, so what sits on it moves with it. Before, an assembly showed a
    part as it was when the assembly opened it.
  - A part edited in its tab and closed without saving leaves the
    assemblies as its file is. Its unsaved edits used to stay in the
    components that shared its document.
  - An undo in an assembly keeps the parts as they are now, and only puts
    back where they were.
  - A part or drawing open in a tab and changed by another program is read
    again. If it has unsaved changes, you are asked first, and keeping them
    is the default.
  - Open Part opens the clicked or chosen component's part in its tab.
  - Bill of Materials lists each part with its count and exports CSV. A
    file spelled two ways is one line.

## Unreleased — Product completeness, Milestone 11 (Phases 139–142)

- **Stable names (139).**
  - A fillet, chamfer or mate stays on its edge or face through edits
    elsewhere.
    - Two boxes in one part no longer share names.
    - A fillet or chamfer leaves the names of other edges as they were, so
      a second fillet finds its edge.
    - Revolve, Sweep and Loft faces are named after the profile element
      they come from, and keep their names when the facet count changes.
  - A curve is one edge and a curved face one face. Clicking a cylinder's
    rim picks the whole rim, and filleting it rounds all of it. The edge
    and face lists show one row for each, and no hidden seams.
    - A rounded or chamfered rim is one face, and a cylinder a cut parts
      in two is still one side.
  - Documents saved before keep their names. A document saved now needs
    this version or later (format 19).

- **Fillet and chamfer at any angle (140).**
  - Fillet and chamfer work on edges between flat faces at any angle, where
    only square corners worked. On a concave (inside) edge they add
    material. The volumes are exact.
  - A fillet or chamfer that ends on a slanted face now meets it. The
    chamfer was refused there, and the fillet built a face that was not
    flat.
  - A part made of several bodies can be filleted; it failed. Chamfers
    already worked.
  - A three-edge corner that is not square, or a chain of fillets across
    edges at different angles, is refused with the reason.

- **Curved faces measured as curved (141).**
  - Mass Properties reports the part as modelled, which is its facets and
    what Booleans and export use, and as designed. The designed values
    measure each face on the curved surface it approximates. A 32-sided
    cylinder read 0.6 % light; now the cylinder is exact.
    - A cylinder, cone, sphere, torus, revolve and filleted box match their
      closed forms to 1e-9.
    - Where a face has no curved surface recorded (a mesh), or two faces'
      surfaces do not meet, the dialog says so.
    - A large part is measured in the background, and closing the dialog
      stops it.
  - STEP cones, spheres and tori are read; they were refused. A cone's
    angle in degrees is read as degrees.
  - An imported cylinder's caps, each bounded by one circle, had no
    polygon, so its volume and Booleans were wrong. Imported curved faces
    are now built in facets that record their surface, like the kernel's
    own.

- **Boolean robustness (142).**
  - A Boolean either gives a valid solid that conserves volume or says why
    it cannot. Tests cover random placements, parts a million millimetres
    from the origin, very small and very large parts, and faces in exact
    contact.
    - Far from the origin, a Boolean could turn a part inside out:
      Subtract gave the intersection.
    - A very small part's Boolean could fail to join up.
  - Tolerances scale with the parts.
  - A Boolean on a finely faceted part no longer risks overflowing the
    stack.
  - A face cut by more than 60 holes is put back together, where it was
    left in hundreds of fragments.

## Unreleased — Product completeness, Milestone 10 (Phases 136–138)

- **A 2D view that scales (136).**
  - Large drawings pan and zoom without rebuilding anything. What is drawn
    is built once and kept until the drawing changes. It is drawn in one
    batch per colour and line style, circles and arcs included, and only
    the part in view is drawn.
    - Measured on 140,000 entities in a debug build: a frame that only moves
      the view took 0.37 s to prepare, and now takes 0.08 ms.
  - Text over the view is painted again only when it changes.
  - Fixed:
    - The grid could hide the drawing. Lines on the drawing plane lost to
      the grid wherever its lines were close together; on Linux with NVIDIA,
      everywhere. The axis indicator in the corner shows again for the same
      reason.
    - Text over the view was drawn upside down, mirrored across the view
      from what it labels, and the view cube and view name were upside down
      at the bottom.
    - Stretch undid itself as it was made; the stretch showed only after an
      undo and a redo.
    - Choosing another tool in the middle of a drag left what was dragged
      moved, with nothing to undo it.

- **The window stays free (137).**
  - Adding a feature builds the part once, and on a worker when builds are
    slow. It was built twice, the first time always while the window
    waited. A feature that fails, such as a cut that would leave nothing, is
    still refused and leaves nothing to undo or redo.
  - A part you open is built in the background, so a large part no longer
    freezes the window as it opens.
  - A part or DXF file of 1 MB or more is read in the background, and its
    tab appears when it has been read.
  - Opening or closing a sketch, or switching tabs, no longer tessellates
    the part again.

- **Bounded memory (138).**
  - Large constrained sketches stay responsive. The analysis behind the
    constraint colours took 33 s after every edit of a 400-line chain (in
    a debug build), and now takes 7 ms. It no longer runs while the view is
    drawn.
  - The constraint colours are per cluster. An over-constrained corner no
    longer turns the whole sketch red, and a free one no longer turns it all
    green.
  - Preferences ▸ Undo steps limits the history each document keeps
    (default 1,000).
  - A move in a constrained sketch keeps, for undo, only what the solve
    moved. It kept two copies of every constrained entity.
  - Every instance of a part in an assembly shares one mesh, and the GPU
    holds it once. A part opened only for an assembly is released when no
    component uses it.

## Unreleased — Product completeness, Milestone 9 (Phases 131–135)

- **Sketches on planes (131).**
  - Model ▸ New Sketch makes a sketch on the XY, XZ or YZ plane, on a flat
    face of the part, or on a datum plane. While you edit it, the view looks
    straight at it and every drawing tool draws into it, in its own
    coordinates. Model ▸ Finish Sketch, or Extrude or Revolve, ends it.
  - The feature tree lists the sketches: double-click one to edit it again.
    The one chosen there is what Extrude and Revolve use.
  - A profile can have holes, and several separate regions. A plate with
    holes extrudes in one step, and a revolve's section can have a window
    in it.
  - Text, dimensions and hatches in a sketch or drawing no longer stop
    Extrude; they are notes, not part of the shape.
  - Revolve turns about the sketch's own vertical or horizontal axis. It
    was always the world's Y.
  - Sketches are saved with their constraints; they were lost. Documents no
    longer carry an empty "Default Sketch", and scripts see no sketches in a
    new document (`sketch_count()` is 0, not 1).
  - Renaming or removing a layer reaches entities in every sketch.
  - Shell's face list named faces by their inside: "facing up" was the
    bottom. It opened the wrong face of any part not the same both ways.
- **Picking in 3D (132).**
  - Click a face or an edge of the part to choose it, and Shift-click to
    add or take one away. What the cursor is over is highlighted as you
    move.
  - Fillet, Chamfer and Shell start with what you clicked. New Sketch on a
    Face uses the face you clicked. Add Mate starts from faces clicked on
    two components.
  - The part now shows its own edges, not every triangle's, and a cylinder
    no longer shows a line between every facet.
- **The missing commands (133).**
  - Model ▸ Loft… and Sweep…, and Model ▸ Datum ▸ Plane, Axis and Point.
    Datums are drawn, and sketches can go on datum planes.
  - When a loft or sweep fails, you are told why: which section has too few
    corners, or where the path turns too tightly.
  - Editing a feature shows angles in degrees (a full revolve used to show
    as 6.2832), counts as whole numbers, and labels in words. Directions and
    axes can be changed too: an extrusion reversed, a revolve's axis moved,
    or along the face or edge you clicked.
  - Roll Back to Here and Roll Forward in the feature tree. Each is one undo
    step, and a saved part keeps where it was rolled back to.
- **Extrude and pattern options (134).**
  - Extrude goes to a distance, both ways (half each), through all of the
    part, or through all both ways, and can be reversed.
  - A pattern can repeat chosen features (a hole, a boss) rather than the
    whole part.
  - Primitives are placed: a base point and the way their axis points,
    both editable afterwards. They always stood at the origin.
- **Finding and seeing the part (135).**
  - Every modelling command is in the Model menu, so Ctrl+K finds it:
    primitives, Extrude, Revolve, the booleans, Fillet, Chamfer, Shell,
    Draft and the patterns were on the ribbon only. Extrude, Revolve,
    Fillet, Chamfer, Shell and Mass Properties have Ctrl+Shift shortcuts
    (E, R, F, C, H, M). The 2D Fillet and Chamfer say "(2D)".
  - Fit All frames the part. It looked only at the drawing, so a solid with
    no drawing was left out of view. In a narrow window it no longer cuts
    the part off at the sides.
  - View ▸ Orthographic, which stays orthographic when the window is
    resized, and Back, Bottom and Left views.
  - View ▸ Display: shaded, shaded with edges, or wireframe.
  - View ▸ Section Plane… cuts the view across X, Y or Z at an offset,
    edges included.
  - Model ▸ Mass Properties… gives the volume, surface area, centre of mass
    and inertia, and the mass for a chosen material.

## Unreleased — Product completeness, Milestone 8 (Phases 127–130)

- **Drawings to PDF and SVG (127).**
  - File ▸ Export ▸ PDF… and SVG… plot the drawing on a chosen paper and
    orientation, fitted or at a scale from 1:200 to 10:1, with line weights
    in millimetres, dash patterns, and text at its height. White lines,
    drawn on the dark screen, plot black; or everything can plot black. If
    the drawing does not fit the paper at the scale chosen, you are told
    before anything is cut off. Printing waits on adding Qt's print support.
  - Blocks now draw everything they hold, on screen too: text, hatches,
    dimensions and blocks inside blocks were left out.
  - Mirroring a block reference mirrors it. It was turned half round
    instead, so a door mirrored to swing the other way could come out
    unchanged. The mirror is kept in saved drawings and in DXF, where a
    mirrored INSERT now comes in as a block reference rather than exploded.
    Explode puts the pieces where the mirrored block drew them.
  - A block scaled by a negative number draws its arcs and text turned with
    it; they kept their angles.
  - Mirrored text reads left to right over the place its mirror image
    covers. It came out upside down, on the wrong side of its point.
- **Drawing to exact sizes (128).**
  - While a drawing tool waits for a point, type one and press Enter:
    `x,y`; `@dx,dy` from the last point; `@length<angle`; or a length alone,
    which goes toward the cursor. The status bar shows what is typed, and
    says why when it is not a point. Escape drops the typing, then the tool.
    Line, polyline, rectangle, circle, arc, ellipse and spline take them.
  - The line tool chains: each line starts where the last ended, until Enter
    or Escape.
  - Object snap (F3), grid snap (F9), ortho (F8) and polar tracking (F10)
    can be switched on and off, from Tools ▸ Drafting Aids or the status bar,
    and are kept between sessions. The status bar used to say "SNAP GRID"
    whatever was on.
  - A selected line's ends, length and angle, a circle's centre and radius,
    and an arc's centre, radius and angles can be typed into the property
    panel. Each is one undo step.
  - Selecting one ellipse (or text, spline or hatch) after another pushed
    "edits" of it onto the undo stack and marked the drawing modified. It no
    longer does.
  - Choosing a tool from the ribbon now puts the keyboard on the drawing, so
    typing reaches the tool.

- **Dimensions and text (129).**
  - Dimension ▸ Style… edits the dimension style: text height, arrow size
    and angle, extension gap and overshoot, decimal places, and the unit
    dimensions are shown in (mm, cm, m, in or ft), with or without it
    ("25.40 mm", `1.00"`). The model stays in millimetres. It is one undo
    step, and it is saved with the drawing. The style could not be changed
    before.
  - Dimension ▸ Continue goes on from the last horizontal or vertical
    dimension, each new one starting where the one before ended. Dimension ▸
    Baseline measures each from the first one's first point, a step further
    out. Enter picks another dimension to go on from.
  - The linear dimension takes typed points.
  - Text can have several lines: the text tool asks for them, the property
    panel edits them (the edit is taken when you leave the field), and they
    draw, select, plot and export one below the other.
  - A text of several lines is saved to DXF as one MTEXT and read back as one
    text. An MTEXT from another program comes in as one text too, unless its
    lines are spaced other than usual; then it is still one text per line,
    grouped.

- **Layers and blocks (130).**
  - Layers can be renamed from the layer panel. Everything on the layer,
    inside blocks too, goes with it, as does the current layer. A layer's
    line weight can be set by double-clicking its Width.
  - Create Block asks for the base point, offering the centre of the
    selection. Undoing it puts the entities back where they were in the
    drawing order (they went to the end), and redoing it brings back the
    same block reference rather than a new one.
  - The README's feature list claimed a few things the product does not do
    (custom hatch patterns, the system clipboard, a dimension style editor,
    shortcuts for every tool). It now says what it does.

## Unreleased — Product completeness, Milestone 7 (Phases 122–126)

The [product-completeness roadmap](docs/superpowers/specs/2026-09-24-product-completeness-roadmap.md)
continues from a fresh audit after Phase 121.

- **Crashes and hangs in 2D (122).**
  - Starting Insert Block while it was already active destroyed the running
    tool and then called it. It no longer does.
  - Removing an entity rebuilt the whole spatial index, so undoing a large
    DXF import could take minutes, and deleting a big selection seconds.
    Entities are now removed from the index in place.
  - Undoing a deletion put the entities back at the end of the drawing,
    drawn over everything else. They now go back where they were.
  - Finding an entity by its id meant scanning the whole drawing. Box
    selection, the grips drawn every frame, most editing commands and
    snapping did it once per entity. It is now a direct lookup.
  - Changing a text's height, content or alignment, closing a spline, or
    changing an ellipse left picking and box selection working from the old
    outline.
- **Operations that gave a wrong part without saying so (123).**
  - Shell rebuilt the part as a new cup made from two of its faces. Every
    hole, boss or pocket was dropped, and only the first of several open
    faces was used. It now hollows only a plain prism (two caps and straight
    sides) and refuses anything else, with the reason.
  - Shell, Draft and Chamfer decided which side was inside by looking at a
    centroid, which is wrong on L-shaped and other non-convex parts: a
    shelled L broke through its own wall. They now use the direction the
    faces are wound in.
  - When one feature failed, the whole part disappeared, and the window
    rebuilt the failing model on every redraw. The part now stays as it was
    before the failing feature, which is marked, and is rebuilt only when
    something changes.
  - Saving a part while it was being rebuilt in the background wrote the
    mesh of the part before the last change into the file, and assemblies
    loading it lightweight showed that. Save now builds the part first.
  - The surface area of non-convex faces was too high: a U-shaped cap
    counted 116 instead of 52.
  - Every three-edge fillet corner carried a whole sphere, which STL and
    glTF exports wrote out. They now write the corner alone.
- **Files that came in wrong, or could not be opened at all (124).**
  - A DXF hatch with islands, or one bounded by arcs, came in as one garbled
    outline: every coordinate in it, the islands' and the seed points'
    included, was read as a single polygon. Its boundary is now read path by
    path; the outer one is kept, arcs and curves are followed, and any
    islands are reported. A spline edge's fit data no longer ends its path.
  - A DXF of blocks inserting blocks, ten at a time and a few levels deep,
    could ask for a hundred million entities from a few kilobytes and run
    out of memory. Flattening now stops after 2,000,000 placements and says
    so, counting blocks within blocks as well as what they place.
  - DXF lineweights were written as any number (a width of 1.5 as 150),
    which strict readers reject. They are now the nearest of the 24 values
    DXF allows (never the hairline 0, which reads back as no width), and
    layers carry their widths too.
  - A native file holding two entities with the same ID made one of them
    impossible to select or delete. The second is now given a new ID that
    no other entity in the file holds, and the report says so.
- **Background work that did not stop, and failures nobody saw (125).**
  - Cancel did not stop a STEP import or an interference check running in
    the background: they ran to the end, and quitting waited for them. Both
    now stop.
  - If the system could not start a thread for background work, the work
    was never done and was waited for forever, along with every rebuild
    after it. It is now done in the foreground instead.
  - An autosave that could not be written (a full disk, a folder that is
    not writable), or could not start at all, was recorded only in the log.
    The status bar now says so until autosave works again.
  - A recovered document that crashed Horizon CAD was offered again at
    every start, with Recover as the default. When a document's recovery
    was followed by another crash, Recover is no longer the default and the
    message says why.
  - Recovered documents were not autosaved until the next autosave, and the
    crashed session's copies were deleted at once, so a crash in between
    lost them. They are now saved in the new session first.
  - On a graphics driver older than OpenGL 3.3, the viewport reported the
    problem and then made OpenGL 3 calls anyway. It now only clears the
    background.
  - A fillet radius or chamfer distance typed as "1.2.3" was taken as 1.2,
    and one typed as "." was dropped without a word. Both are now refused,
    and the prompt says so. The fillet prompt now shows the radius.
  - Seven empty `catch (...)` blocks that swallowed errors are gone.
- **A safety net that catches (126).**
  - CI runs the tests under ThreadSanitizer, now that models rebuild on a
    worker while the window is being edited.
  - CI measures line coverage and shows it by module in each run's summary.
  - CI fuzzes every reader of untrusted input with libFuzzer, a minute each,
    where it only replayed the seed files before. New fuzz targets cover the
    binary part format, plugin manifests, and the PDM's revision archive and
    check-out locks.
  - The Windows build counts its /W4 warnings, by code, in each run.
  - Packages must carry the translations. They were compiled only when Qt's
    own Linguist tools were found, which the release build never had; now
    any lrelease will do, a package build without one fails, and every
    build checks the catalogs are where the application looks for them.
  - The vcpkg cache is kept when a later step fails, so the next run does not
    rebuild Qt. Every job has a time limit, tests run in parallel, and the
    workflow can only read the repository.

## Unreleased — Production readiness, Milestone 6 (Phases 119–121)

- **The vault could give one document to two users, and lose a history
  (119).**
  - A check-out rewrote a shared lock file, so two users checking out at the
    same moment were both given the lock. Each lock is now a file of its own,
    created exclusively, and exactly one user wins.
  - A lock file that could not be read made every document look free. A
    revision archive that could not be read looked empty, and the next
    commit wrote over its history. Both now fail closed: an unreadable lock
    counts as held, and an unreadable archive refuses commits.
  - Revisions are hashed with SHA-256, and content is checked against its
    hash whenever it is read, pushed or fetched. Archives written with the
    old FNV-1a hash still load, verify and sync.
  - A second handle on an archive appends after the first's commit instead
    of overwriting it.
- **A script could reach a document after it was gone (120).**
  - The `doc` a script was given stayed in the interpreter after its run,
    pointing at a document the caller might already have destroyed. It is
    now removed when the run ends, and any copy the script kept raises an
    error if used later.
  - Scripting is off by default (`HZ_ENABLE_SCRIPTING`), because nothing
    sandboxes a script.
  - A plugin is checked again when it is loaded, not only when it is found.
    One that changed since the user enabled it, such as by adding a
    permission, has to be enabled again.
- **G-code that could hurt a machine, and FEA that analysed the wrong shape
  (121).**
  - A program now starts from a known state, loads its tool, and starts the
    spindle before it moves. Its first rapid climbs before it crosses. Every
    later rapid climbs before it crosses, or crosses before it descends, so
    none moves diagonally through the part. It stops the spindle at the end.
  - A program that could rapid through the cut, cut with the spindle
    stopped, or carries a feed of zero or a number that is not finite is
    refused, with the reason. So are toolpath parameters that would make one.
  - The analyses meshed a solid's bounding box, so a cylinder was analysed
    as the bar around it. They now refuse any solid that is not an
    axis-aligned box, and say why.
  - The maturity table in the README says which modules the application
    can reach.

## Unreleased — Production readiness, Milestone 5 (Phases 115–118)

- **The version was not one number, and the installer was never configured
  (115).**
  - The version is now set only in `project()`. It reaches the code through
    a generated header (the About box, `--version`, the log) and the
    installer through CPack.
  - The source revision is kept current on every build.
  - `cmake/CPack.cmake` shadowed CMake's own CPack module, so
    `include(CPack)` never wrote an installer configuration. The settings
    are now `CPackSettings.cmake`, included first.
  - The CHANGELOG's "1.0.0" is now 0.1.0, the version the code has always
    had.
- **A package that installs like an application (116).**
  - Horizon CAD has an icon: on the window, the executable, the installer
    and the Linux desktop.
  - On Linux it has a desktop entry, AppStream metadata and MIME types for
    its files.
  - A package now carries its translations, its licence, the third-party
    notices and the licence text of every library in it; on Windows, the Qt
    runtime too.
  - A script builds an AppImage with linuxdeploy.
  - A test installs the build and checks what a package would hold.
- **A release pipeline, and CI that no longer rebuilds Qt every time
  (117).**
  - The vcpkg binary cache restored nothing. It now keeps built packages
    between runs.
  - CI adds a Linux Release build with `-Werror`. It turned up a GCC false
    positive, now avoided.
  - A tag `vX.Y.Z` builds, tests and packages Windows and Linux (installer,
    AppImage and tarball), with SHA-256 checksums, into a draft release.
  - Dependabot keeps the workflows' actions current.
  - `docs/RELEASING.md` says how to cut a release.
- **How to take part (118).**
  - A security policy: vulnerabilities are reported privately, and the
    policy says that scripts and plugins are not sandboxed.
  - A code of conduct, the Contributor Covenant.
  - Issue forms for bugs and feature requests, and a pull request template
    with the CI gates as a checklist.
  - CONTRIBUTING describes the current build, tool API and tests, and how to
    pass every CI gate locally.
  - The licence is the GNU GPL v3 or later. `LICENSE` held only a fragment
    of it and now holds the full text; the README said MIT and now states
    the GPL. The About box and the Linux desktop metadata say so too.

## Unreleased — Production readiness, Milestone 4 (Phases 111–114)

- **Ten common shortcuts did nothing (111).** Ctrl+Z, Ctrl+Y, Ctrl+S,
  Ctrl+O, Ctrl+N, Ctrl+C, Ctrl+V, Ctrl+D, Ctrl+G and Ctrl+Shift+G were each
  bound twice, once to the menu and once to the ribbon, which Qt treats as
  ambiguous and ignores. Each is now one action, and a test checks every
  shortcut in the window.
- **The application forgot everything between sessions (111):**
  - File ▸ Open Recent lists the ten newest files;
  - the window's size and dock layout are kept;
  - Edit ▸ Preferences sets:
    - the autosave interval;
    - the language;
    - the grid snap spacing;
    - the snap reach;
    - the unit coordinates and measurements are shown in.
  - Help ▸ About shows the version, revision and build.
  - `horizon file...` opens files from the command line.
  - View reaches every dock.
- **Every command is exercised (112).** A smoke test triggers each of the
  window's 166 commands on an empty drawing, part and assembly, and on a
  selection, dismissing whatever dialog it opens. The drawing tools are
  tested through the viewport's own mouse handling: draw, select, delete,
  undo.
- **The viewport leaked GPU memory and did per-frame work it did not need
  (113).**
  - Every model change left the previous model's meshes on the GPU.
  - The constraint solver ran on every mouse move.
  - A picking pass that nothing read was drawn every frame.
  - On a high-DPI screen, dimension text was drawn at half resolution and
    scaled up.

  All four are fixed.
- **Long work froze the window (114).**
  - A model rebuild that takes seconds, such as a Boolean on a finely
    faceted part, used to freeze the window after every edit. It now runs
    on a worker thread, with a progress bar and a Cancel button, and its
    result is used only if the part has not changed meanwhile.
  - Large STEP imports and interference checks also run in the background.
  - Every ID counter is now atomic, so objects made on two threads never
    share an ID.

## Unreleased — Production readiness, Milestone 3 (Phases 107–110)

- **2D snapping and picking depended on zoom, and ignored hidden layers (110).**
  - Snapping reached a fixed 0.5 units: thousands of pixels when zoomed in,
    under one pixel when zoomed out. Picking had a 0.15-unit floor with the
    same effect. Both now reach 10 pixels on screen at any zoom.
  - Hidden and locked layers were snapped to, and Trim cut at them. Both
    now leave them alone.
  - Every snap was called an endpoint, and a line's midpoint was no snap at
    all. Snaps are now typed: endpoint, midpoint, centre, quadrant, and a
    new intersection snap.
  - An object snap in reach now beats the grid.
  - Trim's pieces lost their line type and group; Break, Extend, Chamfer
    and Fillet lost the group. All of these are kept now.
- **STEP files came in at the wrong size, all-or-nothing, with holes filled
  in (109).**
  - The length unit was ignored, so a part in inches came in 25.4 times
    too small and one in metres 1000 times too small.
  - One solid the reader could not rebuild refused the whole file.
  - A face with a hole was drawn over it, counted as solid in mass
    properties, and cut through by Booleans.

  Solids now come in in millimetres, and the conversion is noted. A bad
  solid is reported and the rest come in. Holes are holes.
- **DXF text, colour and units came in wrong (108b).**
  - MTEXT chunks were joined back to front. Its lines ran together into
    one. Formatting codes such as `\pxi-3;` were left in the text.
  - Centred and right-aligned TEXT came in at the wrong point.
  - `%%d`, `%%c` and `\U+00B0` came in as they are written, not as °, ⌀
    and the character.
  - Only ten colours were known, so index 12 came in white; true colour
    was ignored.
  - Pre-2007 files in Windows-1252 or 1251 came in as broken UTF-8.
  - A drawing in inches came in 25.4 times too small.

  All of these are read correctly now. Saving writes true colour, escapes
  text so it reads back, and declares millimetres. Formatting the drawing
  cannot show, such as underline or a height change inside a text, is
  reported.
- **DXF geometry came in wrong in several common cases (108a).**
  - A polyline's arc segments (bulges) came in as straight lines.
  - Mirrored entities (an extrusion of (0, 0, −1)) came in unmirrored.
  - The old POLYLINE / VERTEX form was not read at all.
  - A partial ellipse came in whole.
  - A block inserted with unequal scales was drawn at their average: (2, 1)
    became 1.5. A mirrored insert lost its mirror.
  - A block inside a block was dropped.

  All of these are now read as the file means them. Where the drawing model
  cannot hold something as it was, the import report says what was
  approximated:
  - a polyline with arcs comes in as grouped lines and arcs;
  - a stretched block comes in exploded;
  - a partial ellipse comes in as a polyline.

  Entities out of the drawing's plane, and 3D polylines, are reported rather
  than flattened.
- **Mirroring an arc across an axis that missed its centre put it in the
  wrong place (108a).** The 2D Mirror tool was affected. Fixed.

- **STEP, STL and glTF could not be reached from the window (107).** STEP
  import and export and glTF export existed in code, but no menu reached
  them, and there was no STL export at all. File ▸ Import and File ▸ Export
  now cover all four:
  - **STEP** imports as a new part. Each body is kept inside the part, so the
    part no longer depends on the STEP file.
  - **DXF** imports into the drawing you are working on, as one undoable
    step.
  - **Exports:** STEP, STL (new) and glTF for a part's body, and DXF for a
    drawing.
- **Opening a file silently lost what could not be read.** Since Phase 100 a
  malformed item is skipped instead of failing the whole file, but nothing
  said so, and the next save dropped it for good. A DXF's unread entity types
  vanished the same way. Opening or importing such a file now lists what was
  left out, and why ("entity 7 (spline): …", "3 3DFACE entities not read"),
  before you can save over it. A feature whose sketch is missing now says so.

## Unreleased — Production readiness, Milestone 2 (Phases 102–106b) — complete

- **Every Boolean result was a heap of triangles, which Fillet refused
  (106b).** The CSG triangulates both parts and splits the triangles along
  the other part's planes, then kept the pieces apart. A groove across a block
  came out as more than 20 triangles for 10 faces, and even faces the cut
  never reached came out in pieces. Every corner had extra edges meeting at
  it, so Fillet refused any edge of a part that had been through a Boolean
  ("non box-like corner"). A plate with a hole could not be filleted.

  Each face's pieces are now put back together after the Boolean. A face with
  a hole through it is cut through the hole into two, since a face here has a
  single loop; the cut meets the outer edge part-way along, so the corners are
  unaffected. The grooved block has its 10 faces, and names survive a
  Boolean that doesn't touch them. **A plate with a hole can now be filleted,
  undone, saved and reopened through the window**, which was the goal of
  Milestone 2. Files saved earlier rebuild exactly as before.

- **An edit to a sketch could move a fillet to a different edge (106).** An
  extrusion numbered its side faces in profile order and its edges in storage
  order, and a fillet stores the name of the edge it rounds. Adding a vertex
  anywhere in the sketch renumbered the edges, so the fillet silently rounded
  whichever edge now had its old number.

  New extrusions name their side faces after the sketch entities they come
  from (a line, a side of a rectangle, a facet of an arc) and their edges
  after the two faces they separate. An edit elsewhere in the sketch leaves
  those names alone.

  Boolean results follow the same rule:
  - A face a Boolean splits gets a name for each piece; pieces used to share
    one name.
  - Fillet, Chamfer, Shell and mates that refer to the whole face or edge find
    its pieces.

  Files saved before this keep the old names for every feature and every
  Boolean result, since their references were made against those names. The
  format moves to version 18, so older builds refuse the new files.

  (Boolean results were still left in fragments, which renamed edges beside
  an unrelated cut; 106b, above, puts them back together.)

- **Parts with a hole through them failed validation (105).** The
  Euler–Poincaré check read `V − E + F = 2(S − R)`: the face-hole count was on
  the wrong side, doubled, and the formula had no genus term. So every torus,
  and every part with a hole through it, was "invalid". That was not just a
  label:
  - STEP import refused such parts.
  - Fillet refused to round any edge on one.

  The check is now `V − E + F − R = 2(S − G)`, and `Solid::genus()` reports
  the number of through-holes.

  Every feature's result is now held to the full set of solid checks: every
  edge between two faces, counts Euler allows, flat faces flat, no boundary
  crossing itself, and a closed skin. The same goes for every Join, Cut and
  Intersect result. A malformed solid stops at the feature that made it, with
  a reason ("the result is not a valid solid: …"), instead of corrupting the
  features built on it.

  A profile that crosses or touches itself, such as a figure-eight, is refused
  with the point named. It bounds no single region.

- **Most 3D ribbon commands were demos (104b).** Box, Cylinder, Sphere, Cone,
  Torus, Union, Subtract, Intersect, Fillet and Chamfer each dropped a fixed
  mesh into the viewport, outside the document: the mesh was never saved or
  undone, and it ignored the part entirely. (A fillet was always "one edge of
  a 10 mm box".) Shell, Draft and Pattern had no command at all.

  Each command now asks for its inputs and adds a feature to the part, as one
  undoable step:
  - **Primitives** ask for their sizes and how the body combines with the
    part.
  - **Union / Subtract / Intersect** combine the part's separate bodies.
  - **Fillet and Chamfer** take the edges you choose. **Shell** takes the
    faces to open and a wall thickness. Until the viewport can pick edges and
    faces, they are chosen from lists: edges by their end points, faces by
    which way they face and where their middle is.
  - **Draft** takes a pull direction, a neutral plane and an angle.
  - **Linear and Circular Pattern** take a direction or axis, a spacing or
    angle, and a count.

  A feature that fails on the spot, such as a fillet too big for its edge,
  is refused with its reason rather than added.

  **Loft and Sweep have no command yet.** They need sketches on more than one
  plane, and the window can sketch only on XY; that is the next step
  (Phase 104c).

- **Changes to a part's history could not be undone, and one was not even
  saved (104a).** Undo covered 2D drawing only. Adding, reordering and editing
  features went around the undo stack, and **editing a feature's values
  (double-click) did not mark the document modified**, so closing the window
  discarded the edit without asking. Features could not be deleted or
  suppressed at all. Inserting a component or adding a mate could not be
  undone either, and a mate solve moved components with no way back.

  Every one of these is now a command on the undo stack:
  - adding a feature (with the profile sketch made for it);
  - editing a feature's values and how its body combines with the part, in
    one dialog instead of one prompt per value;
  - reordering, deleting and suppressing a feature, from the feature panel's
    menu and the Delete key;
  - inserting a component, and adding a mate together with the moves its solve
    made.

  Undo rebuilds the model or assembly it changed. The document's modified
  marker follows the undo stack, so undoing back to the saved state clears it.
  Script edits go through the same commands.

  A suppressed feature stays in the history but is left out of the build.
  It is saved as `"featureSuppressed"`, and the file format moves to version
  17: a build from before body operations (102) or suppression would ignore
  both and silently build a different part, so it now refuses the file
  instead.

- **A failed feature said "failed to execute" (103).** Extrude, Revolve,
  Loft, Sweep, Draft, the primitives, patterns and `BooleanOp` returned a bare
  null pointer; Shell, Fillet and Chamfer produced messages that their
  features then dropped; `ProfileValidator` explained a bad profile and
  Extrude threw the explanation away; and the Extrude command guessed "profile
  is not a closed loop" for every failure. `Feature::execute` now takes an
  optional reason, and every feature fills it, so the feature tree and status
  bar say, for example, "the profile is open: its ends at (0, 0) and (0, 10)
  do not meet", "Edge not found: box/no_such_edge", or "the cut removes the
  whole body". Boolean failures now distinguish an empty result (a cut
  through everything, an intersection of bodies that do not touch) from one
  that could not be sewn. Extrude also refuses two inputs it used to turn
  into zero-volume solids without complaint: a zero distance, and a direction
  lying in the sketch plane.

  Writing those messages turned up a larger gap: the profile reader
  understood lines, arcs and a lone circle only, so **a shape drawn with the
  Rectangle or Polyline tool — the usual way to draw one — could not be
  extruded, revolved, lofted or swept**, and was reported as a circle
  problem. Rectangles and polylines (and so imported LWPOLYLINEs) now count as
  the line segments they are drawn with; anything still unsupported is named
  ("an ellipse cannot be used in a profile yet").

- **A part could hold only one feature's geometry (102).** The rebuild
  threaded a single solid through the feature tree, and every feature that
  builds geometry — Extrude, Revolve, Loft, Sweep, the primitives — ignored the
  solid it was handed: a second extrude *replaced* the first, and a hole could
  not be cut. `buildBodies()` knew about separate bodies but only the tests
  called it, and the Boolean feature was a no-op on the product path.

  Each creating feature now has a body operation. *Join*, *Cut* and
  *Intersect* combine its body with the part through `BooleanOp`; *New body*
  keeps it as a separate shell of the part's solid, which rendering, the
  tessellation cache, mates and mass properties already handle. A cut or
  intersect with nothing to act on, a cut that would leave nothing, and a
  Boolean that fails are feature failures with a reason, never an empty part.
  All three build paths share one rule. The Extrude and Revolve commands ask
  for the operation (Join once the part has a body, New body for the first)
  and refuse — rather than add — a feature that fails on the spot. Stored as
  `"bodyOperation"`; files written before load every body as its own, so a
  part that showed only its last extrude now shows all of them.

  Two transforms assumed one body, which a spaced pattern could already
  break and New body now makes routine. Shell rebuilt its cup from one body's
  caps and dropped every other body without a word; it now refuses a part
  with several bodies. Draft oriented every face against the centroid of the
  whole solid, which for two bodies lies between them, outside both — so the
  sides facing each other were drafted the wrong way; each face is now
  oriented against its own body.

## Unreleased — Production readiness (Phases 97–101) — Milestone 1 complete

Work against the [production-readiness roadmap](docs/superpowers/specs/2026-09-23-production-readiness-roadmap.md).

- **The warning and sanitizer gates did not gate (97).** `cmake/CompilerWarnings.cmake`
  and `cmake/Sanitizers.cmake` defined `hz_set_warnings()` and
  `hz_enable_sanitizers()`, and nothing called either. The whole project built
  with no warning flags, and the CI job named "AddressSanitizer" configured
  `-DHZ_ENABLE_SANITIZERS=ON` into a plain Debug build — its log contains no
  `-fsanitize` at all. Both functions are now applied from the root
  `CMakeLists.txt` to every first-party target, so a new module is covered
  without opting in. GCC/Clang build with `-Wall -Wextra -Wpedantic
  -Wconversion` (MSVC `/W4 /permissive-`), and `HZ_WARNINGS_AS_ERRORS` fails
  the Linux CI jobs on any warning. UBSan is built with
  `-fno-sanitize-recover`, since by default it reports and carries on, which
  lets a test pass straight over undefined behaviour. The full suite is clean
  under ASan + UBSan + LeakSanitizer. `-Wsign-conversion` (about 560 size/index
  sites) is left off as a tracked burn-down rather than suppressed site by
  site, and clang-tidy is told the same, because Clang reads GCC's
  `-Wconversion` as including it.

  The warnings found about 25 sites, mostly dead code: unused variables, a
  lambda nothing called, an unused `flipped` flag in the angular dimension's
  arrowheads, `Eigen::Index` narrowed to `int`, a Qt signal deprecated in 6.9,
  and `Solid` forward-declared as a `struct` but defined as a `class` — legal,
  but the tag is part of the MSVC-mangled name.
- **Building on Linux without compiling Qt.** Qt is now a default-on `qt`
  feature of the vcpkg manifest; the new `linux-system-qt` preset turns it off,
  so vcpkg provides only the small libraries and the build uses an installed
  Qt 6. C++20 module scanning is off (no modules are used), which also stops
  CMake ≥ 3.28 with GCC ≥ 14 writing flags into `compile_commands.json` that
  clang-tidy cannot parse.
- **Line endings.** A checkout copied from Windows to Linux showed all 559
  files as modified — CRLF in the working tree, LF in the index.
  `.gitattributes` now pins LF everywhere.
- **Quitting lost unsaved work (98).** There was no `closeEvent`: File ▸ Exit
  and the window's close button discarded every open document without a
  word. And a drawing never became "modified" in the first place — the 2D
  commands edit `DraftDocument`, which had no dirty flag, and
  `Document::m_dirty` was only set by two methods the UI never calls, so the
  tab-close prompt that did exist could not fire for 2D work.

  The modified state now comes from the undo stack: it records which state was
  last saved, so every command marks the document modified, undoing back to
  the saved state clears the mark, and a push that discards the redo history
  holding the saved state leaves it unreachable. Changes made outside the undo
  stack (feature edits, until they become commands) still mark the document
  explicitly. Quitting walks every modified tab and asks Save / Discard /
  Cancel; closing a tab offers Save as well (it offered only Close and
  Cancel); modified tabs show `*` and the title bar Qt's modified marker. A
  new offscreen test binary drives the real `MainWindow` through each of these.
- **Saves could destroy the file they replaced (98).** Every writer opened the
  target with `std::ofstream`, truncating it, *then* serialized. `json::dump()`
  throws on invalid UTF-8 — which DXF-imported text carries whenever its source
  used a legacy code page — so saving such a drawing as `.hcad` left a 0-byte
  file and threw out of the Qt event loop. `file.good()` was also checked
  before `close()`, so a failed final flush (disk full) reported success. All
  writers now build their output in memory and replace the target through
  `io::writeFileAtomically`: temporary file in the same directory, flushed to
  disk, renamed over the target, permissions kept, symlinks followed, and on
  any failure the original untouched and the temporary removed. Invalid UTF-8
  is written as U+FFFD instead of throwing.
- **Non-ASCII paths on Windows (98).** Paths are UTF-8 strings throughout, but
  `std::ifstream(std::string)` reads a narrow path in the Windows code page, so
  a file under `C:\Users\José` could not be opened or saved. Readers and
  writers now open paths as UTF-8, and the executable declares UTF-8 as its
  active code page.
- `~ViewportWidget` dereferenced the current GL context unconditionally and
  crashed when the viewport had never been shown.
- **An error left no trace and no reason (99).** The whole code base made two
  spdlog calls, both to stdout — which a Windows GUI-subsystem executable
  discards — so a problem in the field left nothing behind. Logging now goes
  to a rotating file in the platform's app-data directory (`…/logs/horizon.log`,
  3 × 5 MB), Qt's own warnings are routed into it, warnings flush immediately,
  and start-up records the version, Qt version, OS and OpenGL driver.

  An exception escaping a slot or event handler terminated the application,
  and every open document with it; a kernel op that throws (the NURBS
  constructors do, on invalid input) during a rebuild would do exactly that.
  `hz::ui::Application::notify()` now contains it: the failing command is
  abandoned, the user is told once (not once per repaint), and the session
  survives to save. Feature execution catches too, so a throwing feature is a
  failed feature whose reason appears in the tree. `std::terminate` logs its
  cause before the process goes.

  Files that failed to open or save said "Failed to open file." and nothing
  more. The loaders and savers now return the reason — "the file does not
  exist", "it is a folder, not a file", "parse error at line 3, column 5: …",
  "this is not a Horizon document", the write error — and never throw; a
  wrong-typed field that used to throw `json::type_error` out of `load()` is
  now "the file is damaged: …". A viewport that cannot draw (no OpenGL 3.3,
  shaders that fail to compile, or no context at all) says so instead of
  staying blank.
- **Hostile input could crash, hang or quietly misread the app (100).**
  - *Native files.* The reader indexed `const json` objects with `[]`, which
    for a missing key or an empty array is an assertion in Debug and
    undefined behaviour in Release — no `try` can catch it. Every access in
    the readers is now checked. Integer and enum fields are range-checked (a
    float where an integer belongs went through a plain `static_cast`; a line
    type out of range would index past the dash-pattern table). A file from a
    newer format version is refused with both version numbers instead of
    loading with its unknown content dropped, ready to be destroyed by the
    next save. Counts that cost memory on every rebuild are clamped:
    `segments` at 4096 steps per turn (the cap the tolerance path already
    used), fillet chords at 1024, pattern instances at 10 000 — a pattern
    count of 2e9 in a file tried to allocate that many copies on open, and
    `static_cast<int>` of an infinite parameter was undefined behaviour.
  - *DXF.* A file cut short hung the application: the entity loop re-read its
    last pair forever at end of input. Every section now reads through one
    function that treats the end of input as "the file ends before the end of
    a section (it may be truncated)", a group code without a value and a
    non-numeric code line are errors naming the line, and numbers are parsed
    with `std::from_chars` — `std::stod` follows the C locale, which Qt sets
    from the environment on Unix, so under de_DE "1.5" read as 1.
  - *STEP.* Exceptions from the NURBS constructors (a degree-0 B-spline passes
    the reader's own checks) became the import error instead of escaping;
    `#` entity numbers are overflow-checked; reals are written and read with
    `to_chars`/`from_chars`, which ignore the locale and round-trip exactly
    (`%.15g` did neither), and a malformed number is an error rather than a
    silent prefix.
  - *Expressions* read from files recursed without limit: 200 000 minus signs
    overflowed the stack. Parsing is bounded to 64 levels of nesting and 1024
    nodes (a long flat sum parses in a loop but builds a tree evaluation must
    recurse through), and `Expression::fromJson` now honours its "nullptr on
    any error" contract instead of throwing on a wrong-typed field.
  - *Fuzzing.* libFuzzer targets for the native, DXF and STEP readers and for
    expressions (`HZ_BUILD_FUZZERS=ON`, Clang). The same targets always build
    as corpus replays registered with CTest, so the seed corpus — real
    documents from the real writers, plus every crashing input above — runs
    in every build, including the sanitizer job.
- **A crash lost everything since the last save (101).** Every two minutes
  (`autosave/intervalSeconds` in the settings; 0 turns it off) each modified
  document that changed since its last snapshot is written — atomically, in
  native format even for a DXF drawing — to a recovery directory owned by the
  running session and removed when it exits cleanly. After a crash, the next
  start finds the session whose lock's process is gone and offers its
  documents back: Recover reopens them modified, pointing at where they were
  saved and marked "(recovered)" until saved again; Discard deletes them;
  Later keeps them for the next start. A second instance running at the same
  time is never mistaken for a crashed one, and two instances starting
  together cannot both recover the same documents.

## Unreleased — Post-roadmap kernel work, continued (Phases 89–96)

Continues against the "Not yet addressed" list in the
[post-roadmap kernel findings note](docs/superpowers/notes/2026-09-01-post-1.0-kernel-findings.md).

- **Sweep collapsed on any turning path (89).** `Sweep` carried the profile
  along the path by translation only, which the header described as a
  fidelity limit — "the profile keeps its orientation". For a turning path it
  was a correctness defect: an XY-plane square swept up and then along +X was
  translated edge-on for the second leg, so that leg was a zero-thickness
  sheet. The L-shaped sweep integrated to 40 against 72, with two degenerate
  faces the geometric validator reports and the topology-only test never
  asked about. Separately, `SweepFeature` reduced every arc in a path sketch
  to its chord, so a bent path was swept as one straight segment.

  The profile is now carried by a rotation-minimizing frame: at every interior
  path point the section is the cut of the incoming prism by the miter plane
  bisecting the turn, which makes the cross-section perpendicular to each
  segment the profile turned by the smallest rotation between consecutive
  directions, and the far cap the profile's plane carried through the same
  turns. Every lateral face is exactly planar (its corners lie on two lines
  parallel to the segment), and with the profile's centroid on the path the
  volume is *exactly* normal-section area × path length — which the tests
  assert on a path turning in three planes. A path that doubles back, a
  profile whose plane contains the sweep direction, a turn tight enough that
  the inside of the profile would travel backwards, and any result the
  geometric validator rejects are refused instead of returned.

  Path arcs are sampled at `segments` steps per turn — a `SweepFeature`
  parameter, editable and persisted like the Phase 88 resolutions — so a bent
  sweep converges to area × arc length from below.
- **Accuracy is a distance, not a count (90).** Phase 88 made the facet count
  a feature property, but a count is the wrong unit: a fixed n sags
  r(1 − cos(π/n)), so the default 32 facets are within 0.024 on a radius-5
  cylinder and 0.48 on a radius-100 one. `segmentsForTolerance()` existed on
  `PrimitiveFactory` and `Revolve` and nothing outside the tests called it.
  Curved primitives, revolves, sweeps and fillets now take a `chordTolerance`
  parameter: when positive, the count is re-derived from it and the governing
  radius on every rebuild — the widest circle of a cone or torus, the profile
  vertex farthest from a revolve's axis, each path arc's own radius, and the
  fillet radius over the quarter arc its blend spans (new
  `FilletOp::arcSegmentsForTolerance()`). A radius edit therefore keeps the
  accuracy rather than the count. Setting the count explicitly returns to
  count mode; 0 turns the tolerance off; the tolerance is persisted and
  reloads as a tolerance, not as the count it produced. The feature parameter
  dialog's 0.001 floor would have silently switched a tolerance of 0 on for
  anyone clicking through it, so that field now accepts 0.
- **Rational NURBS surfaces were evaluated off their surface (91).**
  `NurbsSurface::evaluate` runs De Boor in two passes — each row in V, then
  across the rows in U — and gave the second pass unit weights. That discards
  the U-direction rationality, so every point of a cylinder, sphere, torus or
  cone *between* knots sat off the surface: up to 0.30 on a radius-5 cylinder,
  0.18 on a radius-3 sphere, 0.42 on a torus (about 6% of the radius), while
  every knot value was exact. The geometry tests had tolerated this in
  comments ("the two-pass evaluation loses some rational precision", "~5%")
  with tolerances of 0.2–0.5. The second pass now carries each row's weight
  sum Σⱼ Nⱼ(v)·wᵢⱼ, which makes it the exact tensor-product rational surface;
  those tests assert 1e-12, including off-knot samples on all four quadrics
  and the weighted-surface centre against the closed-form homogeneous value.
  Mate frames were unaffected only because they happen to sample at knots.
- **Extrude turned circles into squares (92).** Phase 84 found the curved
  primitives were box topology wearing a curved surface. Extrude, the most
  used feature in the product, had the same disease and was not in that
  sweep: a circle profile was extruded through box topology from four points
  on the circle, with a cylinder surface pasted onto the four flat sides, so a
  radius-5 disc extruded 10 integrated to **500 against 785**. Every arc in a
  line/arc profile was taken as its chord — a slot's round ends vanished (80
  against 111.4). Profile extraction was shared, so Revolve, Sweep and Loft
  chorded arcs the same way, and a circle section made Loft return nothing.

  Profiles are now faceted by one shared sampler: arcs are followed along
  their curve and a circle becomes an N-gon, at `segments` chords per turn or
  at the count each arc's own radius needs under `chordTolerance`, recording
  which chords came from which arc. The extrusion is an exact inscribed prism
  (the tests assert the N-gon volume to 1e-9) that converges to the exact
  solid from below. The lateral facets of an arc record their cylinder on
  `Face::analyticSurface` (for extrusions along the sketch normal) and every
  arc chord its circle on `Edge::analyticCurve`, so mates and radial
  dimensions still resolve from a single pick. A half disc now revolves to a
  sphere, two circles loft to the inscribed frustum, and a circle sweeps a
  pipe. `ExtrudeFeature` gains `segments` and `chordTolerance`, reported only
  when the profile has an arc or circle; Sweep's `segments` now also facets
  its profile. Documents that extrude a circle rebuild as the faceted cylinder
  on open, which changes their edge numbering: a fillet or chamfer that
  referenced one of the old square's edges by TopologyID no longer finds it.
- **Patterns stacked overlapping instances; Booleans and patterns dropped
  the ideals (93).** A pattern cloned every instance into its own shell
  whatever the spacing — the header called the merge "deferred" — so three
  10mm boxes 5 apart integrated to 3000 against the 2000 they occupy, and the
  six copies of a unit square patterned about its own corner were six
  interpenetrating shells that a document test asserted as correct. Every
  structural and geometric check passed. Instances whose bounds touch or
  overlap are now merged with `BooleanOp::Union` (a merge that fails refuses
  the pattern instead of returning interpenetrating shells); instances that
  stay apart remain separate bodies with no Boolean.

  Separately, the ideal geometry Phases 84–92 record was lost by the next
  operation. The pattern clone copied carriers but not `analyticSurface` or
  `analyticCurve`, and Boolean fragments were sewn without them, so a bored
  cylinder's bore — or any instance of a patterned boss — no longer resolved
  to a cylinder for a concentric mate or radial dimension. Pattern now moves
  the ideals with each instance; `BoundaryMesh` and `SolidSewer` carry
  `analyticSurface`, Boolean fragments recover it from their source face
  (looked up per operand, since two primitives of one kind share face IDs),
  and result edges lying along a source edge inherit its ideal curve.
- **Filleting a cylinder rim (94).** The open item Phases 85–86 left: a
  faceted cylinder's rim is a closed chain of chords, every vertex of which
  has two of them, and FilletOp refused any vertex with two selected edges
  ("exactly three are required for a corner blend"). So did two adjacent top
  edges of a box. Where two selected edges meet at a three-edge vertex, share
  one face, and the unselected third edge joins their other faces, the blends
  now meet on the plane bisecting the turn in the shared face: each blend's
  end section is carried along its edge onto that plane, the construction
  Phase 89 used for sweeps. The bands stay planar, the cap receives the inset
  polygon and the side edge is shortened by one radius, and no patch is
  needed. The removed material is exactly the blend cross-section times the
  length of its centroid path, which the tests assert to 1e-9 for a corner
  pair, an open chain, a square rim, a miter turned in a side face, and a
  32-chord cylinder rim. A turn too tight for the radius is refused.
- **Twisted lofts integrated the wrong solid (95).** A loft between a square
  and the same square turned 0.6 rad has lateral bands whose four corners are
  not coplanar. Such a loop encloses no well-defined volume, so each path in
  the kernel picked its own: mass properties and Booleans fanned it along one
  diagonal and got **180.8**, while the renderer drew the ruled patch, which
  encloses **150.7** — 20% apart. A level with any non-planar band is now cut
  along its rulings into strips, each non-planar strip into two triangles
  with the diagonal alternating from strip to strip. Every facet is flat, so
  the display and the computation see the same solid; and because the volume
  a bilinear patch bounds is exactly the mean of its two triangulations,
  alternating diagonals make the faceted volume equal the ruled loft's
  exactly — asserted to 1e-9 against Simpson's rule, which is exact for the
  quadratic section area. `twistSegments` (default 8, rounded up to even)
  only sets how closely the facets follow the curved patch, which each facet
  records on `analyticSurface`. Planar bands stay single quads, so aligned
  and similar sections build exactly as before.
- **Interference checking was unreachable (96).** `InterferenceChecker`
  (Phase 48) had no caller outside its tests: no assembly API, no command.
  It also reported only *whether* two solids clash — its header deferred the
  volume because the intersection Boolean was "not yet robust enough", which
  predates the kernel hardening. Each interfering pair now carries the
  volume of material the two share, from the Boolean intersection (flagged
  if that cannot be resolved); `AssemblyDocument::findInterference()` places
  every resolved, unsuppressed component by its transform and reports
  interfering pairs by component id, listing unresolved components as
  unchecked rather than passing them silently; and assemblies gain a
  **Check Interference** command. Mated faces and tangent cylinders touch
  without interfering. `Pattern::transformed()` exposes the placement copy,
  which moves carriers and ideals with the solid.

## Unreleased — Geometric validation, faceted geometry, working blends (Phases 81–88)

Post-roadmap kernel work, continuing from the review response below. Where kernel
hardening fixed what the Booleans *did*, this pass fixes what the kernel could
not *see*.

- **Geometric B-Rep validation (81).** `Solid::isValid()` walks twin/next/prev
  linkage and counts entities — it never reads a coordinate, so a solid can
  pass every structural check while its loops are self-intersecting,
  non-planar, or spanning positions its twin half-edges disagree about. The
  new `hz::topo::GeometryValidator` checks vertex-chain consistency
  (`he->next->origin == he->twin->origin`), twin positional coincidence,
  degenerate edges and faces, loop planarity against the face's own carrier
  (curved carriers are skipped rather than guessed at), loop
  self-intersection, and closed-shell area-vector balance — with advisory,
  non-failing reports for edge curves that miss their vertices and for
  distinct vertices at the same position.
- **Sharp cones were degenerate (81).** `makeCone` built box topology from two
  4-point rings; with `topRadius = 0` that ring collapses to a point, giving
  four zero-length edges around a zero-area cap. The Cone command asks for
  exactly that shape, so every cone inserted from the UI passed Euler and
  manifold while being geometric nonsense. Sharp cones now build apex topology
  (5V/8E/5F) with the analytic conical carrier; a cone with both radii or zero
  height is refused rather than returning a degenerate solid.
- **ChamferOp rebuilt on `SolidSewer` (82).** This closes the defect pinned in
  `test_AdversarialModels`: a 20mm box with two 2mm edge chamfers integrated
  to 3200 instead of 7920. Two things were wrong — loop construction pushed
  the original vertex back into the loop at corners the chamfer had already
  replaced, and reconstruction replayed the soup through Euler operators with
  a convergence loop that terminated on valid linkage that was not the linkage
  the polygons described. Loops are now vertex-driven and the soup is sewn by
  the same pipeline `BooleanOp` uses, so original faces keep their carriers
  and TopologyIDs. FilletOp needed no rebuild — its Phase-61 strict half-edge
  assembly already produced consistent loops — and both ops now refuse
  geometrically invalid output instead of returning it.
- **Chamfer vertex blends (83).** Two or three selected edges meeting at a
  corner used to be refused outright. The material a chamfer removes is the
  half-space beyond its chamfer plane, so a corner is just that corner clipped
  by the planes of every chamfer meeting there — blends fall out of the
  single-chamfer code path, with no invented corner patch, which is the
  standard planar-chamfer result. Validated against inclusion–exclusion
  volumes of the union of cutting prisms; chamfering all twelve edges of a
  cube yields the chamfered cube (18 faces, 32 vertices) at its exact volume.

Remaining known limits in this area are documented in the headers: chamfers
are for straight edges of planar-faced solids with orthogonal, convex corners
(oblique corners are refused by the geometric gate, not silently mis-built),
and inner face loops are not carried through the chamfer rewrite.

### Faceted curved primitives (84)

The kernel evaluates a solid from its face loops everywhere that matters —
Boolean classification, interference, mass properties, drawing projection,
export. The curved primitives leaned on that being invisible: `makeCylinder`
built box topology (8V/12E/6F) and bound a cylindrical NURBS patch to its four
lateral faces, so the solid *was* a square prism to every computation and only
the renderer disagreed. A cylinder's volume came out 500 against π·r²·h = 785,
a sphere's 192 against 524, and a torus — genus 0 with eight vertices —
enclosed no volume at all. Subtracting one cylinder from another gave 320
where the answer is 503.

- **Curved primitives are now faceted at construction**, built on `SolidSewer`
  from a polygon soup with a tunable `segments` count;
  `PrimitiveFactory::segmentsForTolerance()` inverts a chord-sag budget when
  you want to pick it from a tolerance instead. Facets carry planar patches
  that match their loops, so the B-Rep is exactly what the rest of the kernel
  treats it as. Volumes converge from below — at the default 32 segments a
  cylinder is within 0.65% and at 128 within 0.04% — and Booleans on curved
  solids work: boring a cylinder yields a real tube, correct to the facet
  error rather than 36% light. The torus is now a genuine genus-1 shell.
- **Facets remember what they approximate.** `topo::Face::analyticSurface` and
  `topo::Edge::analyticCurve` record the ideal geometry a facet stands in for,
  distinct from the carrier that actually bounds it. That is what keeps a
  cylindrical mate frame and a radial dimension resolvable from a single pick
  on one facet; binding the cylinder to a planar quad instead would put
  display and computation straight back out of step.
- **A rendering defect fell out of the same diagnosis.** `SolidTessellator`
  emits a face's whole *untrimmed* carrier whenever that carrier is curved, so
  faces sharing a surface each re-emitted all of it: a sphere drew six
  overlapping spheres at 480,000 triangles, and the resulting mesh enclosed
  three times the solid's volume. Faceted primitives take the loop path, so
  each face is emitted once — 960 triangles for that sphere, 124 for a
  cylinder — and the display mesh and the mass-properties integrator now
  agree. Both bounds are pinned by tests.

Two consequences are deliberate and pinned rather than papered over. A torus
is genus 1, so `V - E + F` is 0 and `Solid::checkEulerFormula()` — which has
no genus term — rejects it; `checkManifold()` and the geometric validator both
pass, and the tests assert exactly that. And a faceted cylinder exports to
STEP as planar B-spline faces rather than a rational cylindrical surface: the
written file is exactly the model in memory. Emitting analytic faces would
mean un-faceting on export, which is a separate feature, not a property of the
round trip.

### Blends that work on faceted geometry (85–86)

Faceting the primitives exposed that neither blend operation could act on
them, and checking why turned up defects rather than missing features.

- **Chamfer capacity measured the wrong thing.** It capped the distance at
  half the shortest edge of the adjacent face — a proxy that holds for a box
  and means nothing once faces are faceted. On a 32-sided cylinder the
  shortest edge is the facet chord, so every chamfer over 0.49 was refused
  where the geometry is exact past 4.9; it also rejected a 6mm chamfer on a
  10mm cube that builds correctly. Capacity is now how far the face reaches
  along the offset direction — a necessary condition that never rejects a
  distance that would have worked, with the geometric gate doing the real
  guaranteeing. A cylinder rim now chamfers to the exact truncated cone it
  removes, at any distance up to the cap's inradius: ten times the old range,
  bounded by geometry rather than a heuristic.
- **Vertex identity was inconsistent across the pipeline.** The clip
  deduplicated at 1e-9 × scale, the sewer welds at an absolute 1e-7, and the
  geometric validator judges a loop degenerate relative to its own extent. A
  1.075e-7 segment therefore sewed as legal and validated as degenerate, which
  is what the "self-intersecting loops" refusals actually were. Clipping now
  deduplicates against the loop's extent so all three agree; intersections are
  snapped to the endpoint they land on, since a cut through an existing vertex
  recomputes it with cancellation; and a segment nearly parallel to the clip
  plane no longer has an intersection computed at all, the denominator there
  being the difference of two nearly equal distances. `SolidSewer`'s weld
  tolerance is a named constant so the two cannot drift apart again.
- **A fillet's boundary was the chord, not the arc.** The blend was one flat
  quad joining the two tangent lines, carrying the correct rational-quadratic
  arc surface — the Phase 84 disease in a second place. Every loop-based path
  integrated a chamfer while the renderer drew a fillet: `(1/2)r²L` removed
  instead of `r²(1 − π/4)L`, 2.33 times too much, at every radius. Blends are
  faceted across the arc now, sampled by spherical interpolation between the
  two tangent radii so the points are exact on the arc, with the arc patch
  kept on `Face::analyticSurface`. Corner blends follow the same samples, so
  the spherical patch matches the arcs it joins instead of spanning them
  flat. Volume converges quadratically — inside 0.001% at 16 chords — and
  `analyticSurface` now survives a later operation, which it did not before.
- **Revolve enclosed zero volume (87).** The builder handled exactly one
  input: a four-vertex profile turned a full 360°. For it, it rotated the four
  profile corners to 0° and 180° and built an eight-vertex box from the two
  quads — which are mirror images of each other through the axis, so the box
  is inside-out against itself and encloses nothing. It passed every check the
  suite made (Euler, manifold, `isValid()`, a NURBS surface on all six faces),
  because no test asked for a volume. A torus surface was pasted on all six
  faces, including the two that were the profile itself. Every other input
  returned `nullptr`: any partial angle, any profile that was not a
  quadrilateral. The UI's Revolve command offers 1–360°, so 359 of its 360
  settings silently produced nothing.

  Revolve is now a swept ring stack sewn by `SolidSewer`, the same pipeline as
  the Phase 84 primitives: any closed profile, any angle in (0, 2π], angular
  resolution tunable per call and derivable from a chord-sag budget with
  `Revolve::segmentsForTolerance()`. Partial turns are capped at both ends
  (genus 0); a full turn clear of the axis closes on itself as a genus-1
  torus, which is manifold but which the genus-free Euler check rejects — the
  same documented caveat as `makeTorus`. Profile vertices sitting *on* the
  axis do not move, so their bands collapse to triangles and a profile
  touching the axis sweeps a proper cone. A profile crossing the axis, or one
  whose plane the axis does not lie in, is refused rather than swept through
  itself — one test on the radial vectors catches both.

  Volume converges quadratically to the Pappus value: −9.97% at 8 steps,
  −2.55% at 16, −0.64% at 32, −0.16% at 64, −0.04% at 128, and a partial turn
  carries the same relative error as a full one at equal angular resolution.
  Every band is *exactly* planar — rotating two points about a shared axis
  leaves all four corners on the plane whose normal combines the angular
  bisector with the axis — so unlike the fillet blends, no face here needs an
  approximate carrier. Only the ideals go on the side: the cylinder or cone
  each curved band approximates on `Face::analyticSurface`, the circle each
  profile vertex traces on `Edge::analyticCurve`. A band sweeping a flat
  annulus records neither, because its planar carrier is already exact.
- **Faceting resolution was unreachable from the document (88).** Phases 84–87
  made resolution the knob that decides how close a faceted solid's volume
  gets to the exact one — a cylinder at 32 segments is 0.64% under, at 128
  it is 0.04% — and then left it as a kernel call argument.
  `PrimitiveFeature`, `RevolveFeature` and `FilletFeature` each hard-coded the
  default, so no model could ask for a tighter one, no parameter edit could
  change it, and every reopened file replayed at whatever the build-time
  default happened to be. Accuracy was, in effect, a compile-time constant.

  The count is now a parameter of the feature that owns it: `segments` on
  curved primitives and on revolves, `arcSegments` on fillets, reported by
  `parameters()` and validated by `setParameter()` (three steps is the fewest
  that bounds a volume; one chord is the degenerate blend that removes a
  chamfer's worth of material). That is the path the UI's parameter editor and
  the save format already went through, so it became editable and persistent
  in one move rather than two. A box is exact, so it reports no resolution and
  refuses one.

  It is written into the JSON envelope and rides the FlatBuffers container
  with it. Files written before the field existed simply lack the key and load
  at the feature's default, which is what they were built with.

## Unreleased — Kernel hardening (post-roadmap review response)

Response to the external senior review: fix the Boolean/kernel reality gap
first, then the architecture boundary, interop pinning, and CI enforcement.

- **Boolean rework (the headline).** `BooleanOp` is now a BSP-tree CSG
  pipeline: the trimmed boundary is extracted from face loops
  (`BoundaryMesh`, with ear-clip triangulation of non-convex faces and
  global outward orientation), faces are split along the other solid's face
  planes with exact fragment classification and coplanar-face resolution
  (`MeshCsg`), and selected fragments are welded, T-junction-freed, and sewn
  into a manifold half-edge solid (`SolidSewer`).  Results carry TopologyID
  provenance, chain into further Booleans, and round-trip through STEP.
  Volumes are closed-form-exact for planar-faced solids (unions, subtracts,
  intersects, cavities, through-holes, multi-shell operands).  Curved faces
  participate as their loop polyhedra — analytic surface–surface
  intersection remains future work.
- **Faithful boundary evaluation everywhere.**
  `ExactPredicates::tessellateSolid` and `SolidTessellator` no longer
  tessellate the over-covering bounding-rectangle surface patches for planar
  faces (or re-emit shared curved surfaces once per face) — classification,
  display, interference, and export now agree with the actual solid.
- **CSG-exact interference.** `InterferenceChecker`'s narrow phase is now
  "does the Boolean intersection enclose volume", replacing edge-crossing
  heuristics that missed exactly-grazing symmetric configurations; surface
  touching no longer counts as interference, and cavities are respected.
- **Modeling kernel decoupled from render.** The mesh POD moved to
  `hz::geo::MeshData`; `render::MeshData` is an alias.  `hz_modeling` and
  `hz_document` no longer link `Horizon::Render`.
- **STEP interop pinned by fixtures.** Hand-authored third-party-style
  Part-21 fixtures (FreeCAD/OCC and SolidWorks formatting, analytic
  PLANE/LINE geometry, assembly product structure, `BREP_WITH_VOIDS`) with a
  drop-in directory contract for real vendor exports; documented
  limitations are enforced as tests, and restyled-reimport tests pin parser
  robustness (comments, reflow, entity reordering).
- **Adversarial model suite.** Hole patterns against multi-shell operands,
  nested pockets, non-convex bracket extrudes, shelled containers, Boolean
  volume-conservation fuzzing, and long feature chains — all with
  closed-form expected volumes.  The suite exposed and pins a real
  `ChamferOp` defect (combinatorially valid topology with geometrically
  inconsistent loops; volume integrator undercounts) for a future rebuild on
  `SolidSewer`.
- **clang-tidy is now a gate.** CI fails on any `bugprone-*`/`performance-*`
  finding outside a 10-check known-dirty backlog (which stays visible as
  advisory warnings).
- **Honest maturity documentation.** README gained a per-module Feature
  Maturity table (stable / experimental / prototype) and no longer implies
  1.0 production readiness.

Adversarial-review follow-ups (self-verified from code after the review's
automated verify pass was cut short):

- Fixed a self-inflicted regression: `ExactPredicates::tessellateSolid` (the
  `DrawingProjection` hidden-line occluder) had been switched to pure
  loop triangulation, which collapses curved solids — a torus's eight ring
  corners are coplanar — to a flat, zero-volume mesh.  It now delegates to
  `SolidTessellator`, keeping smooth surface tessellation for curved faces
  and restoring the `tessTol` control.
- The `checkManifold()` output contract now applies on **every** `BooleanOp`
  path, including the disjoint fast paths (previously only the CSG path was
  gated).
- Tolerance stack made consistent: the CSG on-plane epsilon is exported as
  `kCsgPlaneEps`, and `BooleanOp` welds CSG fragments at that tolerance so
  seams the splitter is allowed to open are always reconcilable during
  sewing.
- `SolidSewer`'s degenerate-face area test is now computed relative to the
  loop's first vertex, so a far-from-origin loop's true zero area no longer
  drowns in `R²` cancellation noise.
- Documented (in headers) the remaining known limitations the review
  surfaced: unbalanced BSP recursion depth, discarded face inner-loops on
  inputs, greedy twin pairing at non-manifold edges, and Booleans against
  coarse-box-topology curved primitives (torus/revolve).

## 0.1.0 — The 80-phase roadmap (not released)

All 80 roadmap phases (plus the 61b sheet-metal insert) delivered. ~900
automated regression tests pass across the Windows, Ubuntu, and
AddressSanitizer CI gates, with clang-format and clang-tidy checks. GPU paths
were verified on an RTX 5070 Ti via headless Vulkan.

### Era 0 — Foundation (Phases 1–30)

2D drafting application: math library, OpenGL 3.3 renderer, camera/grid/Qt
shell; document model with undo/redo, selection, snapping, native `.hcad` JSON
format and DXF I/O. Full drawing toolset (line, arc, circle, rectangle,
polyline, ellipse, spline, text, hatch), editing tools (offset, trim, fillet,
chamfer, break, extend, stretch, mirror, rotate, scale, arrays), dimensions and
annotations, a Newton–Raphson/LM constraint solver, blocks/components, layers
with ByLayer inheritance, line types via GPU shader, box selection, grouping,
UI modernization (dark theme, ribbon), an R\*-tree spatial index, parametric
sketch solving, an expression engine, and Linux CI.

### Era 1 — Geometry kernel (Phases 31–40)

NURBS curves and surfaces with adaptive tessellation; a half-edge B-Rep with
TopologyID genealogy and Euler operators; primitives; extrude and revolve;
Boolean operations; fillet/chamfer; feature-tree UI; viewport polish; kernel
hardening.

### Era 2 — Assemblies, interop & scripting (Phases 41–52)

Multi-document architecture with FlatBuffers `.hzpart`/`.hzasm` formats and
lightweight/resolved assembly loading; assembly mates (8 types, 6-DOF solver);
loft and sweep; shell and draft; linear/circular patterns; reference geometry;
Python scripting (embedded CPython via pybind11); collision detection;
measurement and mass properties (Eberly integrals); STEP AP242 import/export
(in-house ISO 10303-21); native binary format with zero-copy tessellation
cache; stabilization (sparse assembly solve, Boolean robustness, memory
guards).

### Era 3 — Professional workflow (Phases 53–64)

2D drawing generation (hidden-line projection, standard/section/detail views,
`.hzdwg`); GD&T feature control frames; BOM and balloons; sheets and title
blocks; in-house FEA (linear static and steady-state thermal); PDM local
version control and multi-user vault locking; advanced fillets (variable-radius
+ spherical corner blends) and drawing section views (61); sheet-metal core
(bend allowance/K-factor/flat pattern, 62) and 3D flange bodies (61b); Python
API phase 2; end-to-end stabilization.

### Era 4 — Cloud, rendering & market parity (Phases 65–80)

Rendering abstraction layer (`RenderBackend`) with OpenGL and staged Vulkan
backends; GPU compute NURBS tessellation (SPIR-V, verified GPU≡CPU); PBR
material library with IBL-lite ambient; an in-house CPU Monte Carlo path tracer;
local-first cloud sync of vault revisions; live-collaboration sessions with
feature-level token locking; CAM (2.5-axis toolpaths + G-code); kinematics
(forward + CCD inverse); advanced simulation (modal + stress-life fatigue);
configuration management (design tables); surfacing (Coons patches, 75); glTF
2.0 GLB export (76); localization infrastructure with starter catalogs (77);
large-assembly instancing + frustum culling (78); a zero-code-execution plugin
registry with a fail-closed permission model (79); and 1.0 release prep (80).

### Notable deviations from the roadmap (in-house instead of a dependency)

- **STEP** — in-house ISO 10303-21 Part-21 reader/writer rather than
  STEPcode/OCCT.
- **Ray tracing** — in-house Monte Carlo path tracer rather than Embree.
- **CAM** — closed-form 2.5-axis toolpaths rather than OpenCAMLib (general
  free-form pocketing/waterline staged behind that integration).

### Deferred beyond the 1.0 code kernel

Phase 73's CFD (deferred by the roadmap itself), and the productization tail of
Phase 80 — signed installers (MSI/AppImage/DMG), the hosted plugin marketplace,
and published SolidWorks/FreeCAD benchmarks — are future work, not code slices.
