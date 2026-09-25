# Milestone 12 — Assemblies you can build (Phases 143–144)

The roadmap (`docs/superpowers/specs/2026-09-24-product-completeness-roadmap.md`,
audit item P7) says an assembly can be started but not built: a component
lands at the origin, and nothing moves it, lists it or takes it out. This
milestone makes assemblies something a user can put together, keep up to
date and account for.

## Baseline

A survey of the assembly code on 2026-09-25 found the following.

**What exists:**
- The data model: `AssemblyDocument`, components with a transform, eight
  mate types, `snapshot`/`restore`.
- The solver: Newton–LM over 6 DOF per component, with grounding and
  redundancy analysis.
- Interference, the `.hzasm` format, `BomGenerator` and `BomExport`, which
  writes CSV.
- `DocumentManager`, which de-duplicates parts and shares meshes between
  instances.
- Undo, through one generic `AssemblyEditCommand` (before/after
  snapshots).

**What the UI offers:** New Assembly, Insert Component, Add Mate (a dialog,
pre-filled from two clicked faces) and Check Interference. All are in the
File menu only.

**Gaps and defects:**
- Nothing moves or rotates a component, and every insert lands at the
  origin, overlapping the last.
- No tree lists the components or mates. Nothing removes, suppresses or
  renames a component, or removes or edits a mate.
- `AssemblyDocument::removeComponent` leaves the component's mates behind.
  Any later solve then fails with `InvalidReference`.
- A freshly inserted component cannot be clicked. The `.hzpart` tessellation
  cache has no face tags, so `pickModel` finds no face until something
  resolves the part in full.
- Add Mate lists raw facet tags (`<face>/facet:k`), while a click returns
  the logical face. For a curved face the combo silently stays on its first
  entry, and a cylinder floods it with one entry per facet.
- Add Mate maps its type combo with `static_cast`, which is right only while
  the combo's order matches the enum's.
- A solve on open that moves components does not mark the assembly
  modified.
- Changes to a part never reach an assembly. There is no watcher: the
  poller in `DocumentManager` is never called. `resolveComponent` never
  refreshes a mesh it already has.
- Nothing opens a part from an assembly.
- The BOM cannot be reached, and it counts one file twice when it is
  spelled two ways.

## Phase 143: Placing components

### As built
- **Remove takes the component's mates with it** (`AssemblyDocument`). Every
  assembly edit, including a remove, is one undo step.
- **An Assembly menu.** The three commands that were in the File menu, plus
  Move, Rotate, Rename, Suppress or Unsuppress and Remove Component, and
  Edit and Remove Mate. The command palette offers them too.
- **An assembly tree** (`AssemblyTreePanel`), tabbed with the feature tree
  and in front for an assembly.
  - It lists the components (suppressed ones greyed) and the mates (their
    type, the two components, and a distance or angle).
  - Its context menu removes, suppresses or unsuppresses and renames a
    component, and edits a mate's value or removes it. The Delete key
    removes too, through a key event: the feature tree's is the one action
    bound to the key.
  - Its current row and the view's selection follow each other. A
    component chosen in the tree is highlighted whole: a pick with no face.
  - The current row is kept only within one assembly. Ids start at 1 in
    each, so switching tabs made another assembly's #1 current, for Delete
    to remove.
- **Commands act on the component clicked or current in the tree.**
  - Move: by a vector. Rotate: about X, Y or Z, through the component's
    middle, by an angle.
  - Each is one undo step and solves the mates again. A change the mates
    cannot hold is undone, with the reason. A component held by a Fixed mate
    is not moved, and the user is told so.
- **Inserted beside the others**, along +X, a tenth of the larger width
  apart.
- **A component can be clicked at once.** The `.hzpart` tessellation cache
  carries face tags, faces per triangle and edges, in optional keys that
  older builds ignore. A cache that does not hold together is read without
  them.
- **Add Mate lists faces as a click picks them.** Each logical face appears
  once, said by what it is and where, for example "cylinder of radius 3
  along (0, 0, 1)". The row's data is its name, so a clicked curved face is
  selected. The type is mapped by value, and the widgets are named.
- A solve on open that moves components marks the assembly modified.

### Tests
- `AssemblyDocument`: removing a component removes its mates, and a snapshot
  restores both.
- The cache carries the solid's own faces and edges.
- Window tests:
  - two inserts side by side;
  - Move and Rotate exact and undoable;
  - a move keeps a coincident mate, and a Fixed component is not moved;
  - the tree renames, edits a mate's value, suppresses, and removes a
    component with its mate, undoably;
  - two cylinders clicked on their sides become concentric. This fails
    without the cache's faces.

## Phase 144: Living assemblies

### As built
- **A part's change reaches the assemblies placing it**
  (`MainWindow::refreshComponentsOf`).
  - Its components' meshes and resolved documents are dropped, and
    `DocumentManager::releasePart` forgets a copy read for components alone,
    so the file is read again. A part open in a tab is kept: it is the part.
  - The mates are solved again, which may move components; a move marks the
    assembly modified. The scene is rebuilt if the active tab is one of them.
  - Three triggers:
    - A save in the part's tab.
    - A change on disk. A timer (`partWatchTimer`, 2 s) calls
      `pollExternalChanges`, but not while a modal dialog is open: Add Mate
      may be listing the components' faces.
    - The part's tab closed with its edits discarded. The components that
      shared its document, or its model's mesh, had them.
- **A tab showing a file changed on disk reads it again first**
  (`MainWindow::reloadTabsOf`). A component of a part open in a tab takes
  the tab's document, which was as old as its read.
  - A tab without unsaved changes is read again into a new document, in
    place of its own.
  - One with unsaved changes asks: read again, losing them, or keep yours
    (the default). Kept, the assemblies show your version.
  - An assembly's own tab is not read again; the status bar says to reopen
    it.
  - A large file is read on a worker, as it is opened, and the tab swapped
    when it is done. Files changed meanwhile wait their turn, by path. Edits
    made while it was read are kept, and the status bar says so.
  - The refresh that follows says it only when the tab was read again, so
    it does not hide why not; the mate solve reports only failures in it.
- **A part read again is tessellated again.** Each `Document` has a serial
  unique in the process, and a model mesh is shared under the path and that
  serial. Under the path alone, a new read built as many times as the old
  one was shown with the old one's mesh, while an undo step still held it.
- **Files placed stay watched.** A lightweight component's file is now
  watched as a resolved one's was. A file any component was resolved from is
  never unwatched, so closing a tab of the part does not stop the watch.
- **An undo keeps the geometry loaded now.** `AssemblyDocument::restore` puts
  back placements and mates. A component still present keeps its current
  mesh and part: an undo after a refresh brought back the old mesh.
- **Open Part** (Assembly menu, and the tree's context menu) opens the
  clicked or current component's part in its tab. A second time, that tab is
  shown again.
- **Bill of Materials** (Assembly menu): a table of item, part and quantity,
  with the path as tooltip, and Export CSV. One line per file:
  `BomGenerator` reads a relative path from the assembly's folder and makes
  it canonical.

### Tests
- Document:
  - a released part is read again, while components keep what they hold;
  - a part open in a tab is not released;
  - a lightweight component's file is watched;
  - a placed part stays watched when its tab closes;
  - `samePath` sees through spelling;
  - a restore keeps the geometry loaded now;
  - a part read again is not shown with its old mesh;
  - the BOM puts one file spelled three ways on one line.
- Window:
  - saving a part in its tab makes both blocks taller and moves the one on
    top;
  - a file rewritten on disk is picked up by the timer;
  - a part closed unsaved leaves the mates solving on the file's geometry;
  - a part open in a tab and changed on disk is read again there, and the
    mates follow;
  - one with unsaved changes asks, keeping them by default, and reads the
    file again when told to;
  - read on a worker, the tab is swapped when the reading is done, and edits
    made meanwhile are kept;
  - Open Part from the menu and from the tree;
  - the BOM lists and exports.
- The window tests that need a hook each fail with it switched off.

## Tracking

Each phase is its own PR, stacked as before, with README rows and CHANGELOG
entries. The phase's section here is replaced by "as built" when it lands.
