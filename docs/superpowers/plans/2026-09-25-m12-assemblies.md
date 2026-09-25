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

### Plan
1. **Refresh when a part changes.**
   - A part file saved, here or by another program, reaches every
     assembly that uses it: `DocumentManager` polls on a timer, and a save
     in a tab notifies it directly.
   - Its components' meshes and resolved documents are dropped and
     resolved again, the mates re-solved and the scene rebuilt.
   - A part edited in a tab and closed unsaved no longer leaves its edits
     in the components that shared its document.
2. **Open a part from the assembly:** from the tree or a clicked component.
3. **A bill of materials:**
   - a dialog listing item, part and quantity, and exporting CSV;
   - one line per file, however its path is spelled.

### Tests
- Saving a part changes an open assembly's view and mates.
- A file changed on disk is picked up.
- Opening a component's part opens its tab.
- The BOM counts instances per canonical file, and exports.

## Tracking

Each phase is its own PR, stacked as before, with README rows and CHANGELOG
entries. The phase's section here is replaced by "as built" when it lands.
