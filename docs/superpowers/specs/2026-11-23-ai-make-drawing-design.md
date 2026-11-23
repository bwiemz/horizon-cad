# AI assistant: a drawing of the part

## Why

People ask a CAD assistant for the drawing as often as for the part. The
assistant can model a part and an assembly but cannot make the sheet that
goes to a shop or a client: on a sheet tab it is switched off, and its tools
reach no sheet.

## Goal

A tool, `make_drawing`, in a part's and an assembly's tab: the drawing the
menu's **New Drawing from Part or Assembly** makes (front, top, right and
isometric views in third-angle projection, laid out over the title block),
with the paper, orientation, scale and title block the request asks for. It
opens in a tab of its own, beside the part, unsaved, for the user to look at
and save.

## Non-goals (for now)

- Editing a sheet that is open (moving views, sections, details, dimensions).
  The sheet's settings live in the drawing workbench and its edits are not
  undo steps; giving the assistant them means moving the sheet into the
  document first. That is the next step, not this one.
- `horizon-mcp`: it has no window to open a tab in; a later tool can write
  the `.hzdwg` (and SVG or DXF) into its root.

## Anti-goals

- A drawing of something other than what the user sees. A sheet is drawn from
  the part's open tab (unsaved changes and all) and follows it; it refers to
  the part by its file, so a part never saved is refused, and one with
  unsaved changes is drawn with a note that it needs saving too, or the saved
  sheet would show the file's version when opened again.
- The request disturbed: switching tabs stops a request and binds the tools
  anew, so the sheet's tab opens behind the part's, not in front of it.
- A file written without the user: the sheet is not saved; the user saves it.

## Approach

`hz_ai` stays free of Qt and of the drawing workbench. The tool reads and
checks its arguments and the document, then calls a host hook
(`CadToolHooks::makeDrawing`, and the same in `AssemblyToolHooks`) with a
plain `DrawingRequest`; the tool is registered only when the hook is set. The
window's hook asks `DrawingWorkbench` to make the sheet (the menu's code,
factored out: a spec in, a tab out) in a tab that is not made current, and
returns what it made: the tab's title, the paper, the scale and the views.

Arguments: `paper` (A0-A4, ANSI A-D, Letter, Tabloid), `orientation`
(landscape, portrait), `scale` ("1:2", "2:1" or a number; omitted: the
largest standard scale that fits), and the title block's `title`,
`part_number`, `revision`, `drawn_by`, `date`, `material`, `company`.

Rejected: making the sheet in `hz_ai` with `DrawingDocumentIO` (it links
`hz_fileio`, which brings Qt through the renderer, and the window still has
to open the tab).
