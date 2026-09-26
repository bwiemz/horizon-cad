# Files

## Kinds of document

| Document | File | Made with |
|---|---|---|
| Drawing (2D) | `.hcad` | File ▸ New Drawing (**Ctrl+N**) |
| Part | `.hzpart` | File ▸ New Part |
| Assembly | `.hzasm` | File ▸ New Assembly |
| Drawing sheet | `.hzdwg` | Drawing ▸ New Drawing from Part or Assembly... |

An assembly and a drawing sheet refer to their part files by a path relative
to themselves: keep them together when you move them. DXF files (`.dxf`)
open as drawings.

**File ▸ Open...** (**Ctrl+O**) opens any of them; **Open Recent** lists the
last ten. Files named on the command line (`horizon part.hzpart`) open in
tabs. A file that is already open is brought to the front.

When a file open in Horizon CAD is changed by another program, its tab is
read again. If you have changed it too, you are asked whether to read it
again or keep yours.

## Importing

| File ▸ Import | |
|---|---|
| STEP as a New Part... | Reads the solids of a STEP file (`.step`, `.stp`) into a new part, in the file's units. |
| STEP as an Assembly... | Writes each part of a STEP assembly to a part file, in a folder beside the new assembly, and opens the assembly. |
| DXF into This Drawing... | Adds a DXF file's layers, blocks and geometry to the drawing. |

What could not be read exactly is listed afterwards, in "Not Everything Was
Read".

## Exporting

| File ▸ Export | |
|---|---|
| STEP... | A part's solid, or an assembly. Curved faces are written as their surfaces where they can be. |
| STL... | The part as triangles, for 3D printing. |
| glTF... | The part as a `.glb` model, for viewers and the web. |
| DXF... | The drawing, in millimetres. |
| PDF..., SVG... | The drawing or sheet on paper: size, orientation, scale (or fit), and colours as drawn or in black. |

An item is available when the document has what it writes.

## Autosave and recovery

Documents with unsaved changes are saved every two minutes to a recovery
folder (change how often in [Settings](settings.md)). The status bar says so
if autosave is off or cannot write. When Horizon CAD closes normally, those
copies are deleted.

If Horizon CAD stops without closing, the next start offers **Recover
Documents**: **Recover** opens the saved copies, marked "(recovered)";
**Discard** deletes them; **Later** keeps them for next time. If a
recovered document seems to have stopped Horizon CAD again, Recover is no
longer the default.

## Crash reports

If Horizon CAD crashes, it writes a report of what happened: where it
stopped, the version, and the last lines of its log. Nothing is sent
anywhere. At the next start it offers the report: **Save Report...** to
attach it to an issue, **Delete Report**, or **Close** to keep it.

Reports are kept in the `crashes` folder, and logs in the `logs` folder, of
Horizon CAD's data folder:

- Linux: `~/.local/share/Horizon CAD Project/Horizon CAD/`
- Windows: `%LOCALAPPDATA%\Horizon CAD Project\Horizon CAD\`
