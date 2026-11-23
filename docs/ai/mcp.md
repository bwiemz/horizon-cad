# Driving Horizon CAD from an AI client (MCP)

`horizon-mcp` serves Horizon CAD's CAD tools over the
[Model Context Protocol](https://modelcontextprotocol.io). Any MCP client can
then build and change parts, assemblies and drawings: Claude Desktop, Claude Code,
ChatGPT's desktop app, Cursor, VS Code, and agents you write yourself. These
are the same tools the AI assistant uses in the window: sketches, features,
variables, faces and edges, mass, and 2D drafting, and in an assembly,
components, mates, interference and the bill of materials. Five more tools
handle the document's files.

It runs headless (no window) and speaks MCP over standard input and output.

## Running it

`horizon-mcp` is installed next to `horizon` on Windows and Linux. On macOS
it is inside the application:
`/Applications/HorizonCAD.app/Contents/MacOS/horizon-mcp`.

```text
horizon-mcp [--root DIR] [--open FILE] [--drawing]

  --root DIR    the folder files may be opened, saved and exported in
                (default: Documents/Horizon CAD MCP in your home folder)
  --open FILE   start with FILE (.hzpart, .hcad, .hzasm or .dxf)
  --drawing     start with an empty 2D drawing (default: an empty part)
```

It works on one document at a time, and saves nothing unless asked.

**Files.** A model chooses the paths it opens, saves and exports, so they are
kept to one folder, `--root`. A relative path is taken in that folder. A path
that leads out of it is refused, whether by `..`, by an absolute path or
through a symbolic link. It is refused before anything is touched when it names
another drive or a network share, and refused when its last part is a
symbolic link. A file already there is replaced only when the call says
`overwrite: true`; the document's own file is the exception. A filesystem
root or your home folder itself is refused as `--root`: point it at a project
folder.

## Claude Desktop

Add this to `claude_desktop_config.json` (Settings ▸ Developer ▸ Edit Config):

```json
{
  "mcpServers": {
    "horizon-cad": {
      "command": "C:\\Program Files\\Horizon CAD\\bin\\horizon-mcp.exe",
      "args": ["--root", "C:\\Users\\you\\Documents\\CAD"]
    }
  }
}
```

On macOS, `command` is
`/Applications/HorizonCAD.app/Contents/MacOS/horizon-mcp`; on Linux, the path
of `horizon-mcp` in your installation. To start on a file, add
`"--open", "/path/to/part.hzpart"` to `args`.

## Claude Code

```sh
claude mcp add horizon-cad -- /path/to/horizon-mcp --root "$PWD"
```

Then ask, for example: *"Make a 60 × 40 × 5 mm plate with four 5 mm holes 8 mm
in from the corners, fillet its top edges by 1 mm, save it as plate.hzpart and
export a STEP."*

## Other clients

Any client that starts an MCP server as a subprocess over stdio works. Give
it the path to `horizon-mcp` as the command.

## The tools

The CAD tools are those of the in-app assistant. See
[the user guide's page](../guide/ai-assistant.md), and `tools/list` for their
schemas. The file tools are:

| Tool | Does |
|------|------|
| `get_session_info` | The document: part, drawing or assembly, its file, unsaved changes, any build error |
| `new_document` | Start an empty part, drawing or assembly in its place |
| `open_document` | Open a `.hzpart`, `.hcad`, `.hzasm` or `.dxf` file in its place |
| `save_document` | Save as `.hzpart` (a part), `.hcad` (a drawing) or `.hzasm` (an assembly) |
| `export_document` | Write the solid as STEP, STL or glTF (`.glb`), the drawing as DXF, or the assembly as STEP (its parts and their placements), STL or glTF |

Lengths are millimetres and angles degrees.

`get_view_image` returns a picture of the part or assembly as MCP image
content, from a side or in iso. Clients that show images, such as Claude
Desktop, show it, and a model that reads images can check its work with it.
It is drawn in software, so no GPU or window is needed.

**Assemblies.** An assembly places part files. To build one, make and save
each part in the folder, start a `new_document` of type `assembly`, insert
the parts with `insert_component`, and mate them with `add_mate`. Save it as
`.hzasm` and export it to STEP for other CAD systems. Parts are taken only
from the assembly's folder (or a folder in it), or from the server's folder
while the assembly is unsaved. An assembly that places a part from outside
the server's folder is not opened by `open_document`. That is checked from
its files before any part is read. `--open` opens it, since that is your own
choice.

`tools/list` marks each tool with hints, which a client can use to decide
what to confirm with you:

- `readOnlyHint`: the queries.
- `destructiveHint`: every tool that changes the document or a file.
- `openWorldHint`: the tools that read or write files.

## Protocol

- JSON-RPC 2.0 over stdio, one message per line.
- Protocol versions 2025-06-18, 2025-03-26 and 2024-11-05. It answers a
  newer request with 2025-06-18.
- Methods: `initialize`, `ping`, `tools/list`, `tools/call`.
- A tool's failure, such as a bad argument or a feature that does not build,
  comes back as a result with `isError: true` and a reason the model can act
  on. A feature that fails is not added.
- A document opened or started by a tool takes over once that call is done,
  before the next message or the next call in a batch.
- Diagnostics go to standard error. Standard output carries only the
  protocol.
