# The AI assistant

The assistant builds and changes parts and drawings from what you type. Ask for
"a 60 × 40 × 5 mm plate with a 5 mm hole 8 mm in from each corner", and it makes
the sketch, extrudes it and drills the holes. What it makes are ordinary
features and entities: you can edit them in the feature tree, and undo them.

It works with the model you choose:

- **Anthropic Claude**, **OpenAI GPT** or **Google Gemini**, with your own API
  key from that provider;
- or **a model on your own computer**, through any server that speaks OpenAI's
  chat API: Ollama, LM Studio, llama.cpp, vLLM, LocalAI, Jan. Nothing then
  leaves your computer.

## Opening it

Choose **Home ▸ AI Assistant** on the ribbon, or press **Ctrl+Shift+A**. The
assistant opens in a tab beside Properties and Layers (**View ▸ AI
Assistant** shows or hides it).

## Setting it up

Click **Settings…** in the assistant.

1. Choose the **Provider**.
2. For Claude, GPT or Gemini, paste your **API key**. It is saved only on
   this computer, in a file only you can read, never in a document. Or set
   it in the environment before starting Horizon CAD: `ANTHROPIC_API_KEY`,
   `OPENAI_API_KEY`, or `GEMINI_API_KEY`. A key from the environment is used
   in place of a saved one.
3. Choose the **Model**. **Fetch Models** lists those the server and your key
   can use.
4. For a local model, set the **Server** to your server's address:
   - Ollama: `http://localhost:11434/v1`
   - LM Studio: `http://localhost:1234/v1`
   - llama.cpp: `http://localhost:8080/v1`

   Choose a model that can call tools ("function calling"). Larger models
   follow longer requests better.

The first time you send a request to a provider on the internet, the
assistant asks before sending. It sends your requests and what it reads of
the open document: features, sketches, faces, edges and drawing entities.
The provider's own terms and privacy policy apply to what it receives.

## Asking

Type a request and press **Enter**. **Shift+Enter** starts a new line.

- Describe a part with its sizes: *"A flanged bushing: 20 mm bore, 30 mm
  outside, 25 mm long, with a 45 mm flange 5 mm thick"*.
- Change what is there: *"Fillet the top edges by 1 mm"*, *"Make the
  holes 6 mm"*, *"Mirror the boss to the other side"*.
- Make it parametric: *"Make the plate's thickness a variable called wall,
  and drive the extrusion from it"*.
- Ask about it: *"What does this part weigh in aluminium?"*.
- Point at things: select faces, edges or drawing entities in the view, then
  ask about *"this face"* or *"these edges"*. The assistant asks the window
  what is selected: *"Put a 5 mm hole in the middle of this face"*, *"Fillet
  these edges by 2 mm"*.
- Draw in 2D: *"Draw a 100 × 70 mm border on a layer called FRAME"*.

As it works, the assistant lists each step it takes: ▸ for the step, ✓ when
it worked, ✗ when it did not. A feature that would fail is not added. The
assistant is told why, and usually tries another way.

Lengths are in millimetres and angles in degrees, whatever unit the document
shows.

## Taking it back

Every change the assistant makes is one step of **Edit ▸ Undo**.
**Undo This Response** takes back every change from the last request at
once. Once you have changed the document yourself, use **Edit ▸ Undo**: the
button never undoes your own work.

**Stop** stops a request that is still working. Changes already made stay,
and Undo This Response takes them back. **New** starts a new conversation.

## What it can do

It works on the active part, drawing or assembly. In a part or a drawing:

- **Sketches** of lines, polylines, rectangles, circles, arcs, slots and
  polygons, on the XY, XZ or YZ plane, on a flat face of the part, or on any
  plane.
- **Features**: extrude, revolve, loft, sweep, the primitives, fillet,
  chamfer, shell, draft, holes, linear and circular patterns, mirror, and
  combining bodies, with datum planes, axes and points to build on. It can
  edit, suppress or delete them, and use variables and expressions for their
  sizes.
- **Looking at the part**: its features, faces and edges, size, volume and
  mass, and measurements between faces, edges and points.
- **2D drawing**: shapes and text on layers. It can move, copy, rotate,
  scale, mirror and delete them. It leaves hidden and locked layers alone.

In an assembly:

- **Components**: it places part and assembly files, moves and turns them,
  renames, suppresses and removes them, and repeats them in linear and
  circular patterns. It takes files only from the assembly's folder (or a
  folder in it), so save the assembly beside its parts first; it never reads
  a file elsewhere on your computer. Insert any other part yourself, with
  **Assembly ▸ Insert Component**.
- **Mates**: fixed, coincident, concentric, distance, angle, parallel,
  perpendicular and tangent, between faces, edges and datums it names, or
  the faces you select. Each is solved as it is added; one the others cannot
  hold is not added, and it says why.
- **Checking it**: which components interfere, and by how much, and the bill
  of materials.

Each of these is one undo step, as the Assembly menu's commands are.

**Seeing what it made.** The assistant can look at a picture of the part or
the assembly, from any side or in iso, to check its work against what you
asked. You see each picture it looks at in the panel. This needs a model that
reads images: the cloud providers' models do, and it is on for them. Many
local models read none, so for a server on your computer it starts off.
Turn it on or off with **Let the AI see pictures of the part** in its
settings. Each picture is sent to the provider like the rest of the
conversation, and only the last three go with each request.

**Replies as they are written.** The assistant's words appear as the model
writes them, and become formatted text when the reply is in. A model that
declines part-way takes back what it wrote. If a server on your computer
shows nothing until the end, or stops part-way, turn off **Show replies as
they are written** in the settings: replies then come whole.

**Drawings.** Ask it for a drawing of the part or the assembly, and it makes
one as **Drawing ▸ New Drawing from Part or Assembly** does: front, top,
right and isometric views over a title block, on the paper, at the scale and
with the title block it is told (or works out from what you asked). The
sheet opens in a tab of its own, beside the part's, and is not saved: switch
to it to look at it, and save it. The part or assembly must have been saved
once, since the sheet refers to its file.

**Changing a sheet.** On a drawing sheet's tab the assistant works on the
sheet: ask it to fill in the title block, change the paper or the scale,
move or remove a view, show or hide a view's hidden edges, tangent edges or
centre lines, or dimension edges ("dimension the width and the holes"). Each
change is one step for **Edit ▸ Undo**, as the Drawing menu's are, and
**Undo response** takes back everything one request changed. It does not add
section or detail views yet; the Drawing menu does.

It cannot open, save or export files.

## Automating from outside

Other AI applications can drive Horizon CAD too. `horizon-mcp`, installed
beside Horizon CAD, serves the same tools over the Model Context Protocol, so
Claude Desktop, Claude Code, Cursor and other MCP clients can build, save and
export parts without the window. Its setup is in `docs/ai/mcp.md` in the
source tree.
