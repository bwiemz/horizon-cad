# Horizon CAD — AI Assistant Design

**Date:** 2026-11-22
**Status:** Approved by the owner's standing goal (2026-11-22): "turn this into a
competitive open source CAD … with natively baked-in AI use so people can have AI
create parts or automate things … with major providers like Claude, GPT, Gemini or
whatever local model they're using." The owner asked that work proceed without
stopping for questions, so the decisions below marked *(call)* are the builder's,
recorded here so they can be reviewed.
**Baseline:** `master` @ `1d82fa75`.

---

## 1. Intent

A designer types what they want — "a 60 × 40 × 5 mm mounting plate with four
M5 clearance holes 6 mm in from each corner and 1 mm fillets on the top edges" —
and the part appears in the feature tree, built from ordinary, editable,
undoable features. The same assistant automates the tedious parts of a session:
"make every hole 0.2 mm bigger", "give the plate a `thickness` variable and drive
the extrusion from it", "what is the mass in aluminium?", "draw a 2D title block
outline on layer BORDER".

It must work with whichever model the user has: Anthropic Claude, OpenAI GPT,
Google Gemini, or a model running on their own machine (Ollama, LM Studio,
llama.cpp, vLLM, LocalAI, Jan — anything that speaks the OpenAI chat API).

No commercial CAD package ships this today with free choice of provider and a
local-model option. It is the one feature where an open-source tool can lead
rather than follow, so it is worth doing properly.

## 2. Goals, non-goals, anti-goals

**Goals**

1. One assistant, four provider families, chosen per user: Anthropic Messages
   API, OpenAI Chat Completions, Google Gemini `generateContent`, and any
   OpenAI-compatible endpoint (local or hosted: Ollama, LM Studio, OpenRouter,
   Groq, Mistral, DeepSeek, xAI…).
2. The AI acts through **structured tools** (function calling), never by
   writing raw file contents. Every change it makes is an ordinary command on
   the undo stack, so it is visible in the feature tree and undoable.
3. Part creation end to end: sketches on planes or faces, extrude, revolve,
   primitives, fillet, chamfer, shell, hole, patterns, mirror, variables and
   expressions, feature edits, suppression and deletion.
4. 2D drafting automation: lines, polylines, circles, arcs, rectangles, text,
   layers, delete and transform.
5. The AI can *see* the model: queries for the document, the feature tree, the
   solid's faces and edges (with geometry, so it can choose "the top edges"),
   mass properties and build errors. A feature that fails to build is withdrawn
   and the reason is returned to the model so it can correct itself.
6. A chat panel in the application with provider/model choice, a Stop button,
   and "Undo this response" that takes back every change the last request made
   in one action.
7. The same tools exposed over the **Model Context Protocol** by a headless
   `horizon-mcp` executable, so Claude Desktop, Claude Code, ChatGPT desktop,
   Cursor, and any MCP client can drive Horizon CAD for batch automation.
8. Everything outside the network transport is testable without a network:
   request building and response parsing are pure functions with unit tests per
   provider; the agent loop runs against a scripted fake transport.

**Non-goals (this design)**

- Training or hosting models. Horizon is a client.
- Text-to-mesh generation (Meshy-style). Parts are built from parametric
  features, or they are not built.
- Streaming tokens into the chat. Responses arrive whole; the panel shows that
  a request is in flight and can stop it. *(call: tool loops are dominated by
  short tool-call turns, and non-streaming keeps the three adapters simple.
  Streaming is a later phase.)*
- Images of the viewport sent to vision models (later phase; the tool API
  already reports geometry as numbers).
- Assemblies and drawing sheets through the AI (later phases; the tool registry
  is built to take them).

**Anti-goals — it fails even if it "works" when:**

- An AI change cannot be undone, or bypasses the undo stack.
- An API key is written into a document, a log, a crash report, or sent to any
  host but the one the user configured.
- A provider's request shape is guessed: every adapter follows the provider's
  published wire format, checked by tests that pin it.
- The application blocks (frozen UI) while waiting for a model.
- The AI can write or delete files without the user saying yes.
- Python code from a model is executed. *(call: the embedded interpreter is not
  sandboxed — `HZ_ENABLE_SCRIPTING` says so — so a model-authored script is
  arbitrary code execution. Structured tools cover the need.)*

## 3. Constraints found in the code

- `doc::UndoStack::push()` executes immediately; features are added with
  `AddFeatureCommand`, edited with `EditFeatureCommand`, removed with
  `RemoveFeatureCommand`, suppressed with `SetFeatureSuppressedCommand`;
  variables with `SetVariablesCommand`; sketches with `AddSketchCommand`;
  drafting entities with `AddEntityCommand` / `RemoveEntitiesCommand` /
  `MoveEntityCommand` etc. None of the model commands rebuild; the caller does.
- `UndoStack::withdraw(cmd)` exists for exactly the "a feature just added
  fails to build" case. The AI tools use it the same way the window does.
- `Feature` has a reflective surface — `parameters()`, `parameterKind()`,
  `parameterChoices()`, `vectors()`, `references()`, `setParameter()` … — so
  one generic `edit_feature` tool covers every feature type.
- Faces and edges have persistent names (`TopologyID` tags; `model::logicalEdge`
  / `logicalFace` collapse the facets of one curved face or edge). Fillet and
  chamfer take edge ids, shell takes face ids, hole and mirror take a face's
  whole name. The window lists them in `ui/ModelPicks.cpp`; the AI layer needs
  a Qt-free equivalent with geometry attached.
- The model is always millimetres and radians (Phase 154 design decision). The
  AI tools take millimetres and **degrees** (models handle degrees far more
  reliably) and convert.
- vcpkg builds Qt with an explicit feature list (`vcpkg.json`): `network` is not
  in it today. CI on Windows/Linux uses vcpkg's Qt; macOS uses Homebrew's Qt
  (which has Network). Local development uses the system Qt 6.11 (has Network
  and the OpenSSL TLS backend).
- nlohmann-json is already a dependency; no new third-party library is needed.

## 4. Approaches considered

| # | Approach | Trade-off | Verdict |
|---|----------|-----------|---------|
| A | **Native C++ provider adapters + structured CAD tools + Qt Network transport**, with an MCP server over the same tools | Most code to write; no new dependency; works offline with local models; every request shape under our tests | **Chosen** |
| B | Bundle a Python LLM SDK (anthropic/openai/google-genai) in the embedded interpreter | Three SDKs to ship and keep current; needs scripting on (off by default, unsandboxed); Python packaging on three OSes | Rejected: dependency weight and it ties AI to the unsandboxed interpreter |
| C | Only an MCP server; let external apps (Claude Desktop, etc.) be the chat | Least code; but no in-app experience, and users of GPT/Gemini/local models without an MCP-capable client get nothing | Kept as goal 7, not as the only path |
| D | libcurl transport instead of Qt Network | One more dependency and its TLS backend per platform; Qt Network is already part of qtbase | Rejected *(call)* |

## 5. Architecture

```
            ┌───────────────────────── hz_ui (Qt) ─────────────────────────┐
            │ AiAssistantPanel (dock)   AiSettingsDialog   QtHttpTransport   │
            └───────────────┬──────────────────────────────────┬────────────┘
                            │ events                           │ HttpTransport
┌───────────────────────────▼──────── hz_ai (no Qt) ──────────▼────────────┐
│ AgentSession — conversation, the loop, cancellation, step limit          │
│   ├─ ProviderAdapter: AnthropicAdapter | OpenAIAdapter | GeminiAdapter   │
│   │     buildRequest(ChatRequest, ProviderConfig) -> HttpRequest         │
│   │     parseResponse(HttpResponse) -> ChatResponse                      │
│   │     buildListModels / parseListModels                                │
│   └─ ToolRegistry — ToolSpec{name, description, JSON schema} + handler   │
│        CadTools: queries, sketches, features, variables, drafting        │
│              (all mutations through doc::UndoStack)                      │
└──────────────────────────────▲───────────────────────────────────────────┘
                               │ same ToolRegistry
                 horizon-mcp (stdio JSON-RPC, MCP 2025-06-18) — headless
```

### 5.1 Provider-neutral conversation (`hz/ai/Conversation.h`)

- `ContentBlock`: `Text{text}`, `ToolCall{id, name, input(json)}`,
  `ToolResult{callId, name, content(text), isError}`, and `Opaque{provider,
  json}` — a provider block this code does not interpret but must send back
  unchanged on later turns (Anthropic `thinking` blocks; Gemini parts carrying
  `thoughtSignature`). Opaque blocks are dropped when the conversation is sent
  to a different provider family.
- `Message{role: User|Assistant, blocks}`. Tool results travel in a User
  message, all results of one turn in one message (Anthropic requires that
  parallel results share one message; the other adapters split them).
- `ChatRequest{model, system, messages, tools, maxTokens}`;
  `ChatResponse{blocks, stop: EndTurn|ToolUse|MaxTokens|Refusal|Other,
  usage{input, output}, error}`.

### 5.2 Adapters (wire formats)

**Anthropic** — `POST {base}/v1/messages`; headers `x-api-key`,
`anthropic-version: 2026-10-21`, `content-type: application/json`. Body:
`model`, `max_tokens` (default 16000), `system`, `messages` with content arrays
(`text`, `tool_use{id,name,input}`, `tool_result{tool_use_id, content,
is_error}`), `tools[{name, description, input_schema}]`. **No `temperature`/
`top_p`/`top_k`** (400 on API version 2026-10-21). Optional
`output_config.effort` (low|medium|high) from settings. For models that support
it (`claude-fable-5-2`, `claude-fable-5-1`, `claude-opus-5-5`, `claude-opus-5`,
`claude-sonnet-5-5`), `fallbacks: "default"` with
`anthropic-beta: server-side-fallback-2026-07-01`, so a request a safety
classifier declines is re-run server-side on Anthropic's recommended fallback
model (setting, on by default). `stop_reason` handled: `end_turn`,
`tool_use`, `max_tokens`, `refusal` (checked before content is read),
`pause_turn`. `thinking` / `redacted_thinking` blocks are kept as Opaque and
replayed unchanged. Errors: `{"type":"error","error":{"type","message"}}`.
Models: `GET {base}/v1/models` → `data[].id`. Default model `claude-opus-5-5`.

**OpenAI / OpenAI-compatible** — `POST {base}/chat/completions` where `base`
includes `/v1` (`https://api.openai.com/v1`, `http://localhost:11434/v1` for
Ollama, `http://localhost:1234/v1` for LM Studio); header `Authorization:
Bearer` when a key is set. Messages: `system`, `user`, `assistant` with
`tool_calls[{id, type:"function", function{name, arguments(JSON string)}}]`,
and one `tool` message per result (`tool_call_id`, `content`). Tools:
`[{type:"function", function{name, description, parameters}}]`. Official
OpenAI uses `max_completion_tokens`; compatible servers get `max_tokens`
*(call: most local servers predate the newer name)*. No temperature by
default (current OpenAI reasoning models reject non-default values).
`finish_reason`: `stop`, `tool_calls`, `length`, `content_filter`. Arguments
that are not valid JSON become a tool error returned to the model rather than a
crash. Models: `GET {base}/models`. Default model for OpenAI `gpt-5.5`; for
local servers none — the user picks from the fetched list.

**Gemini** — `POST {base}/v1beta/models/{model}:generateContent`; header
`x-goog-api-key`. Body: `systemInstruction{parts[{text}]}`, `contents[{role:
user|model, parts[]}]` with `text`, `functionCall{id?, name, args}`,
`functionResponse{id?, name, response{result|error}}`;
`tools[{functionDeclarations[{name, description, parameters}]}]`;
`generationConfig{maxOutputTokens}`. A model part is replayed whole (so its
`thoughtSignature` goes back, which Gemini 3+ requires for function calling).
Calls without an `id` get one generated locally and mapped back by name.
`finishReason`: `STOP`, `MAX_TOKENS`, `SAFETY`/`RECITATION`/`PROHIBITED_CONTENT`
→ Refusal. Errors: `{"error":{"code","message","status"}}`. Models:
`GET {base}/v1beta/models` → `models[].name` minus `models/`, filtered to those
whose `supportedGenerationMethods` has `generateContent`. Default model
`gemini-4-argon`.

JSON schemas for tools use only the subset all three accept: `type`,
`properties`, `required`, `items`, `enum`, `description`, `minimum`,
`maximum`. No `additionalProperties`, `oneOf`, `$ref` or `default`.

### 5.3 The agent loop (`AgentSession`)

`send(text)` appends a User message and runs: build request → transport →
parse → append the assistant message → for each tool call, run it through the
registry and collect results → append one User message of results → repeat,
until the model ends its turn, `maxSteps` (default 24) round trips pass, an
error occurs, or `cancel()` is called. It is asynchronous: the transport calls
back, so the window never blocks. Events go to an observer: assistant text,
tool call started, tool result, error, finished (with token usage). A cancelled
request's late reply is ignored (a generation counter). While a request is in
flight a second `send` is refused.

The session records the undo stack's `undoCount()` and `revision()` when a
request starts and after its last change. **Undo this response** undoes back to
the start count, and only when the stack's revision is still the one after the
AI's last change (so a user edit in between is never undone by it).

### 5.4 CAD tools (`CadTools`)

Units: lengths in mm, angles in degrees. Every mutating tool pushes commands on
the document's undo stack, rebuilds the model, and returns a short JSON
summary. A model feature that fails to build is withdrawn
(`UndoStack::withdraw`) and the tool returns `isError` with the build message.

| Tool | Does |
|------|------|
| `get_document_info` | type, unit, counts, layers, variables, solid summary (bounding box, volume, faces, bodies), last build error |
| `list_features` | per feature: index, id, type, suppressed, operation, parameters (value, kind; angles in degrees), vectors, references, expressions, failure |
| `list_sketches` | id, name, plane (origin, normal, x axis), face, entity summaries |
| `list_edges` | the solid's logical edges: id, endpoints, length, straight or curved, the normals of the faces either side; optional filter (`direction`, `near_point`, `max`) |
| `list_faces` | logical faces: id, whole name, flat or curved, outward normal, centre, area |
| `get_mass_properties` | volume, area, centre of mass, mass at a density (or material: steel, aluminium, ABS, PLA, titanium, brass, wood) |
| `create_sketch` | a sketch on XY/XZ/YZ (with an offset), on a flat face by its name, or on a plane (origin, normal, x axis), with entities in one call: `line`, `polyline` (closed?), `rectangle` (corner or centre), `circle`, `arc` (centre, radius, start/end angle), `slot`, `polygon` (regular, n sides), each optionally construction |
| `extrude` | sketch id, distance, extent (blind, symmetric, through_all, through_all_both), direction (`normal`, `reverse` or a vector), operation (new, join, cut, intersect) |
| `revolve` | sketch id, axis (point + direction, in sketch-local `x`/`y` axis or world vector), angle, operation |
| `add_primitive` | box, cylinder, sphere, cone, torus, with position and axis, operation |
| `fillet` / `chamfer` | edge ids (from `list_edges`), radius / distance |
| `shell` | thickness, faces to remove (ids) |
| `add_hole` | face name, position, diameter, depth or through-all, simple/counterbore/countersink sizes |
| `linear_pattern` / `circular_pattern` | direction & spacing & count / axis & count & total angle |
| `mirror` | a plane (point + normal) or a flat face |
| `edit_feature` | feature index, parameters (by name, mm/deg), vectors, expressions, operation |
| `delete_feature`, `suppress_feature` | by index |
| `set_variables` | name → expression (`"wall": "2 mm"`, `"depth": "wall * 5"`), merged with the document's |
| `draw_2d` | entities as in `create_sketch`, into the top-level drawing on a layer |
| `create_layer` | name, colour, line weight |
| `delete_entities`, `transform_entities` | by id; move / rotate / scale / mirror / copy |
| `list_entities` | the drawing's entities with geometry (paged) |

Approval: tools are tagged `Read`, `Modify` or `External`. Read and Modify run
without asking (Modify is undoable). External (writing files) asks the user;
the first release has no External tools in the app *(call)*. `horizon-mcp`
adds `open_document`, `save_document` and `export` (STEP/STL/DXF/glTF), which
are the client's to gate.

### 5.5 System prompt

Fixed text (so providers can cache it): what Horizon CAD is; units; the
coordinate system (Z up, XY ground plane, front view looks along +Y); the
modelling workflow (sketch → extrude/revolve; list edges before filleting;
prefer variables for sizes the user names); that every change is undoable; be
brief, act rather than describe, say when something cannot be done with the
tools; never invent ids — query first.

### 5.6 Application integration

- `QtHttpTransport` (hz_ui): `QNetworkAccessManager`, JSON POST/GET with a
  timeout (300 s default, local models are slow) and `abort()` on cancel.
  When TLS is unavailable (`QSslSocket::supportsSsl()` false) an `https` URL
  fails with a clear message; `http://localhost` still works.
- `AiAssistantPanel` — a dock on the right, toggled from the View menu, the
  ribbon's Tools/AI button and the command palette ("AI Assistant",
  Ctrl+Shift+I). Transcript (assistant text as Markdown, tool calls as compact
  grey lines, errors in red), input box (Enter sends, Shift+Enter new line),
  Send/Stop, "Undo this response", "New conversation", provider/model shown
  with a settings button.
- After each tool call that changed the model the window refreshes its feature
  tree, sketch list and scene (`rebuildFeatureTree`-equivalent without a
  second build).
- `AiSettingsDialog` — provider (Anthropic, OpenAI, Google Gemini, Local /
  OpenAI-compatible), base URL, model (editable combo + "Fetch models"), API
  key (masked), effort (Anthropic), "server-side fallback on refusal"
  (Anthropic), max steps. Settings in `QSettings` under `ai/`.
- **API keys**: read from the environment first (`ANTHROPIC_API_KEY`,
  `OPENAI_API_KEY`, `GEMINI_API_KEY` / `GOOGLE_API_KEY`); otherwise from a
  per-user file `<AppConfigLocation>/ai-keys.json` written with owner-only
  permissions, never into `QSettings`, a document, a log or a crash report.
  *(call: no OS keychain dependency in this release; the dialog says where the
  key is stored.)*

### 5.7 `horizon-mcp`

Headless executable (no Qt GUI), stdio transport, JSON-RPC 2.0 framed one
message per line, protocol version `2025-06-18` (accepts the client's when it
offers an older one it knows). Implements `initialize`, `notifications/
initialized`, `ping`, `tools/list`, `tools/call`. Holds one document: starts
empty (Part) or with `--open <file>`; extra tools `new_document`,
`open_document`, `save_document(path)`, `export(path, format)`. Logs to stderr
only. Documented with a ready-to-paste Claude Desktop / Claude Code config.

## 6. Phases (one PR each)

1. **AI core** — `src/ai` (Conversation, adapters ×3, AgentSession,
   ToolRegistry, CadTools), tests, this document. No UI, no new dependency.
2. **In-app assistant** — Qt Network in the build (vcpkg `network` feature;
   TLS: OpenSSL on Linux, Schannel on Windows, Secure Transport on macOS; the
   TLS plugin deployed in packages), transport, panel, settings dialog, key
   storage, menu/ribbon/palette wiring, translations extracted.
3. **MCP server** — `horizon-mcp`, document I/O and export tools, docs.
4. Later: streaming, viewport images to vision models, assembly and drawing
   tools, a prompt library of part templates, conversation saved with the
   document.

## 7. Testing

- Per adapter: the request JSON for a conversation with text, a tool call, a
  tool result and an opaque block (pinned field by field); response parsing for
  text, tool calls (several in one turn), each stop reason, refusals, errors,
  malformed bodies (never throws); model-list parsing.
- Agent loop with a scripted fake transport: tool call → result → end; the step
  limit; cancel during a request (late reply ignored); a tool that throws
  becomes an error result; undo-this-response, and its refusal after a user
  edit.
- CAD tools against a real `doc::Document`: sketch + extrude makes a solid of
  the right volume; a cut; fillet on edges chosen from `list_edges`; a failing
  feature is withdrawn with its reason; variables drive a parameter; drafting
  entities land on their layer; every mutating tool is one undo step.
- MCP: `initialize` → `tools/list` → `tools/call` over an in-memory stream.
- UI: the panel against the fake transport (offscreen), as the other window
  tests do.

## 8. Left to the builder

Exact tool descriptions and schema wording; the transcript's visual style
(follow the existing panels); timeouts; how much of `list_edges` geometry to
report by default (keep responses small — tool results count against the
model's context).
