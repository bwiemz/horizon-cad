# AI Assistant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An AI assistant in Horizon CAD that builds and edits parts and drawings through undoable tools, with Anthropic, OpenAI, Gemini and OpenAI-compatible (local) providers, plus a headless MCP server over the same tools.

**Architecture:** A Qt-free `hz_ai` library holds the provider-neutral conversation, one adapter per provider wire format (pure request builders / response parsers), an asynchronous agent loop over an abstract `HttpTransport`, and a registry of CAD tools that act through `doc::UndoStack`. `hz_ui` adds a Qt Network transport, a dock panel and a settings dialog. `horizon-mcp` serves the registry over stdio JSON-RPC.

**Tech Stack:** C++20, nlohmann-json, Qt 6 Widgets + Network (UI only), GoogleTest.

**Spec:** `docs/superpowers/specs/2026-11-22-ai-assistant-design.md`

## Global Constraints

- Namespace `hz::ai`; headers under `src/ai/include/horizon/ai/`; CMake target `hz_ai`, alias `Horizon::AI`.
- `hz_ai` links no Qt. Only `hz_ui` and `src/app` link `Qt6::Network`.
- Anthropic: `anthropic-version: 2026-10-21`; never send `temperature`, `top_p`, `top_k`; default model `claude-opus-5-5`; default `max_tokens` 16000.
- `fallbacks: "default"` + `anthropic-beta: server-side-fallback-2026-07-01` only for `claude-fable-5-2`, `claude-fable-5-1`, `claude-opus-5-5`, `claude-opus-5`, `claude-sonnet-5-5`, and only when the setting is on (default on).
- OpenAI default model `gpt-5.5`, base `https://api.openai.com/v1`, `max_completion_tokens`; compatible servers: `max_tokens`, no default model.
- Gemini default model `gemini-4-argon`, base `https://generativelanguage.googleapis.com`, header `x-goog-api-key`.
- Tool schemas: only `type`, `properties`, `required`, `items`, `enum`, `description`, `minimum`, `maximum`.
- Tool units: millimetres and degrees. Model units: millimetres and radians.
- Every mutation goes through `doc::UndoStack::push`; a feature that fails to build is withdrawn with `UndoStack::withdraw`.
- No exception escapes a parser, a tool handler or a transport callback.
- API keys never in `QSettings`, documents, logs or crash reports.
- Formatting: `.clang-format` (clang-format-15); never run clang-format on CMakeLists.txt.
- JSON integers read only behind `is_number_integer()`/`is_number_unsigned()` checks (bug log, fuzz finding).

## Review Focus

1. A model sends a tool call with missing, mistyped or out-of-range arguments (string where a number goes, negative radius, unknown sketch id) → the tool returns `isError` with a sentence saying which argument, nothing crashes, nothing is half-applied. Test: `CadTools.BadArgumentsAreErrorsNotCrashes` (Task 5).
2. A provider returns HTML, an empty body, a 401/429/500, or JSON with an unexpected shape → a readable error in the chat, the session idle and usable again. Tests per adapter: `ParsesErrorBodies`, `MalformedBodyIsAnError` (Tasks 2–3); loop: `TransportErrorEndsTheRequest` (Task 4).
3. The user edits the model by hand between two AI turns, then presses "Undo this response" → nothing of the user's is undone; the action is refused. Test: `AgentSession.UndoResponseRefusedAfterUserEdit` (Task 4).
4. The user presses Stop while a request is in flight, then sends again → the late reply of the first is ignored, the second runs. Test: `AgentSession.CancelIgnoresLateReply` (Task 4).
5. A huge solid (hundreds of faces) makes `list_edges` / `list_faces` blow the model's context → results are capped (`max`, default 60) and say how many were left out. Test: `CadTools.ListEdgesIsCapped` (Task 5).

---

## Phase 1 — AI core (PR 1)

### Task 1: Conversation model, tool registry, module skeleton

**Files:**
- Create: `src/ai/CMakeLists.txt`, `src/ai/include/horizon/ai/Conversation.h`, `src/ai/src/Conversation.cpp`, `src/ai/include/horizon/ai/ToolRegistry.h`, `src/ai/src/ToolRegistry.cpp`, `src/ai/include/horizon/ai/Http.h`
- Modify: `src/CMakeLists.txt` (add_subdirectory(ai)), `tests/CMakeLists.txt`
- Test: `tests/ai/test_ToolRegistry.cpp`, `tests/ai/CMakeLists.txt`

**Interfaces (Produces):**
- `enum class Role { User, Assistant };`
- `struct ContentBlock { enum class Kind { Text, ToolCall, ToolResult, Opaque }; Kind kind; std::string text; std::string id; std::string name; nlohmann::json input; bool isError=false; std::string provider; nlohmann::json raw; static ContentBlock makeText(std::string); static ContentBlock makeToolCall(std::string id, std::string name, nlohmann::json input); static ContentBlock makeToolResult(std::string callId, std::string name, std::string content, bool isError); static ContentBlock makeOpaque(std::string provider, nlohmann::json raw); };` (ToolResult content in `text`.)
- `struct Message { Role role; std::vector<ContentBlock> blocks; std::string text() const; std::vector<const ContentBlock*> toolCalls() const; };`
- `struct ToolSpec { std::string name, description; nlohmann::json inputSchema; };`
- `struct ChatRequest { std::string model, system; std::vector<Message> messages; std::vector<ToolSpec> tools; int maxTokens = 16000; };`
- `enum class StopReason { EndTurn, ToolUse, MaxTokens, Refusal, Error, Other };`
- `struct Usage { long long inputTokens = 0, outputTokens = 0; };`
- `struct ChatResponse { Message message; StopReason stop = StopReason::Other; Usage usage; std::string error; std::string refusalDetail; };`
- `Http.h`: `struct HttpRequest { std::string method = "POST", url; std::vector<std::pair<std::string,std::string>> headers; std::string body; };` `struct HttpResponse { int status = 0; std::string body; std::string networkError; };` `class HttpTransport { public: virtual ~HttpTransport(); virtual void send(const HttpRequest&, std::function<void(HttpResponse)> done) = 0; virtual void cancelAll() = 0; };`
- `enum class ToolAccess { Read, Modify, External };`
- `struct ToolResult { std::string content; bool isError = false; bool changedModel = false; static ToolResult ok(const nlohmann::json&); static ToolResult message(std::string); static ToolResult error(std::string); };`
- `class ToolRegistry { using Handler = std::function<ToolResult(const nlohmann::json&)>; void add(ToolSpec, ToolAccess, Handler); std::vector<ToolSpec> specs() const; std::optional<ToolAccess> access(const std::string&) const; ToolResult run(const std::string& name, const nlohmann::json& input) const; };` `run` returns an error for an unknown name, a non-object input, a missing required property, and catches every exception from the handler (`"<name> failed: <what>"`).

- [ ] Step 1: tests `ToolRegistry.RunsAHandler`, `UnknownToolIsAnError`, `MissingRequiredIsAnError` (message names the property), `HandlerExceptionBecomesAnError`, `NonObjectInputIsAnError`.
- [ ] Step 2: build, see them fail; implement; see them pass.
- [ ] Step 3: commit.

### Task 2: Anthropic adapter

**Files:** Create `src/ai/include/horizon/ai/Provider.h`, `src/ai/src/Provider.cpp`, `src/ai/src/AnthropicAdapter.cpp`; Test `tests/ai/test_AnthropicAdapter.cpp`.

**Interfaces (Produces):**
- `enum class ProviderKind { Anthropic, OpenAI, Gemini, OpenAICompatible };`
- `struct ProviderConfig { ProviderKind kind; std::string baseUrl, apiKey, model; int maxTokens = 16000; std::string effort; bool serverFallback = true; };`
- `ProviderConfig defaultConfig(ProviderKind);` `const char* providerName(ProviderKind);` `std::optional<ProviderKind> providerFromName(std::string_view);` (names: `anthropic`, `openai`, `gemini`, `openai-compatible`)
- `class ProviderAdapter { public: virtual ~ProviderAdapter(); virtual HttpRequest buildRequest(const ChatRequest&, const ProviderConfig&) const = 0; virtual ChatResponse parseResponse(const HttpResponse&) const = 0; virtual HttpRequest buildListModels(const ProviderConfig&) const = 0; virtual std::vector<std::string> parseListModels(const HttpResponse&, std::string* error) const = 0; };`
- `std::unique_ptr<ProviderAdapter> makeAdapter(ProviderKind);`
- Shared helper `std::string httpErrorMessage(const HttpResponse&, const nlohmann::json* body)` — "HTTP 401: <provider message>", or the network error.

- [ ] Step 1: tests — `BuildsHeadersAndUrl` (url `https://api.anthropic.com/v1/messages`, `x-api-key`, `anthropic-version: 2026-10-21`), `NoSamplingParameters` (body has no `temperature`/`top_p`/`top_k`), `SerializesToolCallAndResult` (assistant `tool_use{id,name,input}`; user `tool_result{tool_use_id, content, is_error}`; two results in one user message), `ReplaysOpaqueBlocks` (a `thinking` block from the response is sent back byte-identical; an opaque block of provider `gemini` is dropped), `FallbackOnlyForSupportedModels` (`claude-opus-5-5` → `fallbacks:"default"` + beta header; `claude-haiku-5-5` → neither; setting off → neither), `EffortGoesInOutputConfig`, `ParsesTextAndToolUse`, `ParsesRefusal` (stop Refusal, `stop_details.explanation` in `refusalDetail`, empty content OK), `ParsesErrorBodies` (401 body → error contains "authentication" message), `MalformedBodyIsAnError` (`"<html>"`, `""`), `ParsesModelList`.
- [ ] Step 2: fail → implement → pass → commit.

### Task 3: OpenAI(-compatible) and Gemini adapters

**Files:** Create `src/ai/src/OpenAIAdapter.cpp`, `src/ai/src/GeminiAdapter.cpp`; Tests `tests/ai/test_OpenAIAdapter.cpp`, `tests/ai/test_GeminiAdapter.cpp`.

- [ ] Step 1: OpenAI tests — `BuildsUrlFromBase` (base with or without trailing `/`), `BearerOnlyWithKey`, `OfficialUsesMaxCompletionTokens` / `CompatibleUsesMaxTokens`, `SystemFirst`, `ToolCallsSerializeArgumentsAsString`, `ToolResultsAreSeparateToolMessages`, `ParsesToolCalls` (two calls; `arguments` string parsed; invalid JSON arguments → the call kept with `input` = `{"_invalid_arguments": "<raw>"}` so the registry returns an error to the model), `NullContentWithToolCalls`, `FinishReasons` (`length`→MaxTokens, `content_filter`→Refusal), `ParsesErrorBodies`, `MalformedBodyIsAnError`, `ParsesModelList`.
- [ ] Step 2: Gemini tests — `BuildsUrlWithModelAndKeyHeader` (`…/v1beta/models/gemini-4-argon:generateContent`, `x-goog-api-key`), `SystemInstruction`, `AssistantRoleIsModel`, `FunctionResponseCarriesName` (`response: {"result": …}` or `{"error": …}`), `ReplaysThoughtSignature` (a model part with `thoughtSignature` comes back unchanged), `GeneratesIdsForCallsWithout`, `FinishReasons` (`SAFETY`→Refusal, `MAX_TOKENS`→MaxTokens; function call present → ToolUse), `PromptBlockedIsRefusal` (`promptFeedback.blockReason`), `ParsesErrorBodies`, `ParsesModelListFiltersGenerateContent`.
- [ ] Step 3: fail → implement → pass → commit.

### Task 4: Agent session

**Files:** Create `src/ai/include/horizon/ai/AgentSession.h`, `src/ai/src/AgentSession.cpp`, `src/ai/include/horizon/ai/SystemPrompt.h`, `src/ai/src/SystemPrompt.cpp`; Test `tests/ai/test_AgentSession.cpp`, `tests/ai/FakeTransport.h`.

**Interfaces (Produces):**
- `class AgentObserver { public: virtual ~AgentObserver(); virtual void onAssistantText(const std::string&) {} virtual void onToolCall(const std::string& name, const nlohmann::json& input) {} virtual void onToolResult(const std::string& name, const ToolResult&) {} virtual void onError(const std::string&) {} virtual void onFinished(const Usage&) {} };`
- `struct UndoHooks { std::function<std::size_t()> undoCount; std::function<std::uint64_t()> revision; std::function<void()> undo; };`
- `class AgentSession { AgentSession(HttpTransport&, const ToolRegistry&, AgentObserver&); void setConfig(ProviderConfig); const ProviderConfig& config() const; void setSystemPrompt(std::string); void setMaxSteps(int); void setUndoHooks(UndoHooks); bool send(const std::string& userText); void cancel(); bool busy() const; void reset(); const std::vector<Message>& messages() const; bool canUndoLastResponse() const; bool undoLastResponse(); };`
- `std::string defaultSystemPrompt();`
- `FakeTransport` (tests): queue of `HttpResponse`; `send` records the request and either replies immediately or holds it until `releaseNext()`.

- [ ] Step 1: tests — `TextOnlyTurn`, `ToolLoopRunsToolsAndReturnsResults` (2nd request carries the tool_result), `ParallelToolCallsInOneResultMessage`, `StepLimitStops` (error says the limit), `TransportErrorEndsTheRequest` (busy false after; send works again), `CancelIgnoresLateReply`, `SecondSendWhileBusyIsRefused`, `UndoResponseUndoesEveryAIChange`, `UndoResponseRefusedAfterUserEdit`, `RefusalIsReported`, `MaxTokensMidToolCallIsReported`, `ResetClearsTheConversation`, `ProviderSwitchDropsOpaqueBlocks` (via adapter behaviour).
- [ ] Step 2: fail → implement → pass → commit.

### Task 5: CAD tools

**Files:** Create `src/ai/include/horizon/ai/CadTools.h`, `src/ai/src/CadTools.cpp` (registration + shared helpers), `src/ai/src/CadQueryTools.cpp`, `src/ai/src/CadModelTools.cpp`, `src/ai/src/CadDraftTools.cpp`, `src/ai/src/ProfileBuilder.cpp/.h` (entities JSON → draft entities); Tests `tests/ai/test_CadTools.cpp`, `tests/ai/test_ProfileBuilder.cpp`.

**Interfaces (Produces):**
- `struct CadToolHooks { std::function<void()> modelChanged; std::function<void()> drawingChanged; };`
- `void registerCadTools(ToolRegistry&, doc::Document&, CadToolHooks = {});`
- `std::vector<std::shared_ptr<draft::DraftEntity>> buildEntities(const nlohmann::json& entities, std::string* error);` (ProfileBuilder; local 2D)
- Tool names and arguments exactly as spec §5.4.

- [ ] Step 1: ProfileBuilder tests — `Rectangle`, `CentredRectangle`, `Circle`, `Arc` (degrees), `ClosedPolyline`, `Slot` (two lines + two arcs, closed), `RegularPolygon`, `UnknownTypeIsAnError`, `NegativeRadiusIsAnError`.
- [ ] Step 2: CadTools tests — `SketchAndExtrudeMakeABox` (60×40 rect, extrude 5 → volume 12000 ± 1e-6), `CutThroughAll` (Ø10 circle cut → volume reduced by π·25·5), `ExtrudeOnXZPlane`, `RevolveMakesACylinder`, `PrimitiveWithPositionAndJoin`, `FilletEdgesFromListEdges` (top edges of a box chosen by normals; build succeeds; volume less than box), `ChamferEdges`, `ShellRemovesTopFace`, `HoleThroughAFace`, `LinearAndCircularPattern`, `MirrorAcrossPlane`, `EditFeatureChangesDistance` (degrees for an angle parameter), `VariablesDriveAParameter` (expression `depth` → rebuilt volume), `DeleteAndSuppressFeature`, `FailingFeatureIsWithdrawn` (fillet radius larger than the box → isError, feature count unchanged, undo count unchanged), `EachMutationIsOneUndoStep`, `Draw2dOnLayer` (layer created; entities on it), `TransformEntities`, `DeleteEntities`, `ListEdgesIsCapped`, `BadArgumentsAreErrorsNotCrashes` (table of bad inputs across tools), `MassPropertiesWithMaterial` (aluminium density 2.70 g/cm³ → mass in g), `DocumentInfoReportsBuildError`.
- [ ] Step 3: fail → implement → pass → full ctest → commit.

### Task 6: Phase 1 wrap-up

- [ ] Full build with `-DHZ_WARNINGS_AS_ERRORS=ON`; full ctest; clang-format-15 dry run on the whole tree; review (feature-dev:code-reviewer) on `src/ai`, `tests/ai`; fix; README Feature Maturity row "ai — experimental"; CHANGELOG entry; PR.

## Phase 2 — In-app assistant (PR 2)

### Task 7: Qt Network in the build

**Files:** Modify `vcpkg.json` (qtbase features `network`, `{ "name": "openssl", "platform": "linux" }`), top-level `CMakeLists.txt` (`find_package(Qt6 … Network)`), `src/app/CMakeLists.txt` (deploy `tls` plugins on Windows/macOS: `Qt6::QSchannelBackendPlugin`, `Qt6::QSecureTransportBackendPlugin`/`Qt6::QTlsBackendOpenSSLPlugin` when present), `packaging/linux/build-appimage.sh` if linuxdeploy needs telling.

- [ ] Configure + build locally; `ldd`/plugin listing shows tls backend; commit.

### Task 8: Transport, key store, settings

**Files:** Create `src/ui/include/horizon/ui/QtHttpTransport.h`, `src/ui/src/QtHttpTransport.cpp`, `src/ui/include/horizon/ui/AiSettings.h`, `src/ui/src/AiSettings.cpp`; Test `tests/ui/test_AiSettings.cpp`, `tests/ui/test_QtHttpTransport.cpp` (local `QTcpServer` answering one HTTP request).

**Interfaces:** `class QtHttpTransport : public QObject, public ai::HttpTransport { explicit QtHttpTransport(QObject* parent = nullptr); void setTimeoutMs(int); void send(...) override; void cancelAll() override; };` `struct AiSettings { ai::ProviderKind provider; std::map<ai::ProviderKind, ai::ProviderConfig> configs; int maxSteps = 24; static AiSettings load(); void save() const; static std::string apiKeyFor(ai::ProviderKind); static void storeApiKey(ai::ProviderKind, const std::string&); static QString keyFilePath(); };` Env names: `ANTHROPIC_API_KEY`, `OPENAI_API_KEY`, `GEMINI_API_KEY` then `GOOGLE_API_KEY`; key file owner read/write only.

- [ ] Tests: `TransportPostsAndReadsReply`, `TransportReportsConnectionRefused`, `TransportCancel`, `SettingsRoundTrip`, `KeyNeverInQSettings`, `EnvironmentKeyWins`, `KeyFileIsOwnerOnly` (POSIX). → implement → commit.

### Task 9: Panel, dialog, window wiring

**Files:** Create `src/ui/include/horizon/ui/AiAssistantPanel.h`, `src/ui/src/AiAssistantPanel.cpp`, `src/ui/include/horizon/ui/AiSettingsDialog.h`, `src/ui/src/AiSettingsDialog.cpp`; Modify `src/ui/src/MainWindow.cpp`/`.h` (create dock, View menu action, palette entry, ribbon button, Ctrl+Shift+I; `CadToolHooks.modelChanged` → refresh feature tree/sketches/scene without a second build; panel re-binds on active document change), `src/ui/CMakeLists.txt`; Test `tests/ui/test_AiAssistantPanel.cpp` (fake transport: a scripted tool call adds a box; transcript shows it; Undo this response removes it).

- [ ] Tests → implement → run the app (`--self-test`) → translations `lupdate` → commit → review → PR.

## Phase 3 — MCP server (PR 3)

### Task 10: `horizon-mcp`

**Files:** Create `src/ai/include/horizon/ai/McpServer.h`, `src/ai/src/McpServer.cpp`, `src/mcp/CMakeLists.txt`, `src/mcp/main.cpp`, `docs/ai/mcp.md`; Test `tests/ai/test_McpServer.cpp`.

**Interfaces:** `class McpServer { McpServer(ToolRegistry&, std::string serverName, std::string version); std::optional<nlohmann::json> handle(const nlohmann::json& message); void serve(std::istream&, std::ostream&); };` Document tools (`new_document`, `open_document`, `save_document`, `export`) registered by `main.cpp` through `registerDocumentTools(ToolRegistry&, DocumentHolder&)`.

- [ ] Tests: `InitializeNegotiatesVersion`, `ToolsListHasSchemas`, `ToolsCallReturnsContent`, `ToolErrorIsIsErrorResult`, `UnknownMethodIsMethodNotFound`, `NotificationGetsNoReply`, `ParseErrorReply`, `ServeLoopOverStringStreams`.
- [ ] Implement → docs (Claude Desktop / Claude Code config snippets) → commit → review → PR.
