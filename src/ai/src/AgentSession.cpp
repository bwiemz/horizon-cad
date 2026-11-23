#include "horizon/ai/AgentSession.h"

#include <exception>
#include <utility>

namespace hz::ai {

namespace {

/// A tool result kept in the conversation is sent again with every later
/// round trip: one larger than this is cut, and says so.
constexpr std::size_t kMaxResultChars = 40000;

}  // namespace

AgentObserver::~AgentObserver() = default;

AgentSession::AgentSession(HttpTransport& transport, const ToolRegistry& tools,
                           AgentObserver& observer)
    : m_transport(transport), m_tools(tools), m_observer(observer) {}

AgentSession::~AgentSession() {
    *m_alive = false;
    if (m_busy) m_transport.cancel(m_inflight);
}

void AgentSession::setConfig(ProviderConfig config) {
    m_config = std::move(config);
}

bool AgentSession::send(const std::string& userText) {
    if (m_busy || userText.empty()) return false;
    if (m_config.model.empty()) {
        m_observer.onError("Choose a model in the assistant's settings first.");
        return false;
    }
    std::string text = userText;
    if (!m_note.empty()) {
        text = m_note + "\n\n" + text;
        m_note.clear();
    }
    m_messages.push_back({Role::User, {ContentBlock::makeText(std::move(text))}});
    m_busy = true;
    m_steps = 0;
    m_usage = {};
    m_aiSteps = 0;
    m_interleaved = false;
    if (m_undo.revision) m_lastRevision = m_undo.revision();
    requestNext();
    return true;
}

void AgentSession::requestNext() {
    if (m_steps >= m_maxSteps) {
        fail("Stopped after " + std::to_string(m_maxSteps) +
             " steps without finishing. Ask the assistant to continue, or raise the step limit in "
             "its settings.");
        return;
    }
    ++m_steps;
    ChatRequest chat;
    chat.model = m_config.model;
    chat.system = m_system;
    chat.messages = m_messages;
    chat.tools = m_tools.specs();
    chat.maxTokens = m_config.maxTokens;
    // Its reply is read in the format it was asked in, whatever the
    // settings are by the time it comes.
    m_requestKind = m_config.kind;
    const auto adapter = makeAdapter(m_requestKind);
    const HttpRequest request = adapter->buildRequest(chat, m_config);

    const std::uint64_t generation = m_generation;
    const std::uint64_t serial = ++m_requestSerial;
    const std::weak_ptr<bool> alive = m_alive;
    const HttpTransport::RequestId id =
        m_transport.send(request, [this, generation, alive](const HttpResponse& http) {
            const auto held = alive.lock();
            if (!held || !*held) return;  // the session is gone
            onReply(generation, http);
        });
    // A transport that answers at once has run the next round trip already,
    // whose request is the one in flight now.
    if (serial == m_requestSerial) m_inflight = id;
}

void AgentSession::onReply(std::uint64_t generation, const HttpResponse& http) {
    if (generation != m_generation || !m_busy) return;  // stopped: a late reply
    ChatResponse response;
    try {
        response = makeAdapter(m_requestKind)->parseResponse(http);
    } catch (const std::exception& e) {
        // The adapters do not throw; a reply they did not foresee still must
        // not reach the transport's event loop.
        response.stop = StopReason::Error;
        response.error = std::string("The reply could not be read: ") + e.what();
    }
    m_usage.inputTokens += response.usage.inputTokens;
    m_usage.outputTokens += response.usage.outputTokens;

    switch (response.stop) {
        case StopReason::Error:
            fail(response.error.empty() ? "The request failed." : response.error);
            return;
        case StopReason::Refusal:
            fail("The model declined this request" + (response.refusalDetail.empty()
                                                          ? std::string(".")
                                                          : ": " + response.refusalDetail));
            return;
        case StopReason::MaxTokens:
            // Not kept: a tool call it was cut off in may be incomplete, and
            // one left unanswered would make the next request invalid.
            if (const std::string text = response.message.text(); !text.empty()) {
                m_observer.onAssistantText(text);
            }
            fail("The reply was cut off at the token limit (" + std::to_string(m_config.maxTokens) +
                 "). Raise it in the assistant's settings, or ask for less at once.");
            return;
        default:
            break;
    }

    if (!response.message.blocks.empty()) m_messages.push_back(response.message);
    if (const std::string text = response.message.text(); !text.empty()) {
        m_observer.onAssistantText(text);
    }
    if (response.stop == StopReason::ToolUse && !response.message.toolCalls().empty()) {
        runTools(response.message, generation);
        if (generation != m_generation || !m_busy) return;  // stopped by a tool's host
        requestNext();
        return;
    }
    finish();
}

void AgentSession::runTools(const Message& message, std::uint64_t generation) {
    const bool tracked = static_cast<bool>(m_undo.revision);
    // Anything done to the document between the AI's own steps (an edit while
    // the model thinks, or while the user is asked about a tool) means its
    // steps are no longer the newest: they are not undone as one.
    const auto noticeEdits = [this, tracked] {
        if (tracked && m_undo.revision() != m_lastRevision) m_interleaved = true;
    };
    const auto stopped = [this, generation] { return generation != m_generation || !m_busy; };

    Message results{Role::User, {}};
    for (const ContentBlock* call : message.toolCalls()) {
        ToolResult result;
        if (stopped()) {
            // Stopped part-way: the rest are not run, but answered, so the
            // conversation stays one the provider accepts.
            result = ToolResult::error("Not run: the user stopped the request.");
        } else {
            noticeEdits();
            m_observer.onToolCall(call->name, call->input);
            const auto access = m_tools.access(call->name);
            const bool allowed =
                access != ToolAccess::External || (m_approve && m_approve(call->name, call->input));
            noticeEdits();
            if (stopped()) {
                result = ToolResult::error("Not run: the user stopped the request.");
            } else if (!allowed) {
                result = ToolResult::error("The user did not allow " + call->name + ".");
            } else {
                result = m_tools.run(call->name, call->input);
                // Counted, not read from the stack's depth: a stack at its
                // limit drops its oldest step as each new one comes.
                m_aiSteps += result.undoSteps;
            }
            if (tracked) m_lastRevision = m_undo.revision();
            if (result.content.size() > kMaxResultChars) {
                result.content.resize(kMaxResultChars);
                result.content += "\n[cut: the result was longer than " +
                                  std::to_string(kMaxResultChars) + " characters]";
            }
            m_observer.onToolResult(call->name, result);
        }
        results.blocks.push_back(
            ContentBlock::makeToolResult(call->id, call->name, result.content, result.isError));
    }
    m_messages.push_back(std::move(results));
}

void AgentSession::finish() {
    m_busy = false;
    m_observer.onFinished(m_usage);
}

void AgentSession::fail(const std::string& message) {
    m_busy = false;
    m_observer.onError(message);
    m_observer.onFinished(m_usage);
}

void AgentSession::cancel() {
    if (!m_busy) return;
    ++m_generation;
    m_transport.cancel(m_inflight);
    finish();
}

void AgentSession::reset() {
    cancel();
    m_messages.clear();
    m_aiSteps = 0;
    m_interleaved = false;
    m_note.clear();
}

bool AgentSession::canUndoLastResponse() const {
    if (m_busy || m_aiSteps <= 0 || m_interleaved) return false;
    if (!m_undo.undoCount || !m_undo.revision || !m_undo.undo) return false;
    return m_undo.revision() == m_lastRevision &&
           m_undo.undoCount() >= static_cast<std::size_t>(m_aiSteps);
}

bool AgentSession::undoLastResponse() {
    if (!canUndoLastResponse()) return false;
    for (int step = 0; step < m_aiSteps; ++step) {
        const std::size_t before = m_undo.undoCount();
        m_undo.undo();
        if (m_undo.undoCount() >= before) break;  // the stack would not go back
    }
    m_aiSteps = 0;
    m_note =
        "(The user has undone every change you made in your previous response: the document is "
        "as it was before it.)";
    return true;
}

}  // namespace hz::ai
