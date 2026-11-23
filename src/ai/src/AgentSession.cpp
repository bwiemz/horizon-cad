#include "horizon/ai/AgentSession.h"

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
    if (m_busy) m_transport.cancelAll();
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
    m_changed = false;
    m_interleaved = false;
    if (m_undo.undoCount && m_undo.revision) {
        m_undoStart = m_undo.undoCount();
        m_lastRevision = m_undo.revision();
    }
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
    const auto adapter = makeAdapter(m_config.kind);
    const HttpRequest request = adapter->buildRequest(chat, m_config);

    const std::uint64_t generation = m_generation;
    const std::weak_ptr<bool> alive = m_alive;
    m_transport.send(request, [this, generation, alive](const HttpResponse& http) {
        const auto held = alive.lock();
        if (!held || !*held) return;  // the session is gone
        onReply(generation, http);
    });
}

void AgentSession::onReply(std::uint64_t generation, const HttpResponse& http) {
    if (generation != m_generation || !m_busy) return;  // stopped: a late reply
    const ChatResponse response = makeAdapter(m_config.kind)->parseResponse(http);
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
        runTools(response.message);
        if (generation != m_generation || !m_busy) return;  // stopped by a tool's host
        requestNext();
        return;
    }
    finish();
}

void AgentSession::runTools(const Message& message) {
    const bool tracked = m_undo.undoCount && m_undo.revision;
    if (tracked && m_undo.revision() != m_lastRevision) m_interleaved = true;

    Message results{Role::User, {}};
    for (const ContentBlock* call : message.toolCalls()) {
        m_observer.onToolCall(call->name, call->input);
        ToolResult result;
        const auto access = m_tools.access(call->name);
        if (access == ToolAccess::External && !(m_approve && m_approve(call->name, call->input))) {
            result = ToolResult::error("The user did not allow " + call->name + ".");
        } else {
            const std::size_t before = tracked ? m_undo.undoCount() : 0;
            result = m_tools.run(call->name, call->input);
            if (result.changedModel || (tracked && m_undo.undoCount() != before)) m_changed = true;
        }
        if (tracked) m_lastRevision = m_undo.revision();
        if (result.content.size() > kMaxResultChars) {
            result.content.resize(kMaxResultChars);
            result.content += "\n[cut: the result was longer than " +
                              std::to_string(kMaxResultChars) + " characters]";
        }
        m_observer.onToolResult(call->name, result);
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
    m_transport.cancelAll();
    finish();
}

void AgentSession::reset() {
    cancel();
    m_messages.clear();
    m_changed = false;
    m_interleaved = false;
    m_note.clear();
}

bool AgentSession::canUndoLastResponse() const {
    if (m_busy || !m_changed || m_interleaved) return false;
    if (!m_undo.undoCount || !m_undo.revision || !m_undo.undo) return false;
    return m_undo.revision() == m_lastRevision && m_undo.undoCount() > m_undoStart;
}

bool AgentSession::undoLastResponse() {
    if (!canUndoLastResponse()) return false;
    while (m_undo.undoCount() > m_undoStart) {
        const std::size_t before = m_undo.undoCount();
        m_undo.undo();
        if (m_undo.undoCount() >= before) break;  // the stack would not go back
    }
    m_changed = false;
    m_note =
        "(The user has undone every change you made in your previous response: the document is "
        "as it was before it.)";
    return true;
}

}  // namespace hz::ai
