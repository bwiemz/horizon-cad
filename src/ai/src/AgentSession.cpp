#include "horizon/ai/AgentSession.h"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace hz::ai {

namespace {

/// A tool result kept in the conversation is sent again with every later
/// round trip: one larger than this is cut, and says so.
constexpr std::size_t kMaxResultChars = 40000;
/// No more images sent in one request than this, the newest: each request
/// carries the whole conversation, and an image costs as much as a page of
/// text every time it is sent. Older ones are said to have been left out.
constexpr std::size_t kMaxImagesSent = 3;

/// @p messages with the images of all but the newest @p limit left out, a
/// word in their place. A limit of none: a model that is not sent images
/// (the user turned them off, or switched to a model that reads none), which
/// would refuse a request carrying the ones shown before.
std::vector<Message> withRecentImages(std::vector<Message> messages, std::size_t limit) {
    std::size_t kept = 0;
    for (auto message = messages.rbegin(); message != messages.rend(); ++message) {
        for (auto block = message->blocks.rbegin(); block != message->blocks.rend(); ++block) {
            if (block->images.empty()) continue;
            const std::size_t room = limit - std::min(kept, limit);
            if (block->images.size() <= room) {
                kept += block->images.size();
                continue;
            }
            const std::size_t dropped = block->images.size() - room;
            block->images.erase(block->images.begin(),
                                block->images.begin() + static_cast<std::ptrdiff_t>(dropped));
            kept += block->images.size();
            block->text +=
                "\n[" + std::to_string(dropped) + " image(s) shown earlier are not sent again]";
        }
    }
    return messages;
}

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
    chat.messages = withRecentImages(m_messages, m_config.images ? kMaxImagesSent : 0);
    chat.tools = m_tools.specs();
    chat.maxTokens = m_config.maxTokens;
    // Its reply is read in the format it was asked in, whatever the
    // settings are by the time it comes.
    m_requestKind = m_config.kind;
    const auto adapter = makeAdapter(m_requestKind);
    // Streamed when it can be: the text shows as it is written.
    const bool stream = m_config.stream && m_transport.streams();
    const HttpRequest request = stream ? adapter->buildStreamingRequest(chat, m_config)
                                       : adapter->buildRequest(chat, m_config);

    const std::uint64_t generation = m_generation;
    const std::uint64_t serial = ++m_requestSerial;
    const std::weak_ptr<bool> alive = m_alive;
    m_shownPart = false;
    HttpTransport::RequestId id = 0;
    if (stream) {
        // Held by both callbacks; the transport lets them go when it is done.
        const std::shared_ptr<StreamReader> reader = adapter->makeStreamReader();
        id = m_transport.sendStreaming(
            request,
            [this, generation, alive, reader](std::string_view bytes) {
                const auto held = alive.lock();
                if (!held || !*held) return;
                if (generation != m_generation || !m_busy) return;  // stopped
                const std::string text = reader->feed(bytes);
                if (text.empty()) return;
                m_shownPart = true;
                m_observer.onAssistantTextPart(text);
            },
            [this, generation, alive, reader](const HttpResponse& http) {
                const auto held = alive.lock();
                if (!held || !*held) return;
                onReply(generation, http, reader.get());
            });
    } else {
        id = m_transport.send(request, [this, generation, alive](const HttpResponse& http) {
            const auto held = alive.lock();
            if (!held || !*held) return;  // the session is gone
            onReply(generation, http, nullptr);
        });
    }
    // A transport that answers at once has run the next round trip already,
    // whose request is the one in flight now.
    if (serial == m_requestSerial) m_inflight = id;
}

void AgentSession::onReply(std::uint64_t generation, const HttpResponse& http,
                           StreamReader* reader) {
    if (generation != m_generation || !m_busy) return;  // stopped: a late reply
    ChatResponse response;
    try {
        response = reader != nullptr ? reader->finish(http)
                                     : makeAdapter(m_requestKind)->parseResponse(http);
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
            // What came before it declined is not the answer.
            if (m_shownPart) m_observer.onAssistantTextWithdrawn();
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
        results.blocks.push_back(ContentBlock::makeToolResult(
            call->id, call->name, result.content, result.isError, std::move(result.images)));
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

void AgentSession::addNote(const std::string& note) {
    if (note.empty()) return;
    m_note = m_note.empty() ? note : m_note + "\n" + note;
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
