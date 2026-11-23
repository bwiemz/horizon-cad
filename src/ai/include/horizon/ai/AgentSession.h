#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "horizon/ai/Conversation.h"
#include "horizon/ai/Http.h"
#include "horizon/ai/Provider.h"
#include "horizon/ai/ToolRegistry.h"

namespace hz::ai {

/// What a session tells the window as a request goes along. Called on the
/// transport's thread (the GUI thread in the application).
class AgentObserver {
public:
    virtual ~AgentObserver();
    virtual void onAssistantText(const std::string& text) { (void)text; }
    virtual void onToolCall(const std::string& name, const nlohmann::json& input) {
        (void)name;
        (void)input;
    }
    virtual void onToolResult(const std::string& name, const ToolResult& result) {
        (void)name;
        (void)result;
    }
    /// The request failed or was refused; @p message is for the user.
    virtual void onError(const std::string& message) { (void)message; }
    /// The request is over (done, failed, refused or stopped): the session is
    /// idle. @p usage is the request's tokens, all its round trips.
    virtual void onFinished(const Usage& usage) { (void)usage; }
};

/// How a session sees the document's undo stack, to take back a response's
/// changes as one action (see AgentSession::undoLastResponse).
struct UndoHooks {
    std::function<std::size_t()> undoCount;
    std::function<std::uint64_t()> revision;
    std::function<void()> undo;
};

/// A conversation with a model that can call tools: the user's request, then
/// as many round trips as the model needs (each tool call it makes is run
/// and its result sent back), until it ends its turn, the step limit is
/// reached, a request fails, or cancel() is called.
///
/// Asynchronous: send() returns at once. Not thread-safe: use it from the
/// thread that owns the transport.
class AgentSession {
public:
    AgentSession(HttpTransport& transport, const ToolRegistry& tools, AgentObserver& observer);
    ~AgentSession();

    AgentSession(const AgentSession&) = delete;
    AgentSession& operator=(const AgentSession&) = delete;

    /// The provider, model and key. Takes effect at the next round trip; the
    /// conversation is kept (another provider's opaque blocks are left out of
    /// what is sent).
    void setConfig(ProviderConfig config);
    const ProviderConfig& config() const { return m_config; }
    void setSystemPrompt(std::string prompt) { m_system = std::move(prompt); }
    /// Round trips per request, at least 1 (default 24).
    void setMaxSteps(int steps) { m_maxSteps = steps < 1 ? 1 : steps; }
    int maxSteps() const { return m_maxSteps; }
    void setUndoHooks(UndoHooks hooks) { m_undo = std::move(hooks); }
    /// Asked before an External tool runs; refused when unset.
    void setApprover(
        std::function<bool(const std::string& tool, const nlohmann::json& input)> ask) {
        m_approve = std::move(ask);
    }

    /// Start a request. False (and nothing sent) while one is running, for
    /// empty text, or with no model chosen (onError says so).
    bool send(const std::string& userText);
    /// Stop the request: what is in flight is abandoned and a late reply is
    /// ignored. Changes already made stay (Undo takes them back).
    void cancel();
    bool busy() const { return m_busy; }
    /// Forget the conversation.
    void reset();
    const std::vector<Message>& messages() const { return m_messages; }

    /// Whether the last request changed the document and nothing has been
    /// done to it since (by the user, or by an undo or redo): then all its
    /// changes can be taken back at once.
    bool canUndoLastResponse() const;
    /// Undo every change the last request made. False, changing nothing,
    /// unless canUndoLastResponse(). The model is told at the next request.
    bool undoLastResponse();

private:
    void requestNext();
    void onReply(std::uint64_t generation, const HttpResponse& http);
    void runTools(const Message& message);
    void finish();
    void fail(const std::string& message);

    HttpTransport& m_transport;
    const ToolRegistry& m_tools;
    AgentObserver& m_observer;
    ProviderConfig m_config;
    std::string m_system;
    int m_maxSteps = 24;
    UndoHooks m_undo;
    std::function<bool(const std::string&, const nlohmann::json&)> m_approve;

    std::vector<Message> m_messages;
    bool m_busy = false;
    int m_steps = 0;
    Usage m_usage;
    std::uint64_t m_generation = 0;
    std::shared_ptr<bool> m_alive = std::make_shared<bool>(true);

    // The last request's changes to the document.
    std::size_t m_undoStart = 0;
    std::uint64_t m_lastRevision = 0;
    bool m_changed = false;
    bool m_interleaved = false;  ///< the document changed between its tool calls
    std::string m_note;          ///< told to the model with the next request
};

}  // namespace hz::ai
